#include "hal.h"
#include "i2c.h"
#include <xc.h>
#include "usb.h"
#include <string.h>
#include "usb_config.h"
#include "usb_ch9.h"
#include "usb_cdc.h"
//******************************************************************************
//* GLOBALS
//******************************************************************************
uint16_t commregs[COMMREGS_SIZE] = {0};
uint8_t usb_tx_buf[USB_TX_BUF_SIZE]; //circular
uint8_t usb_tx_count = 0;
uint8_t usb_tx_buf_end = 0;
uint8_t usb_tx_buf_start = 0;
uint8_t usb_rx_buf[USB_RX_BUF_SIZE]; //circular
uint8_t usb_rx_count = 0;
uint8_t usb_rx_buf_end = 0;
uint8_t usb_rx_buf_start = 0;
uint8_t uart_tx_buf[UART_TX_BUF_SIZE]; //circular - exported to i2c.c for slave
uint8_t uart_tx_count = 0;
uint8_t uart_tx_buf_end = 0;
uint8_t uart_tx_buf_start = 0;
uint8_t uart_rx_buf[UART_RX_BUF_SIZE]; //circular - exported to i2c.c for slave
uint8_t uart_rx_count = 0;
uint8_t uart_rx_buf_end = 0;
uint8_t uart_rx_buf_start = 0;
uint8_t daisy_state = 0;
uint8_t *in_buf;                //used in service
uint8_t *out_buf;               //used in service
uint16_t i = 0;                  //temporary index variable
uint8_t usb_rx_temp_count = 0;  //used in service
uint8_t num_modules = 1;        //number of modules in the system self included
uint8_t i2c_tx_len = 0;         //i2c slave
static uint8_t i2c_reg_addr = 0;//i2c slave
static uint8_t rx_state = 0;    //i2c slave
static uint8_t pingval = 0;     //i2c slave
volatile uint32_t ctr = 0;
volatile uint8_t pwm_led[4] = {0};
const uint8_t SINETABLE[64] = {
12,13,14,15,16,17,18,19,
20,20,21,22,22,23,23,23,
23,23,23,22,22,21,21,20,
19,18,17,16,15,14,13,12,
11,10,9,8,7,6,5,4,
3,2,2,1,1,0,0,0,
0,0,0,1,1,2,3,3,
4,5,6,7,8,9,10,12
};
//******************************************************************************
//* UartTxByte - debug function to spit out a byte on uart
//******************************************************************************
void UartTxByte(uint8_t a)
{
    while(!TXSTAbits.TRMT);
    TXREG = a;
}
//******************************************************************************
//* two byte sequential copy
//******************************************************************************
void memcpy_2(void *dest,void *src)
{
    char *csrc = (char*)src;
    char *cdest = (char*)dest;
    cdest[0] = csrc[0];
    cdest[1] = csrc[1];
}
//******************************************************************************
//* initialize_usart - called by hardware_init
//******************************************************************************
static void initialize_usart(void)
{
    TRISCbits.TRISC5=1; /* RX on RC5 is an input */
    TRISCbits.TRISC4=0; /* TX on RC4 is an output */
	TXSTA = 0x24;
	RCSTA = 0x90;
	SPBRG  = 103; /* 115200 bps */
	SPBRGH = 0;
	BAUDCON = 0x08;
	(volatile void)RCREG; /* clear any data in receive buffer */
    RCIE = 1;
}
//******************************************************************************
//* hardware_init - Initializes hardware components and starts UART and USB
//******************************************************************************
void hardware_init(void)
{
    /* magic numbers from Microchip */
	OSCTUNE = 0;
	OSCCON = 0xFC; /* 16MHz HFINTOSC with 3x PLL enabled (48MHz operation) */
	ACTCON = 0x90; /* Enable active clock tuning with USB */

    initialize_usart();
    
    /*INIT all IOs*/
    ANSELA = 0x00; // not using any analog functionality
    ANSELC = 0x00;
    LED0_PIN  = HIGH;
    LED1_PIN  = HIGH;
    LED2_PIN  = HIGH;
    LED3_PIN  = HIGH;
    LED4_PIN  = HIGH;
    LED0_TRIS  = OUTPUT;
    LED1_TRIS  = OUTPUT;
    LED2_TRIS  = OUTPUT;
    LED3_TRIS  = OUTPUT;
    LED4_TRIS  = OUTPUT;
    
    EXT_PSU_TRIS = OUTPUT; //OUTPUT LOW turns the LED off
    EXT_PSU_PIN  = LOW;
    
    #ifdef USB_USE_INTERRUPTS
	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;
    #endif

    //set up tmr0 for 1464 hz
    OPTION_REGbits.TMR0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS0 = 1; 
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS2 = 0;
    INTCONbits.TMR0IE = 1;
    INTCONbits.TMR0IF = 0;
    
    //ensure the voltage reference is set up
    FVRCON = 0x83; //4.096V to ADC

	usb_init(); //USB stack function
    
    //User Initialization
    commregs[REG_PSU_CUTOFF_LOW] = 523;
    commregs[REG_PSU_CUTOFF_HIGH] = 630;
    commregs[REG_PSU_CUTOFF_HYST] = 5;
}

//******************************************************************************
//* service - routine to handle polled USB functions and send any pending
//*           UART or USB messages in the buffers. Also receives any UART or
//*           USB data and puts it into buffers
//******************************************************************************
void service(void)
{     
#ifndef USB_USE_INTERRUPTS
    usb_service(); //USB stack function
#endif
        
    /* TX to UART */
    /* if our TX buffer is not empty *AND* the USART can accept 
     * another byte, transmit another byte */
    if (TXSTAbits.TRMT && uart_tx_count)
    {
        TXREG = uart_tx_buf[uart_tx_buf_start];
        uart_tx_count--;
        uart_tx_buf_start = (uart_tx_buf_start + 1) % UART_TX_BUF_SIZE;
    }

    /* if USB isn't configured, there is no point in proceeding further */
    if (!usb_is_configured()){return;}

    /* TX to PC */
    /* proceed further only if the PC can accept more data */
    if (usb_in_endpoint_halted(2) || usb_in_endpoint_busy(2)){return;}
    /* if we've reached here, the USB stack can accept more; 
     * if we have data to send, we hand it over */
    if (usb_tx_count > 0)
    {
        in_buf = usb_get_in_buffer(2);
        for(i=0;i<usb_tx_count;i++)
        {
            in_buf[i] = usb_tx_buf[usb_tx_buf_start];
            usb_tx_buf_start = (usb_tx_buf_start + 1) % USB_TX_BUF_SIZE;
        }
        usb_send_in_buffer(2, usb_tx_count);
        usb_tx_count = 0;
    }

    /* RX FROM PC */
    /* if we pass this test, we are committed to make 
     * the usb_arm_out_endpoint() call */
    if (!usb_out_endpoint_has_data(2)){return;}
    /* ask USB stack for more PC2PIC data */
    usb_rx_temp_count = usb_get_out_buffer(2, &out_buf);
    /* if there was any, put it in the circular rx buffer */
    for(i=0;i<usb_rx_temp_count;i++)
    {
        usb_rx_buf[usb_rx_buf_end] = out_buf[i];
        usb_rx_buf_end = (usb_rx_buf_end+1) % USB_RX_BUF_SIZE;
        usb_rx_count++;
    }
    usb_arm_out_endpoint(2);
}

void service_leds(void)
{
    static uint8_t i = 0;
    for(i=0;i<4;i++)
    {
        switch(commregs[i] & 0x00FF) //for this to work, REG_LED0-REG_LED3 must be 0-3
        {
            case LED_OFF: pwm_led[i] = 0; break;
            case LED_BLIP: 
                pwm_led[i] = (ctr < 3) ? 24 : 0; break;
            case LED_FLASH_SLOW: 
                pwm_led[i] = (ctr < 128) ? 24 : 0; break;
            case LED_FLASH_FAST: 
                if(ctr < 43) {pwm_led[i] = 24;}
                else if (ctr < 85) {pwm_led[i] = 0;}
                else if (ctr < 128) {pwm_led[i] = 24;}
                else if (ctr < 171) {pwm_led[i] = 0;}
                else if (ctr < 214) {pwm_led[i] = 24;}
                else                {pwm_led[i] = 0;}
                break;
            case LED_PWM: pwm_led[i] = commregs[i] >> 8;
            case LED_ON: pwm_led[i] = 24; break;
            case LED_RAMP_UP: pwm_led[i] = ctr / 11; break;
            case LED_RAMP_DOWN: pwm_led[i] = 24 - (ctr / 11); break;
            case LED_SINE: pwm_led[i] = SINETABLE[ctr >> 2]; break;
        }      
    }    
}

void service_psu(void)
{
    static uint16_t ctr = 0;
    uint32_t result = 0;
    uint8_t current_psu_state = EXT_PSU_PIN;
    
    //only update the measurement every 10000th time ~ 0.5 seconds
    if( ctr < 10000 )
    {
        ctr++;
        return;
    }
    ctr = 0;
    
    
    //take analog measurement
    ADCON0 = 0b00011101; // set channel to an7
    ADCON1 = 0b11100011; //rjustify,fosc/64,FVR REF
    
    EXT_PSU_PIN  = LOW;
    for(i=0;i<500;i++);
    EXT_PSU_TRIS = INPUT; 
    ANSELCbits.ANSC3 = 1; // turn on analog functionality to the psu pin
    for(i=0;i<1200;i++);
    for(i=0;i<128;i++)
    {
        ADCON0bits.ADGO = 1; //start the conversion
        while(ADCON0bits.ADGO); //wait for it to complete
        result += (ADRESH << 8) + ADRESL;
    }
    result >>= 7;
    commregs[REG_PSUVOLTAGE] = result;
    //for(i=0;i<128;i++);
    //Anything higher than 5.2V (measured 2.15V since it is a divide-by-2 circuit, minus diode drop)
    //is not supported due to the inability to measure voltage when the psu is that high
    //Anything lower than 4.4V (measured 1.76V) is not supported due to not being able to charge batteries to 100% soc
    // 5.2V --> 440
    // 4.4V --> 360
    //if(current_psu_state == HIGH && (result > 545 || result < 440))
    if( result > 360 )
    {
        ANSELCbits.ANSC3 = 0;
        EXT_PSU_TRIS = OUTPUT; 
        EXT_PSU_PIN  = HIGH;
        for(i=0;i<1200;i++); //make sure it stays on long enough to mask the measurement as an intentional blink
    }  

    if(current_psu_state == HIGH && (result > commregs[REG_PSU_CUTOFF_HIGH] || result < commregs[REG_PSU_CUTOFF_LOW])) 
    {
        static uint8_t accident_forgiveness = true;
        if(accident_forgiveness == true)
        {
            accident_forgiveness = false;
        }
        else
        {
            accident_forgiveness = true;
            EXT_PSU_PIN  = LOW; //PSU not detected or out of range
        }
    }
    //else if (current_psu_state == LOW && (result < 540 && result > 445)) //deadband built in for PSU detection
    else if (current_psu_state == LOW && (result < (commregs[REG_PSU_CUTOFF_HIGH] - commregs[REG_PSU_CUTOFF_HYST])  && result > (commregs[REG_PSU_CUTOFF_LOW] + commregs[REG_PSU_CUTOFF_HYST])))
    {
        EXT_PSU_PIN  = HIGH; //PSU Detected
    }
    else
    {
        EXT_PSU_PIN = current_psu_state;
    }
    
    //set the pin back to output so we can display status on the LED
    ANSELCbits.ANSC3 = 0;
    EXT_PSU_TRIS = OUTPUT;  
}

//******************************************************************************
//* get_usb_packet - reads usb buffer until packet is found then fills packet
//*                  Returns length of packet - 0 if no packet
//******************************************************************************
uint8_t get_usb_packet(packet* p)
{
    //search through the buffer until we find a packet
    while(usb_rx_buf[usb_rx_buf_start] != PACKET_START_BYTE)
    {
        if(usb_rx_count < USB_RX_PACKET_LEN) 
        {
            return false; //full packet not in buffer
        } 
        usb_rx_count--;
        usb_rx_buf_start = (usb_rx_buf_start + 1) % USB_RX_BUF_SIZE; 
    }
    if(usb_rx_count < USB_RX_PACKET_LEN) 
    {
        return false;
    } 
    p->len = USB_RX_PACKET_LEN;  //if we got here then there is a full packet
    for(i=0;i<p->len;i++)
    {
        p->payload[i] = usb_rx_buf[usb_rx_buf_start];
        usb_rx_buf_start = (usb_rx_buf_start + 1) % USB_RX_BUF_SIZE;
        usb_rx_count--;
    }
    return p->len;
}
//******************************************************************************
//* generate_global_response  - interprets packet meant for COMMS pic and
//*                             does an appropriate action and responds to USB
//******************************************************************************
void process_packet(packet* response,packet* command)
{
    uint8_t addr = command->payload[2] & 0x7F;
    //parse the command and do what it says
    //respond to the command (See BATLAB command spec)
    response->len = 5;
    response->payload[0] = 0xAA;
    response->payload[1] = command->payload[1]; //Parrot of Destination Address
    response->payload[2] = command->payload[2]; //Parrot of Register Address
    response->payload[3] = 0x01; //fail
    response->payload[4] = 0x01; //fail
    
    
    if( command->payload[2] & 0x80 ) //Message was a master write
    {
        //response->len = 0; // don't actually respond because it takes up bandwidth
        //uint8_t success = true;
        //Handle LED Writes
        if(addr < COMMREGS_SIZE) //map the cell led numbers to the schematic led numbers
        {
            memcpy_2( &(commregs[addr]) , &(command->payload[3]) );
            //case REG_LED0: LED4_PIN = !!command->payload[4]; break;
            //case REG_LED1: LED0_PIN = !!command->payload[4]; break;
            //case REG_LED2: LED3_PIN = !!command->payload[4]; break;
            //case REG_LED3: LED2_PIN = !!command->payload[4]; break;
            //default: success = false;
            response->payload[3] = 0x00; // success
            response->payload[4] = 0x00; //  success
        }
        else
        {
            response->payload[3] = 0x01; // fail
            response->payload[4] = 0x01; //  fail
        }
    }
    else                             //Message was a master read
    {
        if(addr < COMMREGS_SIZE)
        {
            //read the state of the PSU pin
            commregs[REG_PSU] = EXT_PSU_PIN;
            memcpy_2( &(response->payload[3]) , &(commregs[addr]) );
        }
        else
        {
            response->payload[3] = 0x01; //fail
            response->payload[4] = 0x01; //fail
        } 
    }
}
//******************************************************************************
//* send_usb_packet - adds packet to outgoing USB buffer
//******************************************************************************
void send_usb_packet(packet* p)
{
    for(i=0;i<p->len;i++)
    {
        usb_tx_buf[usb_tx_buf_end] = p->payload[i];
        usb_tx_buf_end = (usb_tx_buf_end+1) % USB_TX_BUF_SIZE;
        usb_tx_count++;
    } 
}
//******************************************************************************
//* send_uart_packet - adds packet to outgoing UART buffer
//******************************************************************************
void send_uart_packet(packet* p)
{
    for(i=0;i<p->len;i++)
    {
        uart_tx_buf[uart_tx_buf_end] = p->payload[i];
        uart_tx_buf_end = (uart_tx_buf_end+1) % UART_TX_BUF_SIZE;
        uart_tx_count++;
    } 
}
//******************************************************************************
//* send_i2c_packet - sends packet to given I2C slave
//******************************************************************************
void send_i2c_packet(packet* p,uint8_t slave)
{
    I2C_Master_Start();
    I2C_Master_Write(slave << 1);
    I2C_Master_Write(SLAVE_RX_ADDR);
    I2C_Master_Write(p->len);
    for(i=0;i<p->len;i++)
    {
        I2C_Master_Write(p->payload[i]);
    }
    I2C_Master_Stop();
}
//******************************************************************************
//* send_i2c_message - sends message to I2C slave (See Batlab I2C spec)
//*                    Returns false if no response from slave
//******************************************************************************
bool send_i2c_message(uint8_t addr,uint8_t register_addr,uint8_t data)
{
    I2C_Master_Start();
    I2C_Master_Write(addr << 1);
    I2C_Master_Write(register_addr);
    if(!I2C_ACK()) {return false;}
    I2C_Master_Write(data);
    I2C_Master_Stop();
    return true;
}
//******************************************************************************
//* get_i2c_message - get message from I2C slave (See Batlab I2C spec)
//*                    
//******************************************************************************
uint8_t get_i2c_message(uint8_t addr,uint8_t register_addr)
{
    uint8_t retval;
    I2C_Master_Start();
    I2C_Master_Write( (addr << 1) );
    I2C_Master_Write(register_addr);
    I2C_Master_RepeatedStart();
    I2C_Master_Write( (addr << 1) | 0x01);
    retval = I2C_Master_Read(1);
    I2C_Master_Stop();
    return retval;
}
//******************************************************************************
//* get_i2c_packet - gets packet from slave, Returns length - 0 if no packet
//******************************************************************************
uint8_t get_i2c_packet(packet* p,uint8_t slave)
{
    uint8_t temp;
    temp = get_i2c_message(slave,SLAVE_TX_LEN_ADDR);
    if(!temp) {return false;} //temp=0 if no packet and = len if packet ready
    p->len = temp;
    I2C_Master_Start();
    I2C_Master_Write(slave << 1);
    I2C_Master_Write(SLAVE_TX_ADDR);
    I2C_Master_RepeatedStart();
    I2C_Master_Write((slave<<1) | 0x01 );
    for(i=0;i<(p->len - 1);i++)
    {
        p->payload[i] = I2C_Master_Read(0);
    }
    p->payload[p->len - 1] = I2C_Master_Read(1);
    I2C_Master_Stop();
    return temp; //no incoming byte to read
}
//******************************************************************************
//* check_for_uart_packet - reads buffer for uart packet - returns length or 0
//******************************************************************************
uint8_t check_for_uart_packet()
{
    //search through the buffer until we find a packet
    while(uart_rx_buf[uart_rx_buf_start] != PACKET_START_BYTE && uart_rx_buf[uart_rx_buf_start] != EXT_PACKET_START_BYTE )
    {
        if(uart_rx_count < USB_RX_PACKET_LEN) 
        {
            return false; //full packet not in buffer
        } 
        uart_rx_count--;
        uart_rx_buf_start = (uart_rx_buf_start + 1) % UART_RX_BUF_SIZE; 
    }
    if (uart_rx_buf[uart_rx_buf_start] == EXT_PACKET_START_BYTE)
    {
        if(uart_rx_count < EXT_PACKET_LEN) {return false;}
        return EXT_PACKET_LEN;
    }
    else
    {
        if(uart_rx_count < USB_RX_PACKET_LEN) {return false;}
        return USB_RX_PACKET_LEN;
    }
}
//******************************************************************************
//* get_uart_packet - read found packet into buffer - return length or 0 if none
//******************************************************************************
uint8_t get_uart_packet(packet* p)
{
    p->len = check_for_uart_packet(); //returns 0 if no packet
    
    for(i=0;i<p->len;i++)
    {
        p->payload[i] = uart_rx_buf[uart_rx_buf_start];
        uart_rx_buf_start = (uart_rx_buf_start + 1) % UART_RX_BUF_SIZE;
        uart_rx_count--;
    }
    return p->len;
}
//******************************************************************************
//* i2c_rx_handler - state machine to control i2c slave reg reads-called in ISR
//******************************************************************************
void i2c_rx_handler(uint8_t rx)
{
    return;
}
//******************************************************************************
//* i2c_tx_handler - controls slave writes - called in ISR
//******************************************************************************
uint8_t i2c_tx_handler()
{
    return 0;
}
//******************************************************************************
//* UART_Receive_ISR - called in ISR
//******************************************************************************
void UART_Receive_ISR(void)
{
    if(RCSTAbits.OERR)
    {
        RCSTAbits.SPEN = 0;
        RCSTAbits.SPEN = 1;
    }
    /* RX from UART */
    /* if the USART has received another byte, add it to the queue */
    if (RCSTAbits.OERR){RCSTAbits.CREN = 0;} /* reset port if overrun */
    uart_rx_buf[uart_rx_buf_end] = RCREG;
    uart_rx_buf_end = (uart_rx_buf_end + 1) % UART_RX_BUF_SIZE;
    uart_rx_count++;
    RCSTAbits.CREN = 1;  /* and then (re-)enable receive */
}
//******************************************************************************
//* interrupt handler for I2C slave and USB stack
//******************************************************************************
void interrupt isr()
{
  static uint8_t dummy;
  /*if(0)//SSP1IF && SSP1IE)
  { 
    LED0_PIN    = HIGH;
    if(!SSPSTATbits.R_nW) //slave receiving
    {
        if(!SSPSTATbits.D_nA) //Address
        {
            dummy = SSPBUF; //clear the buffer 
        }
        else //data
        {
            i2c_rx_handler(SSPBUF);
        }
        if (SSPCONbits.WCOL) 
        {
            SSPCONbits.WCOL = 0;  // Clear the collision bit
            dummy = SSPBUF; // Read the previous value to clear the buffer
        }
        SSPCONbits.CKP = 1;
    }
    else if(SSPSTATbits.R_nW) //slave sending
    {
        if(!SSPSTATbits.D_nA)
        {
            dummy = SSPBUF; //clear the buffer
        } 
        while(BF);
        SSPBUF = i2c_tx_handler(); //send data out
        SSPCONbits.CKP = 1;    
    }  
    SSP1IF = 0;
    LED0_PIN    = LOW;
  }*/
  if(BCL1IF && BCL1IE)
  {
      dummy = SSPBUF;
      BCL1IF = 0;
      SSPCONbits.CKP = 1;
  }
  if(RCIF & RCIE)
  {
      UART_Receive_ISR();
      RCIF = 0;
  }
  if(TMR0IF && TMR0IE) //1464 Hz 61 * 24. if we have 24 discrete levels, then we can do 61 counts per second
  {
      static uint8_t pwmctr = 0;
      static uint8_t ctr2 = 0;
      LED4_PIN = (pwmctr >= pwm_led[0]) ? 1 : 0;
      LED0_PIN = (pwmctr >= pwm_led[1]) ? 1 : 0;
      LED3_PIN = (pwmctr >= pwm_led[2]) ? 1 : 0;
      LED2_PIN = (pwmctr >= pwm_led[3]) ? 1 : 0;
      pwmctr++;
      if(pwmctr > 23){pwmctr = 0;ctr2++;}
      if(ctr2 > 1){ctr2 = 0;ctr++;}
      if(ctr > 255){ctr = 0;}
      TMR0IF = 0;
  }
#ifdef USB_USE_INTERRUPTS
    usb_service(); //USB stack function
#endif
}



