#ifndef HAL_H
#define	HAL_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <xc.h>
#include "usb.h"
#include "hal_common.h"
//******************************************************************************
//* APPLICATION DEFINES
//******************************************************************************
#define PACKET_START_BYTE 0xAA
#define EXT_PACKET_START_BYTE 0xAF
#define USB_RX_PACKET_LEN 5
#define EXT_PACKET_LEN 13

#define NAMESPACE_COMMS 0xFF
    
 
#define REG_LED0               0x00 //we rely on these 4 being equivalent 
#define REG_LED1               0x01 //to cell index. this speeds things 
#define REG_LED2               0x02 //up quite a bit
#define REG_LED3               0x03
#define REG_PSU                0x04
#define REG_PSUVOLTAGE         0x05
#define REG_PSU_CUTOFF_LOW     0x06
#define REG_PSU_CUTOFF_HIGH    0x07
#define REG_PSU_CUTOFF_HYST    0x08
#define COMMREGS_SIZE          0x09

#define LED_OFF                0x0000 
#define LED_BLIP               0x0001
#define LED_FLASH_SLOW         0x0002
#define LED_FLASH_FAST         0x0003
#define LED_ON                 0x0004
#define LED_PWM                0x0005 //High byte used for PWM setpoint
#define LED_RAMP_UP            0x0006
#define LED_RAMP_DOWN          0x0007
#define LED_SINE               0x0008
       
#define EXT_PSU_PIN (PORTCbits.RC3)
#define LED0_PIN    (PORTCbits.RC2)   //cell1
#define LED1_PIN    (PORTAbits.RA5)   //COMMS
#define LED2_PIN    (PORTAbits.RA4)   //cell3
#define LED3_PIN    (PORTCbits.RC0)   //cell2
#define LED4_PIN    (PORTCbits.RC1)   //cell0
    
#define EXT_PSU_TRIS (TRISCbits.TRISC3)
#define LED0_TRIS    (TRISCbits.TRISC2)   //cell1
#define LED1_TRIS    (TRISAbits.TRISA5)   //COMMS
#define LED2_TRIS    (TRISAbits.TRISA4)   //cell3
#define LED3_TRIS    (TRISCbits.TRISC0)   //cell2
#define LED4_TRIS    (TRISCbits.TRISC1)   //cell0
    
#define USB_TX_BUF_SIZE 0x80
#define USB_RX_BUF_SIZE 0x80
#define UART_TX_BUF_SIZE 0x80
#define UART_RX_BUF_SIZE 0x80

#define MAX_PACKET_LENGTH 15

typedef struct packets
{
    uint8_t len;
    uint8_t payload[MAX_PACKET_LENGTH];
} packet;
//******************************************************************************
//* I2C DEFINES
//******************************************************************************
#define SLAVE_RX_ADDR     0x00
#define SLAVE_TX_ADDR     0x01
#define SLAVE_TX_LEN_ADDR 0x02
#define SLAVE_ADDR_ADDR   0x03
#define SLAVE_DAISY_ADDR  0x04
#define SLAVE_PING_ADDR   0x05
#define SLAVE_DAISY_LOW   0x00
#define SLAVE_DAISY_HIGH  0xFF
#define UNINITIALIZED_SLAVE_ADDR 0x7F
#define RX_STATE_IDLE 0x00
#define RX_STATE_CMD_READY 0x01
#define RX_STATE_SLAVE_RX 0x02
//******************************************************************************
//* FUNCTIONS
//******************************************************************************
void UartTxByte(uint8_t a);
uint8_t get_usb_packet(packet* p);
uint8_t get_uart_packet(packet* p); 
uint8_t get_i2c_packet(packet* p,uint8_t slave);
void process_packet(packet* response,packet* command);
void send_uart_packet(packet* p);
void send_i2c_packet(packet* p,uint8_t slave);
void send_usb_packet(packet* p);
void slave_poll_daisy(void);
void service(void);
void service_leds(void);
void service_psu(void);
void hardware_init(void);
uint8_t check_for_uart_packet();
uint8_t arbitrate(void);
void i2c_rx_handler(uint8_t rx);
uint8_t i2c_tx_handler();
//******************************************************************************
//* EXPORTED VARIABLES
//******************************************************************************
extern uint8_t num_modules;
extern uint8_t i2c_tx_len;         //i2c slave

#ifdef	__cplusplus
}
#endif

#endif	/* HAL_H */

