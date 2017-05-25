#include "i2c.h"
#include <xc.h>
//******************************************************************************
//* I2C_Initialize
//******************************************************************************
void I2C_Initialize(bool master,uint8_t addr)
{
    TRISCbits.TRISC0 = INPUT; /* SCL is on RC0 */
    TRISCbits.TRISC1 = INPUT; /* SDA is on RC1 */
    if(master)
    {
        __delay_ms(10);
        SSPCON = 0b00101000;   //SSP Module as Master
        SSPCON2 = 0x00;
        SSPADD = 119; // XTAL/(4*freq)-1) 119 = 100k baud
        SSPSTAT = 0x80;
        SSPCON3bits.SDAHT = 1; //hold time 300ns
        SSP1IF = 0;        //Clear interrupt flag
    }
    else
    { 
        __delay_ms(5);
        // Set it up as a slave but don't turn on the module
        SSPSTAT = 0x80;    
        SSPADD = addr << 1; //Starting address
        SSPCON = 0x16;  // 0x20 will actually enable it. //As a slave device
        SSPCON2 = 0x01;
        SSPCON3 = 0x00; //stop bit interrupt enable
        SSPCON3bits.BOEN = 1;
        SSPCON3bits.SDAHT = 1; //hold time 300ns
        SSPCON3bits.SBCDE = 1; //bus collision interrupt enable
        GIE = 1;          //Global interrupt enable
        PEIE = 1;         //Peripheral interrupt enable
        SSP1IF = 0;        //Clear interrupt flag
        BCL1IF = 0;
        BCL1IE = 1;
        SSP1IE = 1;        //Synchronous serial port interrupt enable
    }
}
//******************************************************************************
//* I2C_Master_Wait
//******************************************************************************
void I2C_Master_Wait()
{
  while(SSP1IF==0);
  SSP1IF=0;
}
//******************************************************************************
//* I2C_ACK - returns false if ack was not received
//******************************************************************************
bool I2C_ACK()
{
  return !SSPCON2bits.ACKSTAT;
}
//******************************************************************************
//* I2C_Master_Start - send start bit
//******************************************************************************
void I2C_Master_Start()
{
  SEN = 1;
  I2C_Master_Wait();
}
//******************************************************************************
//* I2C_Master_RepeatedStart - send repeated start bit
//******************************************************************************
void I2C_Master_RepeatedStart()
{
  RSEN = 1;
  I2C_Master_Wait();
}
//******************************************************************************
//* I2C_Master_Stop - send stop bit
//******************************************************************************
void I2C_Master_Stop()
{
  PEN = 1;
  I2C_Master_Wait();
}
//******************************************************************************
//* I2C_Master_Write - send byte
//******************************************************************************
void I2C_Master_Write(uint8_t d)
{
  SSPBUF = d;
  I2C_Master_Wait();
}
//******************************************************************************
//* I2C_Master_Read - get byte, argument send nack. Last byte in read is nacked
//******************************************************************************
uint8_t I2C_Master_Read(uint8_t nack)
{
  uint8_t temp;
  RCEN = 1;
  I2C_Master_Wait();
  temp = SSPBUF;
  ACKDT = (nack)?1:0;
  ACKEN = 1;
  I2C_Master_Wait();
  return temp;
}

