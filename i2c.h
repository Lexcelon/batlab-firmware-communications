#ifndef HAL_I2C
#define	HAL_I2C

#include <xc.h>
#include "hal_common.h"
//******************************************************************************
//* I2C exported functions
//******************************************************************************
void I2C_Initialize(bool master,uint8_t addr);
void I2C_Master_Wait();
bool I2C_ACK();
void I2C_Master_Start();
void I2C_Master_RepeatedStart();
void I2C_Master_Stop();
void I2C_Master_Write(uint8_t d);
uint8_t I2C_Master_Read(uint8_t nack);

#endif