/*
 * ambient_light.h
 *
 *  Created on: Apr 20, 2019
 *      Author: santhosh
 */
//#include <stdbool.h>
//#include "em_cmu.h"
//#include "em_gpio.h"
//#include "em_i2c.h"
//#include "i2cspm.h"
//
//
//
#ifndef SRC_AMBIENT_LIGHT_H_
#define SRC_AMBIENT_LIGHT_H_
//
//#define AMBIENT_LIGHT_ENABLE_PORT gpioPortF
//#define AMBIENT_LIGHT_ENABLE_PIN  3
//#define AMBIENT_LIGHT_ADDRESS 0x39
//#define DEVICE_ID_REGISTER    0x0A
//#define MANUFACTURER_ID_REGISTER 0x7E
//
//
//void ambient_light_init();
//void ambient_light_enable();
//void Ambient_Light_I2C_Init();
//void i2c_read_ambient_light_reg(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t reg_addr);
//void i2c_write_ambient_light_reg(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t reg_addr);
//
//
#include "em_i2c.h"
#include "em_cmu.h"
#include "timer0.h"


#define SLAVE_ADDRESS_TSL2561         (0x39)

#define TSL2561_Command_register      (0x80)
#define TSL2561_Word_register         (0x20)
#define TSL2561_Control_Power_Up      (0x03)
#define TSL2561_Control_Power_Down    (0x00)
#define TSL2561_Register_Control      (0x00)

#define TSL2561_CH0_Low               (0x0c)
#define TSL2561_CH0_High              (0x0d)
#define TSL2561_CH1_Low               (0x0e)
#define TSL2561_CH1_High              (0x0f)

void I2C0_init(void);
uint8_t I2C0_readByte(uint8_t slave_addr,uint8_t reg_addr);
void I2C0_writeByte(uint8_t slave_addr,uint8_t reg_addr,uint8_t data);
double getLuminosityValue();
void lum_enable();

#endif /* SRC_AMBIENT_LIGHT_H_ */
