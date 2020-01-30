/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>


// external event flag
#define PUSHBUTTON_FLAG 0x13
#define BP1_PIN_INDEX 	128
#define BP0_PIN_INDEX 	64
#define F_PRESENCE_PIN_INDEX 8
#define B_PRESENCE_PIN_INDEX 1024
#define B_PRESENCE_FLAG 	1<<4
#define F_PRESENCE_FLAG 	1<<3
#define BP0_FLAG 1<<0
#define BP1_FLAG 1<<1
#define SM_FLAG  1<<2
#define BR_FLAG  1<<3

#define GPIO_SET_DISPLAY_EXT_COMIN_IMPLEMENTED 	1
#define GPIO_DISPLAY_SUPPORT_IMPLEMENTED		1



void gpioInit();
void GPIO_ODD_IRQHandler(void);
void GPIO_EVEN_IRQHandler(void);
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();
void gpioTmpSenSetOn();
void gpioTmpSenSetOff();
void gpioI2CSetOn();
void gpioI2CSetOff();
void TempSensorSet(bool evt);
void gpioEnableDisplay();
void gpioSetDisplayExtcomin(bool high);
#endif /* SRC_GPIO_H_ */
