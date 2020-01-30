/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>

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
