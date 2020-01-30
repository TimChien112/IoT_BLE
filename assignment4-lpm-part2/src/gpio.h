/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>

void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();
void gpioTmpSenSetOn();
void gpioTmpSenSetOff();
void gpioI2CSetOn();
void gpioI2CSetOff();
#endif /* SRC_GPIO_H_ */
