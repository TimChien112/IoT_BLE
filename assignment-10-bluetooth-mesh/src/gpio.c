/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include <src/common.h>
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>
#include "log.h"


/**
 * TODO: define these.  See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
 * and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
 */
#define gpioPortF	5		  // Port F
#define	LED0_port 	gpioPortF //PF4
#define LED0_pin	4
#define LED1_port	gpioPortF //PF5
#define LED1_pin	5



void gpioInit()
{
	//GPIO_DriveStrengthSet(LED0_port,  gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
	//GPIO_DriveStrengthSet(LED1_port,  gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

	GPIOINT_Init();
	// for PB0
	GPIO_PinModeSet(gpioPortF, 6, gpioModeInputPull, 1);
	GPIO_IntConfig(gpioPortF,6,1,1,1);

	// for PB1
	GPIO_PinModeSet(gpioPortF, 7, gpioModeInputPull, 1);
	GPIO_IntConfig(gpioPortF,7,1,1,1);

	// for presence detector
	GPIO_PinModeSet(gpioPortF, 2, gpioModeInputPull, 1);
	GPIO_IntConfig(gpioPortF,2,1,1,1);
}

void GPIO_EVEN_IRQHandler(void)
{
	LOG_INFO("EVEN here!");
	uint32_t reason = GPIO_IntGet();
	GPIO_IntClear(reason);
	gecko_external_signal(PUSHBUTTON_FLAG);
}
void GPIO_ODD_IRQHandler(void)
{
	LOG_INFO("ODD here!");
	uint32_t reason = GPIO_IntGet();
	GPIO_IntClear(reason);
	gecko_external_signal(PUSHBUTTON_FLAG);
}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}
void gpioI2CSetOn()
{
	// turn on I2C GPIO
	GPIO_PinModeSet(gpioPortC,10,gpioModeWiredAndPullUp,true);
	GPIO_PinModeSet(gpioPortC,11,gpioModeWiredAndPullUp,true);
}
void gpioI2CSetOff()
{
	// turn off I2C GPIO
	GPIO_PinModeSet(gpioPortC,10,gpioModeDisabled,true);
	GPIO_PinModeSet(gpioPortC,11,gpioModeDisabled,true);
}

void gpioTmpSenSetOn()
{
	GPIO_PinOutSet(gpioPortD,15);
}
void gpioTmpSenSetOff()
{
	GPIO_PinOutClear(gpioPortD,15);
}
void TempSensorSet(bool evt)
{
	if(evt)	//Turn on Temperature sensor
	{
		gpioTmpSenSetOn();
	}
	else
	{
		gpioTmpSenSetOff();
	}
}
void gpioEnableDisplay()
{
//	GPIO_PinOutSet(gpioPortC,6);		//SI
//	GPIO_PinOutSet(gpioPortC,8);		//CLK
//	GPIO_PinOutSet(gpioPortD,13);		//EXTCOMIN
//	GPIO_PinOutSet(gpioPortD,14);		//CSC
	GPIO_PinOutSet(gpioPortD,15);		//Enable
}
void gpioSetDisplayExtcomin(bool high)
{
	if(high)
	{
		GPIO_PinOutSet(gpioPortD,13);		//EXTCOMIN off
	}
	else
	{
		GPIO_PinOutClear(gpioPortD,13);		//EXTCOMIN off
	}
}

