/*
 * presence_sensor.c
 *
 *  Created on: 2019¦~4¤ë22¤é
 *      Author: tim01
 */


#include "presence_sensor.h"


void presence_init()
{
	presence_state = 0;
	GPIO_PinModeSet(PRESENCE_FORDWARD_ENABLE_PORT , PRESENCE_FORDWARD_ENABLE_PIN, gpioModeInputPull, 0);
	GPIO_PinModeSet(PRESENCE_BACKWARD_ENABLE_PORT , PRESENCE_BACKWARD_ENABLE_PIN, gpioModeInputPull, 0);
}

void presence_enable()
{
	 GPIO_PinModeSet(PRESENCE_FORDWARD_ENABLE_PORT , PRESENCE_FORDWARD_ENABLE_PIN, gpioModeInputPull, 1);
	 GPIO_PinModeSet(PRESENCE_BACKWARD_ENABLE_PORT , PRESENCE_BACKWARD_ENABLE_PIN, gpioModeInputPull, 1);
	 enable_presence_interrupt();
}




void presence_disable()
{
	disable_presence_interrupt();
	GPIO_PinModeSet(PRESENCE_FORDWARD_ENABLE_PORT , PRESENCE_FORDWARD_ENABLE_PIN, gpioModeDisabled, 0);
	GPIO_PinModeSet(PRESENCE_BACKWARD_ENABLE_PORT , PRESENCE_BACKWARD_ENABLE_PIN, gpioModeInputPull, 0);
}

//code for button interrupts--working but is commented due to use of callback register.

void enable_presence_interrupt()
{

	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	GPIO_ExtIntConfig(PRESENCE_FORDWARD_ENABLE_PORT, PRESENCE_FORDWARD_ENABLE_PIN, PRESENCE_FORDWARD_ENABLE_PIN, true, false, true);
	GPIO_ExtIntConfig(PRESENCE_BACKWARD_ENABLE_PORT, PRESENCE_BACKWARD_ENABLE_PIN, PRESENCE_BACKWARD_ENABLE_PIN, true, false, true);
}

void disable_presence_interrupt()
{
	GPIO_PinModeSet(PRESENCE_FORDWARD_ENABLE_PORT, PRESENCE_FORDWARD_ENABLE_PIN, gpioModeDisabled, 0);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

}
