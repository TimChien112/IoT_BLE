/*
 * state_machine.c
 *
 *  Created on: 2019¦~2¤ë27¤é
 *      Author: tim01
 */

#include "state_machine_params.h"
#include "em_core.h"
#include "sleep.h"
#include "I2C.h"
#include "log.h"

#define true	1
#define false	0

#define enable_Log	0
// state machine flag
	volatile STATE_T next_state = SENSOR_POWEROFF;
	volatile STATE_T current_state = SENSOR_POWEROFF;
	volatile uint8_t event = NO_EVENT;
	volatile uint8_t connection_flag = BLE_DISCONNECT;
state_stop_flag = 0;
void state_machine()
{
	//LOG_INFO("state_machine in case %d and next state is %d\n",current_state, next_state);
	switch(current_state)
	{
		case SENSOR_POWEROFF:
			if(event & LETIMER0_UF_FLAG)
			{
				TempSensorSet(true);
				CORE_CRITICAL_SECTION(
						event &= ~(LETIMER0_UF_FLAG);
				);
				Wait_usec_c(80000);
				next_state = SENSOR_WAITFORPOWERUP;

			}
			break;
		case SENSOR_WAITFORPOWERUP:
			if(event & LETIMER0_COMP1_FLAG)
			{
				I2CWrite_start();
				CORE_CRITICAL_SECTION(
						event &= ~(LETIMER0_COMP1_FLAG);
				);
				next_state = I2C_WAITFORWRITECOMPLETE;
			}
			break;
		case I2C_WAITFORWRITECOMPLETE:

			if(event & I2C_TRANSFER_COMPLETE)
			{
				I2CRead_start();
				CORE_CRITICAL_SECTION(
						event &= ~(I2C_TRANSFER_COMPLETE);
				);
				next_state = I2C_WAITFORREADCOMPLETE;
			}
			else if(event & I2C_TRANSFER_ERROR){
				CORE_CRITICAL_SECTION(
						event &= ~(I2C_TRANSFER_ERROR);
				);
				//turn off the sensor
				gpioI2CSetOff();
				SLEEP_SleepBlockEnd(sleepEM2);
				next_state = SENSOR_POWEROFF;
			}

			break;
		case I2C_WAITFORREADCOMPLETE:

			if(event & I2C_TRANSFER_COMPLETE)
			{
				gpioI2CSetOff();
				TempValueLog();
				SLEEP_SleepBlockEnd(sleepEM2);
				CORE_CRITICAL_SECTION(
						event &= ~(I2C_TRANSFER_COMPLETE);
				);
				next_state = SENSOR_POWEROFF;
			}
			else if(event & I2C_TRANSFER_ERROR){
				CORE_CRITICAL_SECTION(
						event &= ~(I2C_TRANSFER_ERROR);
				);
				SLEEP_SleepBlockEnd(sleepEM2);
				gpioI2CSetOff();
				next_state = SENSOR_POWEROFF;
			}
			break;
	}

	if((current_state != next_state) && (state_stop_flag==0)){
		gecko_external_signal(TEMPREAD_FLAG);
		if(enable_Log)LOG_INFO("State Transition:%d to %d",current_state,next_state);
		current_state = next_state;

	}
}


