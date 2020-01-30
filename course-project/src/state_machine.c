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
#include "gatt_db.h"
#include "display.h"
#include "infrastructure.h"
#include "common.h"
#include "mesh_custom_model_map.h"
//#include "gecko_native.h"

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


void TempValueLog(void)
{
	uint8_t htmTempBuffer[5]; 	/* Stores the temperature data in the Health Thermometer (HTM) format. */
	uint8_t flags = 0x00;   	/* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
	int32_t temp;     			/* Stores the Temperature data read from the RHT sensor. */
	uint32_t temperature;   	/* Stores the temperature data read from the sensor in the correct format */
	uint8_t *p = htmTempBuffer; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */
	UINT8_TO_BITSTREAM(p, flags);


	//print out the temperature value
	temp = read_buffer_data[0];
	temp = temp<<8;
	temp |= read_buffer_data[1];
	float final_temp = (175.72*((float)temp)/65536)-46.85;
	if(final_temp >= 32){
		window_state = BUTTON_0_PRESS;
		LOG_INFO("Window opened %d \n",window_state);
		displayPrintf(DISPLAY_ROW_TEMPVALUE, "Window Opened");
	}
	LOG_INFO("Read temperature  %04f\n",final_temp);
}

