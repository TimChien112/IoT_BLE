/*
 * I2C.c
 *
 *  Created on: 2019¦~2¤ë27¤é
 *      Author: tim01
 */

#include "em_i2c.h"
#include "I2C.h"
#include "sleep.h"
#include "state_machine_params.h"
#include "log.h"

// parameters for I2C transfer function
	I2C_TransferSeq_TypeDef seq;
	uint8_t write_buffer_data = 0xE3;
	//uint8_t write_buffer_data = 0x8A;			//test leghtness sensor
	uint16_t write_buffer_len = 1;
	uint8_t read_buffer_data[2];
	uint16_t read_buffer_len  = 2;

void I2CWrite_start(void)
{
	gpioI2CSetOn();
	NVIC_EnableIRQ(I2C0_IRQn);
	//LOG_INFO("Start writing\n");
	//write command to Si7021
	seq.flags=I2C_FLAG_WRITE;
	seq.buf[0].data = &write_buffer_data;
	seq.buf[0].len = write_buffer_len;
	SLEEP_SleepBlockBegin(sleepEM2);
	I2C_TransferInit(I2C0,&seq);
	//I2C only work in EM1
}

void I2CRead_start(void)
{
	NVIC_EnableIRQ(I2C0_IRQn);
	//LOG_INFO("Start Reading\n");
	//read data from Si7021
	seq.flags=I2C_FLAG_READ;
	seq.buf[0].data = read_buffer_data;
	seq.buf[0].len = read_buffer_len;
	I2C_TransferInit(I2C0,&seq);
}

// Initialize I2CSPM & seq.device_address
void i2c_Init(void)
{
	gpioTmpSenSetOff();
	uint16_t device_address = 0x40;
	//uint16_t device_address = 0x39;  // test lightness sensor
	seq.addr = device_address<<1;
}

void I2C0_IRQHandler(void)
{
	I2C_TransferReturn_TypeDef ret = I2C_Transfer(I2C0);

	if(ret == i2cTransferDone)
		{
			event |= I2C_TRANSFER_COMPLETE;
		}
	else if(ret != i2cTransferInProgress)
		{
			LOG_ERROR("I2C Error %d",ret);
			event |= I2C_TRANSFER_ERROR;
		}
}
