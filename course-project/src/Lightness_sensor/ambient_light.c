/*
 * ambient_light.c
 *
 *  Created on: Apr 20, 2019
 *      Author: santhosh
 */
//#include "ambient_light.h"
//#include "log.h"
//#include "wait_funct.h"
//
//uint8_t read_buffer[3];
//uint16_t value;
//I2C_TransferSeq_TypeDef seq;
//I2C_TransferReturn_TypeDef ret;
//void ambient_light_init()
//{
//	ambient_light_enable();
//	Ambient_Light_I2C_Init();
//	for (int i = 0; i<10;i++){
//		i2c_read_ambient_light_reg(I2C0,AMBIENT_LIGHT_ADDRESS,DEVICE_ID_REGISTER);
//		for (int j = 0; j<1000000;j++);
//	}
//	value = read_buffer[1]<<8 | read_buffer[2];
//	LOG_INFO("light value: %d", value);
//}
//
//void ambient_light_enable()
//{
//	GPIO_PinModeSet(AMBIENT_LIGHT_ENABLE_PORT, AMBIENT_LIGHT_ENABLE_PIN, gpioModePushPull, 1);
//}
//
//void Ambient_Light_I2C_Init()
//{
//
//		I2CSPM_Init_TypeDef init_ambient_light ={
//				I2C0,                       /* Use I2C instance 0 */                       \
//			    gpioPortC,                  /* SCL port */                                 \
//			    10,                         /* SCL pin */                                  \
//			    gpioPortC,                  /* SDA port */                                 \
//			    11,                         /* SDA pin */                                  \
//			    14,                         /* Location of SCL */                          \
//			    16,                         /* Location of SDA */                          \
//			    0,                          /* Use currently configured reference clock */ \
//			    I2C_FREQ_STANDARD_MAX,      /* Set to standard rate  */                    \
//			    i2cClockHLRStandard,        /* Set to use 4:4 low/high duty cycle */       \
//			  };
//		I2CSPM_Init(&init_ambient_light);
//
//		LOG_INFO("temperature sensor is initialized");
//}
//
//
//
//void i2c_read_ambient_light_reg(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t reg_addr)
//{
//
//	//since using 7 -bit address mode, we shift the address
//	seq.addr = (uint16_t)((slaveAddr<<1));
//
//	read_buffer[0] =  reg_addr;
//	seq.buf[0].data = read_buffer;
//	seq.buf[0].len = 1;
//
//	seq.flags = I2C_FLAG_WRITE_READ;
//	read_buffer[1] = 0;
//	seq.buf[1].data = read_buffer+1;
//	seq.buf[1].len = 2;
//	I2C_TransferReturn_TypeDef ret = I2CSPM_Transfer(i2c, &seq);
//	LOG_INFO("I2C result: %d", ret);
//	//I2C_TransferInit(i2c, &seq);
//	//NVIC_EnableIRQ(I2C0_IRQn);
//
//
//
//}
//
//void i2c_write_ambient_light_reg(I2C_TypeDef *i2c, uint8_t slaveAddr, uint8_t reg_addr)
//{
//
//	//since using 7 -bit address mode, we shift the address
//	seq.addr = (uint16_t)((slaveAddr<<1) & ((uint8_t)0xFE));
//
//	read_buffer[0] =  reg_addr;
//	seq.buf[0].data = read_buffer;
//	seq.buf[0].len = 1;
//
//	seq.flags = I2C_FLAG_WRITE;
//
//	read_buffer[1] = 0;
//	seq.buf[1].data = read_buffer+1;
//	seq.buf[1].len = 1;
//
//	I2C_TransferInit(i2c, &seq);
//	  NVIC_EnableIRQ(I2C0_IRQn);
//
//
//}
//
//
//
//void I2C0_IRQHandler(void)
//{
//	I2C_TransferReturn_TypeDef ret = I2C_Transfer(I2C0);
//
//	if(ret == i2cTransferDone)
//		{
//			value = read_buffer[1]<<8 | read_buffer[2];
//			LOG_INFO("light value: %d", value);
//		}
//	else if(ret != i2cTransferInProgress)
//		{
//			LOG_ERROR("I2C Error %d",ret);
////			event |= I2C_TRANSFER_ERROR;
//		}
//}
#include "ambient_light.h"
#include "em_gpio.h"
#include <math.h>



void I2C0_init(void)
{
	CMU_ClockEnable(cmuClock_HFPER,true);
	CMU_ClockEnable(cmuClock_I2C0, true);

	//set gpio pin modes and using wiredAnd mode for the Pull ups
	GPIO_PinModeSet(gpioPortC, 10, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(gpioPortC, 11, gpioModeWiredAnd, 1);

	I2C_Init_TypeDef i2c_init_t = I2C_INIT_DEFAULT;
	i2c_init_t.clhr = i2cClockHLRStandard;
	i2c_init_t.master = true;
	i2c_init_t.refFreq = 0;
	i2c_init_t.freq = 5000; //normal mode
	i2c_init_t.enable = false; //will start when required. Not on Init

	//routing the pins to specific location
	I2C0 -> ROUTELOC0 = I2C_ROUTELOC0_SDALOC_LOC16 |I2C_ROUTELOC0_SCLLOC_LOC14;
	I2C0 -> ROUTEPEN = I2C_ROUTEPEN_SCLPEN | I2C_ROUTEPEN_SDAPEN;

	//init I2C
	I2C_Init(I2C0, &i2c_init_t);

	//reset the I2C slave
	for (int i=0; i<9; i++){
		GPIO_PinOutClear(gpioPortC, 10);
		GPIO_PinOutSet(gpioPortC, 10);
	}

	//reset the I2C bus
	if(I2C0->STATE & I2C_STATE_BUSY){
		I2C0->CMD = I2C_CMD_ABORT;
	}
}

uint8_t I2C0_readByte(uint8_t slave_addr,uint8_t reg_addr){

	I2C0->CMD |= I2C_IFC_MSTOP;
	I2C0->CMD |= I2C_IFC_SSTOP;
	I2C0->CMD |= I2C_IFC_TXC;

	uint8_t read_data;
	uint16_t i;
	I2C0->TXDATA = (slave_addr << 1) ;//start with write

	//send the START bit
	I2C0->CMD = I2C_CMD_START;
	//I2C0 -> IFC = I2C_IFC_START;

	//wait for the salve to respond
	while ((I2C0->IF & I2C_IF_ACK) == 0);

	//after ACK has been received, it must be cleared from the IF
	I2C0->IFC = I2C_IFC_ACK;

	// Set register to 0xd0 for ID
	I2C0->TXDATA = reg_addr;

	//I2C0->CMD = I2C_CMD_START;

	//wait for the salve to respond
	while ((I2C0->IF & I2C_IF_ACK) == 0);

	//after ACK has been received, it must be cleared from the IF
	I2C0->IFC = I2C_IFC_ACK;



	I2C0->CMD = I2C_CMD_START;
	I2C0->TXDATA = (slave_addr << 1) | 0x01 ;//read


	//wait for the salve to respond
	while ((I2C0->IF & I2C_IF_ACK) == 0);

	//after ACK has been received, it must be cleared from the IF
	I2C0->IFC = I2C_IFC_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) ==0);

	read_data = I2C0->RXDATA;
    I2C0->IFC = I2C_IFC_ACK;
    I2C0->CMD |= I2C_CMD_ACK;
	I2C0->CMD |= I2C_CMD_STOP;

	for(i=0;i<1000;i++);

	return read_data;
}

void I2C0_writeByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t data){

	I2C0->TXDATA = slave_addr << 1;
	//send the START bit
	I2C0->CMD = I2C_CMD_START;
	//wait for the salve to respond
	while ((I2C0->IF & I2C_IF_ACK) == 0);
	//after ACK has been received, it must be cleared from the IF
	I2C0->IFC = I2C_IFC_ACK;

	I2C0->TXDATA =reg_addr;
	//wait for the salve to respond
	while ((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC = I2C_IFC_ACK;

	I2C0->TXDATA =data;
	//wait for the salve to respond
	while ((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC = I2C_IFC_ACK;


	I2C0->CMD = I2C_CMD_STOP;//stop I2C
}

void lum_enable()
{
	I2C_Enable(I2C0,true);
	I2C0_writeByte(SLAVE_ADDRESS_TSL2561, TSL2561_Command_register | TSL2561_Register_Control,0x02); //setting gain =0 and time=2 ie 402ms
	I2C0_writeByte(SLAVE_ADDRESS_TSL2561, TSL2561_Command_register | TSL2561_Register_Control,TSL2561_Control_Power_Up);//power up

	//settling time for the sensor
	TIMER0_startwithCount(1900);
	while(!TIMER_INT_SERVED);
	TIMER_INT_SERVED = false;
}

double getLuminosityValue()
{
	uint8_t DataLow0, DataLow1, DataHigh0, DataHigh1;
	uint32_t Ch0, Ch1;
	double ratio, lux = 0;

	//channel 0
	DataLow0 = I2C0_readByte(SLAVE_ADDRESS_TSL2561,TSL2561_Command_register | TSL2561_Word_register | TSL2561_CH0_Low);
	DataHigh0 = I2C0_readByte(SLAVE_ADDRESS_TSL2561,TSL2561_Command_register | TSL2561_Word_register | TSL2561_CH0_High);
	//Combining the DataHigh0 and DataLow0 into a word
	Ch0 = 256 * DataHigh0 + DataLow0;

	//channel 1
	DataLow1 = I2C0_readByte(SLAVE_ADDRESS_TSL2561,TSL2561_Command_register | TSL2561_Word_register | TSL2561_CH1_Low);
	DataHigh1 = I2C0_readByte(SLAVE_ADDRESS_TSL2561,TSL2561_Command_register | TSL2561_Word_register | TSL2561_CH1_High);
	//Combining the DataHigh1 and DataLow1 into a word
	Ch1 = 256 * DataHigh1 + DataLow1;

	ratio = (double)((double)Ch1)/Ch0;

	//scaling needs to be done as the gain is of 1x. Referred from the datasheet
	Ch0 = Ch0*16;
	Ch1 = Ch1*16;

	//Calculate LUX - calculations are referred from the Sensor datasheet
	if (ratio > 0 && ratio <= 0.50)
	{
		lux = 0.0304*Ch0 - 0.062*Ch0*(pow(ratio, 1.4));
	}
	else if (ratio > 0.50 && ratio <= 0.61)
	{
		lux = 0.0224*Ch0 - 0.031*Ch1;
	}
	else if (ratio > 0.61 && ratio <= 0.80)
	{
		lux = 0.0128*Ch0 - 0.0153*Ch1;
	}
	else if (ratio > 0.80 && ratio <= 1.30)
	{
		lux = 0.00146*Ch0 - 0.00112*Ch1;
	}
	else if (ratio > 1.30)
	{
		lux = 0;
	}

	return lux;
}
