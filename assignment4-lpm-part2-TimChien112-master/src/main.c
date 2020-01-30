/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#include "em_device.h"
#include "em_chip.h"
#include "gpio.h"
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif

#include "sleep.h"
#include "em_letimer.h"

#include "i2cspm.h"
#include "em_i2c.h"

#include "log.h"
#include "em_core.h"

#include "clock_init.h"
#include "state_machine_params.h"
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
};

// flag for event trigger
	#define read_temp_flag	1
	uint8_t event_buffer 	= 0;
	uint32_t rollover_times = 0;

// parameters for I2C transfer function
	I2C_TransferSeq_TypeDef seq;
	uint8_t write_buffer_data = 0xE3;
	uint16_t write_buffer_len = 1;
	uint8_t read_buffer_data[2];
	uint16_t read_buffer_len  = 2;

	volatile STATE_T next_state = SENSOR_POWEROFF;
	volatile STATE_T current_state = SENSOR_POWEROFF;
	volatile uint8_t event = NO_EVENT;

//
//
//// state machine flag
//	uint8_t current_state_flag 	=	0;
//	#define INITIALIZATION			0
//	#define WAIT_TIMER_INTERRUPT	1
//	#define GPIO_ON_AND_WAIT		2
//	#define	START_WRITE				3
//	#define	WAIT_WRITE_COMPLETE		4
//	#define WAIT_TEMP_READY			5
//	#define	START_READ				6
//	#define WAIT_READ_COMPLETE		7
//	#define	OFF_GPIO_AND_LOG		8

 // Time stamp parameters
 	#define	stamp_start_flag		0
	#define	stamp_stop_flag			1
	uint32_t start_timestamp_tick	= 0;
	uint32_t end_timestamp_tick		= 0;
	uint32_t stamp_rollover_times	= 0;


// Log for Time stamp
void loggerGetTimestamp_c(uint8_t flag)
{
	if (flag == stamp_start_flag) 		// 0 => start
	{
		start_timestamp_tick = LETIMER_CounterGet(LETIMER0);
		stamp_rollover_times = 0;
	}else if(flag == stamp_stop_flag)	// 1 => stop & log time stamp
	{
		end_timestamp_tick   = LETIMER_CounterGet(LETIMER0);
		uint32_t time_stamp_t = 0;
		if(stamp_rollover_times !=0) time_stamp_t = (stamp_rollover_times+1)*LETIMER_CompareGet(LETIMER0,0)-(end_timestamp_tick-start_timestamp_tick);
		else  time_stamp_t = start_timestamp_tick - end_timestamp_tick;
		uint32_t time_stamp = time_stamp_t * (1000000/1024);
		LOG_INFO("stamp_rollover_times %d end_timestamp_tick %d start_timestamp_tick %d\n",stamp_rollover_times,end_timestamp_tick,start_timestamp_tick);
		LOG_INFO("Timer stamp %d (us) tick %d\n",time_stamp,time_stamp_t);
	}
}

// Wait function in us with Comp1
void Wait_usec_c(uint32_t wait_us)
{
	uint32_t current_tick = LETIMER_CounterGet(LETIMER0);
	uint32_t tick_diff = wait_us/1000*(1024/1000);
	uint32_t end_tick;
	if(tick_diff>=current_tick) {
		rollover_times = (tick_diff-current_tick)/LETIMER_CompareGet(LETIMER0,0)+1;
		end_tick = rollover_times*LETIMER_CompareGet(LETIMER0,0)-(tick_diff-current_tick);
	}else{
		rollover_times = 0;
		end_tick = current_tick - tick_diff;
	}
	LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
	LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);
	LETIMER_CompareSet(LETIMER0, 1, end_tick);

}
// Wait function in us without Comp1
void Wait_usec(uint32_t wait_us)
{
	uint32_t current_tick = LETIMER_CounterGet(LETIMER0);
	uint32_t tick_diff = wait_us/1000*(1024/1000);
	uint32_t rollover_times;
	uint32_t end_tick;
	uint32_t rollover_compare;
	if(tick_diff>=current_tick) {
		rollover_times = (tick_diff-current_tick)/LETIMER_CompareGet(LETIMER0,0)+1;
		end_tick = rollover_times*LETIMER_CompareGet(LETIMER0,0)-(tick_diff-current_tick);
	}else{
		rollover_times = 0;
		end_tick = current_tick - tick_diff;
	}
	LOG_INFO("Timer wait start ");
	while (rollover_times>0){
		rollover_compare = LETIMER_CounterGet(LETIMER0);
		if(rollover_compare >= current_tick)
		{
			rollover_times--;
			current_tick = LETIMER_CounterGet(LETIMER0);
			LOG_INFO(" rollover left times %d ",rollover_times);
		}
	}
	while (rollover_times == 0)
	{
		current_tick = LETIMER_CounterGet(LETIMER0);
		if(current_tick<end_tick) break;
	}
	LOG_INFO("Timer wait complete \n");
}
// Initialize I2CSPM & seq.device_address
void i2c_Init(void)
{
	gpioTmpSenSetOff();
	uint16_t device_address = 0x40;
	seq.addr = device_address<<1;
}

// Do something when timer interrupt occurs (1/27 added)
void LETIMER0_IRQHandler(void)
{
	uint32_t Int_Flag = LETIMER_IntGet(LETIMER0);
	// clear interrupt flag IF by setting IFC
	LETIMER_IntClear(LETIMER0,Int_Flag);
	if (Int_Flag & LETIMER_IF_UF)
	{
		// Underflow reach
		event |= LETIMER0_UF_FLAG;
		/*for loggerGetTimestamp function*/
		stamp_rollover_times ++;
		/*for loggerGetTimestamp function*/
	}else if (Int_Flag & LETIMER_IF_COMP1)
	{
		LOG_INFO("LETIMER0_COMP1\n");
		/*for wait_usec_x function*/
		if (rollover_times == 0)
		{
			event |= LETIMER0_COMP1_FLAG;
			LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
			LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
			LETIMER_CompareSet(LETIMER0, 1, 0xFFFF);
		}else
		{
			rollover_times --;
		}
		/*for wait_usec_x function*/
	}
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


static void I2CWrite_start(void)
{
	gpioI2CSetOn();
	NVIC_EnableIRQ(I2C0_IRQn);
	LOG_INFO("Start writing\n");
	//write command to Si7021
	seq.flags=I2C_FLAG_WRITE;
	seq.buf[0].data = &write_buffer_data;
	seq.buf[0].len = write_buffer_len;
	I2C_TransferInit(I2C0,&seq);
	SLEEP_SleepBlockBegin(sleepEM2);
	//I2C only work in EM1
}

static void I2CRead_start(void)
{
	NVIC_EnableIRQ(I2C0_IRQn);
	LOG_INFO("Start Reading\n");
	//read data from Si7021
	seq.flags=I2C_FLAG_READ;
	seq.buf[0].data = read_buffer_data;
	seq.buf[0].len = read_buffer_len;
	I2C_TransferInit(I2C0,&seq);
}

static void TempValueLog(void)
{
	//print out the temperature value
	uint16_t temp = read_buffer_data[0];
	temp = temp<<8;
	temp |= read_buffer_data[1];
	float final_temp = (175.72*((float)temp)/65536)-46.85;
	LOG_INFO("Read temperature  %f\n",(float)final_temp);
}

static void TempSensorSet(bool evt)
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

void state_machine()
{
	LOG_INFO("state_machine in case %d\n",current_state);
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
				TempSensorSet(false);
				gpioI2CSetOff();
				SLEEP_SleepBlockEnd(sleepEM2);
				next_state = SENSOR_POWEROFF;
			}

			break;
		case I2C_WAITFORREADCOMPLETE:
			if(event & I2C_TRANSFER_COMPLETE)
			{
				TempSensorSet(false);
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
				//turn off the sensor
				TempSensorSet(false);
				gpioI2CSetOff();
				SLEEP_SleepBlockEnd(sleepEM2);
				next_state = SENSOR_POWEROFF;
			}
			break;
	}

	if(current_state != next_state){
		LOG_INFO("State Transition:%d to %d",current_state,next_state);
		current_state = next_state;
	}
}

int main(void)
{
// Initialize device
  initMcu();
// Initialize board
  initBoard();
// Initialize application
  initApp();
  // Initialize stack
    gecko_init(&config);

  // Initialize sleep mode function before using SLEEP_Sleep() function
    SLEEP_Init_t init = { .sleepCallback = NULL, .wakeupCallback = NULL, .restoreCallback = NULL};
    SLEEP_InitEx(&init);
// Initialize Log for debug
  logInit();

  //Initialize timer, clock & Oscillator
   Clock_Init();
  gpioInit();
//Initialize I2C
  i2c_Init();


  logFlush();

//Infinite loop
  while (1) {
//	  state_machine_flag_check();
	  state_machine();

	  CORE_CRITICAL_SECTION(
		  if(sleep_mode != sleepEM0 && event == NO_EVENT){
			  SLEEP_Sleep();
		  }
	  );
//	  if(next_state==SENSOR_POWEROFF|next_state==SENSOR_WAITFORPOWERUP)
//	  {
//		  EMU_EnterEM3(true);
//	  }else
//	  {
//		  EMU_EnterEM1();
//	  }

//#define wait_us_check 	1
#ifdef wait_us_check
	#define wait_us_c		0
	#define wait_us			1

	  if(wait_us_check == wait_us)
	  {
		  GPIO_PinOutSet(gpioPortF,4);
		  loggerGetTimestamp_c(stamp_start_flag);
		  Wait_usec(1000000);				//wait for 11 seconds
		  loggerGetTimestamp_c(stamp_stop_flag);
		  GPIO_PinOutClear(gpioPortF,4);
	  }else if(wait_us_check == wait_us_c)
	  {
		  if(event_buffer==0)
		  {
			  GPIO_PinOutSet(gpioPortF,4);
			  loggerGetTimestamp_c(stamp_start_flag);
			  Wait_usec_c(1000000);		//wait for 11 seconds
			  event_buffer=2;
		  }else if(event_buffer==1)
		  {
			  loggerGetTimestamp_c(stamp_stop_flag);
			  GPIO_PinOutClear(gpioPortF,4);
			  event_buffer=0;
		  }
	  }
	  EMU_EnterEM3(true);
#endif

  }
}
