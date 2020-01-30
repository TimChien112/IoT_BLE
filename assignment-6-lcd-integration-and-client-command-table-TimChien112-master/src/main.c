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
#include "infrastructure.h"

#include "clock_init.h"
#include "state_machine_params.h"
#include "display.h"

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


// state machine flag
	volatile STATE_T next_state = SENSOR_POWEROFF;
	volatile STATE_T current_state = SENSOR_POWEROFF;
	volatile uint8_t event = NO_EVENT;
	volatile uint8_t connection_flag = BLE_DISCONNECT;
 // Time stamp parameters
 	#define	stamp_start_flag		0
	#define	stamp_stop_flag			1
	uint32_t start_timestamp_tick	= 0;
	uint32_t end_timestamp_tick		= 0;
	uint32_t stamp_rollover_times	= 0;

	I2C_TransferReturn_TypeDef ret;

 // BLE parameters

	#define TEMP_START 				1<<0
	#define RSSI_GET 				1<<1
	volatile uint32_t g_temp_ext_event_status = 0;
	#define LE_MIN_ADVERTISING_INTERVAL_MS		(250)
	#define LE_MAX_ADVERTISING_INTERVAL_MS		(250)
	#define LE_MIN_ADVERTISING_INTERVAL_COUNT	(400)			//count = LE_MIN_ADVERTISING_INTERVAL_MS / 0.625
	#define LE_MAX_ADVERTISING_INTERVAL_COUNT	(400)			//count = LE_MAX_ADVERTISING_INTERVAL_MS / 0.625
	#define LE_MIN_CONNECTION_INTERVAL_MS		(75)
	#define LE_MAX_CONNECTION_INTERVAL_MS		(75)
	#define LE_MIN_CONNECTION_INTERVAL_COUNT	(60)			//Count = LE_MIN_CONNECTION_INTERVAL_MS/1.25
	#define LE_MAX_CONNECTION_INTERVAL_COUNT	(60)			//Count = LE_MAX_CONNECTION_INTERVAL_MS/1.25
	#define LE_SLAVE_LATENCY_MS					(300)
	#define LE_SLAVE_LATENCY					(4-1)			//Latency = (LE_SLAVE_LATENCY_MS/LE_MAX_CONNECTION_INTERVAL_MS) - 1
	#define LE_CONNECTION_TIMEOUT_MS			(600)			//Timeout >= (latency_ms*2)
	#define LE_TX_MAX							(80)
	#define LE_TX_MIN							(-80)

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
		uint32_t time_stamp = time_stamp_t * (1000000/1000);
		//LOG_INFO("stamp_rollover_times %d end_timestamp_tick %d start_timestamp_tick %d\n",stamp_rollover_times,end_timestamp_tick,start_timestamp_tick);
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
		gecko_external_signal(TEMPREAD_FLAG);

		/*for loggerGetTimestamp function*/
		stamp_rollover_times ++;
		/*for loggerGetTimestamp function*/
	}else if (Int_Flag & LETIMER_IF_COMP1)
	{
		/*for wait_usec_x function*/
		if (rollover_times == 0)
		{
			event |= LETIMER0_COMP1_FLAG;
			gecko_external_signal(TEMPREAD_FLAG);
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
	//LOG_INFO("Start writing\n");
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
	//LOG_INFO("Start Reading\n");
	//read data from Si7021
	seq.flags=I2C_FLAG_READ;
	seq.buf[0].data = read_buffer_data;
	seq.buf[0].len = read_buffer_len;
	I2C_TransferInit(I2C0,&seq);
}

static void TempValueLog(void)
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
	LOG_INFO("Read temperature  %04f\n",final_temp);
	//LOG_INFO("Read temperature  %d %d\n",read_buffer_data[0],read_buffer_data[1]);

	displayPrintf(DISPLAY_ROW_TEMPVALUE,"tempe %04f", final_temp);
	temperature = FLT_TO_UINT32(temp, -3);
	UINT32_TO_BITSTREAM(p, temperature);
	gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_temperature_measurement, 5, htmTempBuffer);
}

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
		gecko_external_signal(TEMPREAD_FLAG);
		//LOG_INFO("State Transition:%d to %d",current_state,next_state);
		current_state = next_state;

	}
}


void gecko_ecen5823_update(struct gecko_cmd_packet* evt)
{
	int8_t rssi;

	gecko_update(evt);
	switch (BGLIB_MSG_ID(evt->header)){
		case gecko_evt_system_boot_id:
			//bleEventSystemBoot(&evt->data.evt_system_boot);
			gecko_cmd_le_gap_set_adv_parameters(LE_MIN_ADVERTISING_INTERVAL_COUNT, LE_MAX_ADVERTISING_INTERVAL_COUNT,7);
			gecko_cmd_le_gap_set_mode(le_gap_general_discoverable,le_gap_undirected_connectable);
			struct gecko_msg_hardware_set_soft_timer_rsp_t *tmrResp;
			tmrResp = gecko_cmd_hardware_set_soft_timer(32576,0,0);
			displayPrintf(DISPLAY_ROW_CONNECTION,"Advertising");
			displayPrintf(DISPLAY_ROW_NAME,"Server");
			struct gecko_msg_system_get_bt_address_rsp_t * server_addr = gecko_cmd_system_get_bt_address();
			displayPrintf(DISPLAY_ROW_BTADDR, "%02x:%02x:%02x:%02x:%02x:%02x",
					  server_addr->address.addr[5],
					  server_addr->address.addr[4],
					  server_addr->address.addr[3],
					  server_addr->address.addr[2],
					  server_addr->address.addr[1],
					  server_addr->address.addr[0]);
			break;

		case gecko_evt_le_connection_opened_id:
			gecko_cmd_le_connection_set_parameters(evt->data.evt_le_connection_opened.connection, \
	  	  	  	  	  	  LE_MIN_CONNECTION_INTERVAL_COUNT, \
						  LE_MAX_CONNECTION_INTERVAL_COUNT, \
						  LE_SLAVE_LATENCY, \
						  LE_CONNECTION_TIMEOUT_MS);
			connection_flag = BLE_CONNECT;
			displayPrintf(DISPLAY_ROW_CONNECTION,"Connection opened");
			LOG_INFO("Connection open here\n");
			break;


		case gecko_evt_le_connection_closed_id:
			gecko_cmd_system_set_tx_power(0);
		    /* Restart advertising after client has disconnected */
		    gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
		    LOG_INFO("Conection close here\n");
		    displayPrintf(DISPLAY_ROW_CONNECTION,"Advertising");
		    connection_flag = BLE_DISCONNECT;
		    break;


		case gecko_evt_system_external_signal_id:
			//READ_TEMP();
			if((evt->data.evt_system_external_signal.extsignals & TEMPREAD_FLAG) != 0)
			{
				/*if(connection_flag & BLE_CONNECT)*/state_machine();
			}

			break;

		case gecko_evt_le_connection_rssi_id:
			rssi = evt->data.evt_le_connection_rssi.rssi;

			uint16_t TX_Power = 0;
			if(rssi > -35)
				TX_Power = LE_TX_MIN;
			else if(rssi > -45)
				TX_Power= -200;
			else if(rssi > -55)
				TX_Power = -150;
			else if(rssi > -65)
				TX_Power = -50;
			else if(rssi > -75)
				gecko_cmd_system_set_tx_power(0);
			else if(rssi > -85)
				TX_Power = 50;
			else
				TX_Power = LE_TX_MAX;
			LOG_INFO("rssi is %d\n",rssi);
			gecko_cmd_system_halt(1);
			gecko_cmd_system_set_tx_power(TX_Power);
			gecko_cmd_system_halt(0);
			break;

		case gecko_evt_gatt_server_characteristic_status_id:
			if (evt-> data.evt_gatt_server_characteristic_status.status_flags == gatt_server_confirmation)
			{

			  gecko_cmd_le_connection_get_rssi(evt-> data.evt_gatt_server_characteristic_status.connection);
			}
			break;

		case gecko_evt_hardware_soft_timer_id:
			displayUpdate();
			break;
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
// Initialize Log for debug
  logInit();
// Initialize Bluetooth stack
  gecko_init(&config);
//Initialize GPIO
  gpioInit();
//Initialize timer, clock & Oscillator
  Clock_Init();
//Initialize I2C
  i2c_Init();
//Initialize LCD
  displayInit();
//Log
  logFlush();


//Infinite loop
  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();
    gecko_ecen5823_update(evt);


  }
}
