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
#include "I2C.h"

#include "log.h"
#include "em_core.h"
#include "infrastructure.h"

#include "clock_init.h"
#include "state_machine_params.h"
#include "display.h"

#include "wait_funct.h"
#include "ble_stack_params.h"
#include "ble_device_type.h"

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

extern uint8_t external_event;

void LETIMER0_IRQHandler(void)
{
	uint32_t Int_Flag = LETIMER_IntGet(LETIMER0);
	// clear interrupt flag IF by setting IFC
	LETIMER_IntClear(LETIMER0,Int_Flag);
	if (Int_Flag & LETIMER_IF_UF)
	{
		// Underflow reach
		event |= LETIMER0_UF_FLAG;
		external_event |= TEMPREAD_FLAG;
		gecko_external_signal(external_event);

		/*for loggerGetTimestamp function*/
		stamp_rollover_times ++;
		/*for loggerGetTimestamp function*/
	}else if (Int_Flag & LETIMER_IF_COMP1)
	{
		/*for wait_usec_x function*/
		if (rollover_times == 0)
		{
			event |= LETIMER0_COMP1_FLAG;
			external_event |= TEMPREAD_FLAG;
			gecko_external_signal(external_event);
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
	LOG_INFO("Read temperature  %04f\n",final_temp);
	displayPrintf(DISPLAY_ROW_TEMPVALUE,"tempe %04f", final_temp);

	temperature = FLT_TO_UINT32(final_temp*1000, -3);
	UINT32_TO_BITSTREAM(p, temperature);
	gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_temperature_measurement, 5, htmTempBuffer);
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
// device check  change device type in "ble_device_type.h"
  bool device_type = IsServerDevice();
  enable_button_interrupts();

//Infinite loop
  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();
    if (device_type) // serve : true
    {
    	gecko_server_update(evt);
    }
    else
    {
    	gecko_client_update(evt);
    }

  }
}
