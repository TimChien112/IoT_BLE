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

// parameters for Clock & sleep mode
	#define sleep_mode 		3
	#define period_ms 		3000
	#define on_time_ms		0
	uint32_t Period_tick;
	uint32_t On_time_tick;

// flag for event trigger
	#define read_temp_flag	1
	uint8_t event_buffer 	= 0;

// parameters for I2C transfer function
	I2C_TransferSeq_TypeDef seq;
	I2CSPM_Init_TypeDef I2C_Init_Config = I2CSPM_INIT_DEFAULT;
	uint8_t write_buffer_data = 0xE3;
	uint16_t write_buffer_len = 1;
	uint8_t read_buffer_data[2];
	uint16_t read_buffer_len  = 2;
	I2C_TransferReturn_TypeDef ret;


void Clock_Init(void)
{

	uint32_t Period_Time_LFXO = period_ms*32768/4/1000;
	uint32_t On_Time_LFXO = (period_ms-on_time_ms)*32768/4/1000;
	uint32_t Period_Time_ULFRCO = period_ms*1024/1000;
	uint32_t On_Time_ULFRCO = (period_ms-on_time_ms)*1024/1000;

	// Set configurations for LETIMER0
	  const LETIMER_Init_TypeDef LETimer_init_config =
	     {
	         .enable         = false,                   // Don't start counting when init completed - only with RTC compare match
	         .debugRun       = false,                  // Counter shall not keep running during debug halt.
	         //.rtcComp0Enable = false,                  // Start counting on RTC COMP0 match.
	         //.rtcComp1Enable = false,                  // Don't start counting on RTC COMP1 match.
	         .comp0Top       = true,                   // Load COMP0 register into CNT when counter underflows. COMP is used as TOP
	         .bufTop         = false,                  // Don't load COMP1 into COMP0 when REP0 reaches 0.
	         .out0Pol        = 0,                      // Idle value for output 0.
	         .out1Pol        = 0,                      // Idle value for output 1.
	         .ufoa0          = letimerUFOANone,        // Pulse output on output 0
	         .ufoa1          = letimerUFOANone,        // No output on output 1
	         .repMode        = letimerRepeatFree,      //Free mode
	     };

	// enable the clock sources we will use

	  if (sleep_mode == sleepEM1 || sleep_mode == sleepEM2){	//EM1 & EM2
	  CMU_OscillatorEnable(cmuOsc_LFXO,true,true);		//LFXO enable
	  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);	//use LFXO for LFA clock tree
	  CMU_ClockDivSet(cmuClock_LETIMER0,cmuClkDiv_4);   //Div_4  4*(1/32768)*65535 = 7.999 sec for LFXO


	  //Setting up the LETIMER0 registers
	  LETIMER_Init(LETIMER0, &LETimer_init_config);
	  	  // Set initial compare values for COMP0 & COMP1
	  	  Period_tick=Period_Time_LFXO;
	  	  On_time_tick = On_Time_LFXO;
	  	  /*LETIMER_CompareSet(LETIMER0, 0, Period_Time_LFXO);  	//<= 7s for LFXO with Div_4
	  	  LETIMER_CompareSet(LETIMER0, 1, On_Time_LFXO);  		//<= 6s for LED turn off*/
	  }

	  else if(sleep_mode == sleepEM3){							//EM3
	  CMU_OscillatorEnable(cmuOsc_ULFRCO,true,true);	//ULFRCO enable
	  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);//use  ULFRCO for LFA clock tree
	  CMU_ClockDivSet(cmuClock_LETIMER0,cmuClkDiv_1);   //Div_1  1*(1/1024)*65535 = 63.999 sec for ULFRCO

	  //Setting up the LETIMER0 registers
	  LETIMER_Init(LETIMER0, &LETimer_init_config);
	  	  // Set initial compare values for COMP0 & COMP1
	  	  Period_tick=Period_Time_ULFRCO;
	  	  On_time_tick = On_Time_ULFRCO;
	  	  /*LETIMER_CompareSet(LETIMER0, 0, Period_Time_ULFRCO);  //<= 7s for ULFRCO with Div_1
	  	  if(on_time_ms!=0)LETIMER_CompareSet(LETIMER0, 1, On_Time_ULFRCO);  	//<= 6s for LED turn off*/
	  }
	  LETIMER_CompareSet(LETIMER0, 0, Period_tick);  //<= 7s for ULFRCO with Div_1
	  if(on_time_ms!=0)LETIMER_CompareSet(LETIMER0, 1, On_time_tick);  	//<= 6s for LED turn off

	  CMU_ClockEnable(cmuClock_LFA,true);
	  CMU_ClockEnable(cmuClock_LETIMER0, true);


		// enable Timer interrupt
		  LETIMER_IntEnable(LETIMER0,LETIMER_IEN_UF+LETIMER_IEN_COMP1); //enable interrupt flag for UF & Compare1
		  NVIC_EnableIRQ(LETIMER0_IRQn);
		//enable LETimer0
		  LETIMER_Enable(LETIMER0,true);



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
	I2CSPM_Init(&I2C_Init_Config);
	GPIO_PinOutClear(gpioPortD,15);

	uint16_t device_address = 0x40;
	seq.addr = device_address<<1;
}
// Read temperature (GPIO turn on -> write command -> read temperature -> GPIO turn off)
void READ_TEMP(void)
{

	LOG_INFO("Start writing");
	//write command to Si7021
	seq.flags=I2C_FLAG_WRITE;
	seq.buf[0].data = &write_buffer_data;
	seq.buf[0].len = write_buffer_len;
	ret = I2CSPM_Transfer(I2C_Init_Config.port,&seq);
	LOG_INFO("ret %d",ret);

	LOG_INFO("Start Reading");
	//read data from Si7021
	seq.flags=I2C_FLAG_READ;
	seq.buf[0].data = read_buffer_data;
	seq.buf[0].len = read_buffer_len;
	ret = I2CSPM_Transfer(I2C_Init_Config.port,&seq);
	LOG_INFO("ret %d",ret);
	uint16_t temp = read_buffer_data[0];
	temp = temp<<8;
	temp |= read_buffer_data[1];
	temp = (temp*(175.72/65536))-46.85;
	LOG_INFO("Read temperature  %d ",temp);


}
// Do something when timer interrupt occurs (1/27 added)
void LETIMER0_IRQHandler(void)
{
	uint32_t reason = LETIMER_IntGet(LETIMER0);
	// clear interrupt flag IF by setting IFC
	LETIMER_IntClear(LETIMER0,reason);
	if (reason & LETIMER_IF_UF)
	{
		// Underflow reach
		event_buffer |= read_temp_flag;	//enable read_temp_flag
	}
}

static void delayApproxOneSecond(void)
{
	/**
	 * Wait loops are a bad idea in general!  Don't copy this code in future assignments!
	 * We'll discuss how to do this a better way in the next assignment.
	 */
	volatile int i;
	for (i = 0; i < 350000; ) {
		  i=i+1;
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
// Initialize stack
  gecko_init(&config);
  gpioInit();
//Initialize timer, clock & Oscillator
  Clock_Init();
//Initialize I2C
  i2c_Init();

  // Initialize sleep mode function before using SLEEP_Sleep() function
  SLEEP_InitEx(NULL);
  SLEEP_SleepBlockBegin(sleep_mode+1);
  //GPIO_PinOutSet(gpioPortD,15);


  logFlush();
//Infinite loop
  while (1) {
	  if (sleep_mode == sleepEM1 || sleep_mode == sleepEM2)
	  {
		  SLEEP_Sleep();
	  }
	  else if (sleep_mode == sleepEM3)
	  {
		  EMU_EnterEM3(true);
	  }
	  if(event_buffer>0)
	  {
		  event_buffer &= ~read_temp_flag;	//clear event_trigger bit
		  GPIO_PinOutSet(gpioPortD,15);
		  Wait_usec(80000);

		  READ_TEMP();
		  GPIO_PinOutClear(gpioPortD,15);
	  }
  }
}
