
#include "em_cmu.h"
#include "sleep.h"
#include "em_letimer.h"
#include "state_machine_params.h"
#include "log.h"
#include "wait_funct.h"

extern uint8_t external_event;
// parameters for Clock & sleep mode
	#define sleep_mode 		3
	#define period_ms 		3000
	#define on_time_ms		0
	uint32_t Period_tick;
	uint32_t On_time_tick;

void Clock_Init(void)
{

	uint32_t Period_Time_LFXO = period_ms*32768/4/1000;
	uint32_t On_Time_LFXO = (period_ms-on_time_ms)*32768/4/1000;
	uint32_t Period_Time_ULFRCO = period_ms;
	uint32_t On_Time_ULFRCO = (period_ms-on_time_ms);

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
	  CMU_ClockEnable(cmuClock_LFA,true);
	  CMU_ClockEnable(cmuClock_LETIMER0, true);

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
	  CMU_ClockEnable(cmuClock_LFA,true);
	  CMU_ClockDivSet(cmuClock_LETIMER0,cmuClkDiv_1);   //Div_1  1*(1/1024)*65535 = 63.999 sec for ULFRCO
	  CMU_ClockEnable(cmuClock_LETIMER0, true);

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


		// enable Timer interrupt
	  LETIMER_IntEnable(LETIMER0,LETIMER_IEN_UF); //enable interrupt flag for UF & Compare1
	  NVIC_EnableIRQ(LETIMER0_IRQn);
	  //enable LETimer0
	  LETIMER_Enable(LETIMER0,true);
}

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
