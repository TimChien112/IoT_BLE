/*
 */
#include "em_cmu.h"
#include "sleep.h"
#include "em_letimer.h"

// parameters for Clock & sleep mode
	#define sleep_mode 		3
	#define period_ms 		3000
	#define on_time_ms		0
	uint32_t Period_tick;
	uint32_t On_time_tick;
	volatile bool TIMER_INT_SERVED;

void Clock_Init();
void LETIMER0_IRQHandler(void);
