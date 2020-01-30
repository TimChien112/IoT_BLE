/*
 * wait_funct.c
 *
 *  Created on: 2019¦~2¤ë27¤é
 *      Author: tim01
 */


#include "wait_funct.h"
#include "em_letimer.h"
#include <stdint.h>

// flag for event trigger
	uint32_t rollover_times = 0;

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
