/*
 * log.c
 *
 *  Created on: Dec 18, 2018
 *      Author: Dan Walkes
 */

#include "retargetserial.h"
#include "log.h"
#include <stdint.h>



#if INCLUDE_LOGGING
/**
 * @return a timestamp value for the logger, typically based on a free running timer.
 * This will be printed at the beginning of each log message.
 */
uint32_t loggerGetTimestamp(void)
{


	//return timerGetRunTimeMilliseconds();
	return 0;
}

/**
 * Initialize logging for Blue Gecko.
 * See https://www.silabs.com/community/wireless/bluetooth/forum.topic.html/how_to_do_uart_loggi-ByI
 */
void logInit(void)
{
	RETARGET_SerialInit();
	/**
	 * See https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__RetargetIo.html#ga9e36c68713259dd181ef349430ba0096
	 * RETARGET_SerialCrLf() ensures each linefeed also includes carriage return.  Without it, the first character is shifted in TeraTerm
	 */
	RETARGET_SerialCrLf(true);
	LOG_INFO("Initialized Logging");
}

/**
 * Block for chars to be flushed out of the serial port.  Important to do this before entering SLEEP() or you may see garbage chars output.
 */
void logFlush(void)
{
	RETARGET_SerialFlush();
}

// Time stamp parameters
	#define	stamp_start_flag		0
	#define	stamp_stop_flag			1
	uint32_t start_timestamp_tick	= 0;
	uint32_t end_timestamp_tick		= 0;
	uint32_t stamp_rollover_times	= 0;

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

#endif
