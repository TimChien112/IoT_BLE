/*
 * BLE_stack_params.h
 *
 *  Created on: 2019¦~2¤ë27¤é
 *      Author: tim01
 */
#include <stdint.h>

#ifndef SRC_BLE_STACK_PARAMS_H_
#define SRC_BLE_STACK_PARAMS_H_

	#define RSSI_GET 				1<<1
	volatile uint32_t g_temp_ext_event_status;
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




#endif /* SRC_BLE_STACK_PARAMS_H_ */
