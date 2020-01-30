/*
 * gecko_event_handler.h
 *
 *  Created on: 2019¦~2¤ë27¤é
 *      Author: tim01
 */

#include "native_gecko.h"
#include <stdint.h>

#ifndef SRC_GECKO_EVENT_HANDLER_H_
#define SRC_GECKO_EVENT_HANDLER_H_

#define enable_bonding	1

#define conn_state(evt_type)	evt->data.evt_type.connection
#define UINT32_TO_FLT(b)         (((float)((int32_t)(b) & 0x00FFFFFFU)) * (float)pow(10,((int32_t)(b) >> 24)))

typedef enum BT_CONN_STATE
{
	STATE_DISCONNECTED = 0,
	STATE_BONDING,
	STATE_DISCOVER_SERVICES,
	STATE_DISCOVER_CHARACTERISTICS,
	STATE_ENABLE_NOTIFICATIONS,
	STATE_ENABLE_NOTIFICATIONS2,
	STATE_DATA_NOTIF_READY,
}BT_CONN_STATE_T;


typedef struct
{
	uint8 data[16];
}uint8array_16;

typedef struct BT_SERVER_CONNECTION
{
	bd_addr serverBTAddress;
	BT_CONN_STATE_T connectionState;
	uint8_t connectionHandle;
	uint8_t numOfServices;
	uint8array_16 servicesUUID[4];
	uint32_t servicesHandle[4];

}BT_SERVER_CONNECTION_T;


// Health Thermometer service UUID defined by Bluetooth SIG
const uint8_t thermoService[2];
// Temperature Measurement characteristic UUID defined by Bluetooth SIG
const uint8_t thermoChar[2];


void gecko_client_update(struct gecko_cmd_packet* evt);
void gecko_server_update(struct gecko_cmd_packet* evt);

#if enable_bonding
uint8_t external_event;
#define PB0_PRESSED (1<<0)
#define PB1_PRESSED (1<<1)
void buttonPushedCallback(uint8 pin);
bool waitingForConfirm;
void enable_button_interrupts(void);
void disable_button_interrupts();
#endif

#endif /* SRC_GECKO_EVENT_HANDLER_H_ */
