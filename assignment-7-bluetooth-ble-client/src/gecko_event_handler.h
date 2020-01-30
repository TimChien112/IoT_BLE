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

#define CONNECTION_HANDLE_INVALID     (uint8_t)0xFFu
#define SERVICE_HANDLE_INVALID        (uint32_t)0xFFFFFFFFu
#define CHARACTERISTIC_HANDLE_INVALID (uint16_t)0xFFFFu
#define TABLE_INDEX_INVALID           (uint8_t)0xFFu
#define TEMP_INVALID                  (uint32_t)0xFFFFFFFFu
#define RSSI_INVALID                  (int8_t)127

typedef enum {
  scanning,
  opening,
  discoverServices,
  discoverCharacteristics,
  enableIndication,
  running
} ConnState;
#define EVT_CONN_HANDLE(evt_type)	evt->data.evt_type.connection
#define UINT32_TO_FLT(b)         (((float)((int32_t)(b) & 0x00FFFFFFU)) * (float)pow(10,((int32_t)(b) >> 24)))

typedef enum BT_CONN_STATE
{
	STATE_DISCONNECTED = 0,
	STATE_DISCOVER_SERVICES,
	STATE_DISCOVER_CHARACTERISTICS,
	STATE_ENABLE_NOTIFICATIONS,
	STATE_DATA_NOTIF_READY,
}BT_CONN_STATE_T;

typedef struct BT_SERVER_CONNECTION
{
	bd_addr serverBTAddress;
	BT_CONN_STATE_T connectionState;
	uint8_t connectionHandle;
	uint8_t numOfServices;
	uint8array servicesUUID[4];
	uint32_t servicesHandle[4];

}BT_SERVER_CONNECTION_T;
// State of the connection under establishment
ConnState connState;

// Health Thermometer service UUID defined by Bluetooth SIG
const uint8_t thermoService[2];
// Temperature Measurement characteristic UUID defined by Bluetooth SIG
const uint8_t thermoChar[2];

typedef struct {
  uint8_t  connectionHandle;
  int8_t   rssi;
  uint16_t serverAddress;
  uint32_t thermometerServiceHandle;
  uint16_t thermometerCharacteristicHandle;
  uint32_t temperature;
} ConnProperties;
#define MAX_CONNECTIONS               4
ConnProperties connProperties[MAX_CONNECTIONS];

// Counter of active connections
uint8_t activeConnectionsNum;

void gecko_client_update(struct gecko_cmd_packet* evt);
void gecko_server_update(struct gecko_cmd_packet* evt);


#endif /* SRC_GECKO_EVENT_HANDLER_H_ */
