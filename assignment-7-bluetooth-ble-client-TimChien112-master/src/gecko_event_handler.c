/*
 * gecko_event_handler.c
 *
 *  Created on: 2019¦~2¤ë27¤é
 *      Author: tim01
 */
#include "gecko_event_handler.h"
#include "native_gecko.h"
#include "ble_stack_params.h"
#include "state_machine_params.h"
#include "log.h"
#include "display.h"
#include "client_funct.h"
#include "ble_device_type.h"

// Health Thermometer service UUID defined by Bluetooth SIG
const uint8_t thermoService[2] = { 0x09, 0x18 };
// Temperature Measurement characteristic UUID defined by Bluetooth SIG
const uint8_t thermoChar[2] = { 0x1c, 0x2a };
#define MAX_NUM_SERVER	2
BT_SERVER_CONNECTION_T servers[MAX_NUM_SERVER] = {0};


uint16_t tempCharacteristicHandle;
void gecko_client_update(struct gecko_cmd_packet* evt)
{

	gecko_update(evt);

	switch (BGLIB_MSG_ID(evt->header)){
		case gecko_evt_system_boot_id:
			displayPrintf(DISPLAY_ROW_CONNECTION,"Discovering");
			displayPrintf(DISPLAY_ROW_NAME,"Client");
			struct gecko_msg_system_get_bt_address_rsp_t * client_addr = gecko_cmd_system_get_bt_address();
			displayPrintf(DISPLAY_ROW_BTADDR, "%02x:%02x:%02x:%02x:%02x:%02x",
					client_addr->address.addr[5],
					client_addr->address.addr[4],
					client_addr->address.addr[3],
					client_addr->address.addr[2],
					client_addr->address.addr[1],
					client_addr->address.addr[0]);
			struct gecko_msg_hardware_set_soft_timer_rsp_t *tmrResp;
			tmrResp = gecko_cmd_hardware_set_soft_timer(32576,0,0);

			// Start scanning - looking for thermometer devices
			gecko_cmd_le_gap_start_discovery(le_gap_phy_1m, le_gap_discover_generic);
			break;

		case gecko_evt_le_gap_scan_response_id:
				gecko_cmd_le_gap_end_procedure();
				// and connect to that device
				bd_addr serverAddr = { .addr = SERVER_BT_ADDRESS};
				struct gecko_msg_le_gap_connect_rsp_t *gecko_rsp_msg = gecko_cmd_le_gap_connect(serverAddr, 0 , le_gap_phy_1m);
			break;


		case gecko_evt_le_connection_opened_id:
	    	servers[0].connectionState = STATE_DISCOVER_SERVICES;
	    	servers[0].connectionHandle = evt->data.evt_le_connection_opened.connection;
	    	servers[0].serverBTAddress = evt->data.evt_le_connection_opened.address;
			displayPrintf(DISPLAY_ROW_BTADDR2, "%02x:%02x:%02x:%02x:%02x:%02x",
					servers[0].serverBTAddress.addr[5],
							servers[0].serverBTAddress.addr[4],
							servers[0].serverBTAddress.addr[3],
							servers[0].serverBTAddress.addr[2],
							servers[0].serverBTAddress.addr[1],
							servers[0].serverBTAddress.addr[0]
						  );
			// Discover Health Thermometer service on the slave device
			gecko_cmd_gatt_discover_primary_services_by_uuid(evt->data.evt_le_connection_opened.connection,
															 2,
															 thermoService);
			//connState = discoverServices;
			displayPrintf(DISPLAY_ROW_CONNECTION,"Connection opened");
			LOG_INFO("Connection open here\n");
			break;


		case gecko_evt_gatt_service_id:		// This event is generated when a new service is discovered
			servers[0].servicesHandle[servers[0].numOfServices] = evt->data.evt_gatt_service.service;
			servers[0].servicesUUID[servers[0].numOfServices].len = evt->data.evt_gatt_service.uuid.len;
			memcpy(	&servers[0].servicesUUID[servers[0].numOfServices].data[0],
					&evt->data.evt_gatt_service.uuid.data[0],
					evt->data.evt_gatt_service.uuid.len);
			LOG_INFO("Discovered Service Handle: %lu, UUID: 0x%02x%02x \n",
					servers[0].servicesHandle[servers[0].numOfServices],
					servers[0].servicesUUID[servers[0].numOfServices].data[0],
					servers[0].servicesUUID[servers[0].numOfServices].data[1]);
			servers[0].numOfServices++;
			break;


		      // This event is generated when a new characteristic is discovered
		case gecko_evt_gatt_characteristic_id:
			if((memcmp(&evt->data.evt_gatt_characteristic.uuid.data[0],thermoChar,2) == 0))
			{
				tempCharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
				LOG_INFO("Temp char UUID: 0x%02x%02x\n",evt->data.evt_gatt_characteristic.uuid.data[1],evt->data.evt_gatt_characteristic.uuid.data[0]);
			}
			break;

		case gecko_evt_gatt_procedure_completed_id:
			// If service discovery finished
			switch(servers[0].connectionState)
			{
				case STATE_DISCOVER_SERVICES:
					if(evt->data.evt_gatt_procedure_completed.result == 0)
					{
						servers[0].connectionState = STATE_DISCOVER_CHARACTERISTICS;
					}
					struct gecko_msg_gatt_discover_characteristics_by_uuid_rsp_t *rsp = gecko_cmd_gatt_discover_characteristics_by_uuid(
											  servers[0].connectionHandle,
											  servers[0].servicesHandle[servers[0].numOfServices-1],
											  2,
											  (const uint8_t*)thermoChar);
					break;
			// If characteristic discovery finished
				case STATE_DISCOVER_CHARACTERISTICS:

					if(evt->data.evt_gatt_procedure_completed.result == 0){
						if(tempCharacteristicHandle > 0){
							servers[0].connectionState = STATE_ENABLE_NOTIFICATIONS;
							struct gecko_msg_gatt_set_characteristic_notification_rsp_t *rsp = gecko_cmd_gatt_set_characteristic_notification(
													servers[0].connectionHandle,
													tempCharacteristicHandle,
													gatt_indication);
						}
					}
			}
			break;

		case gecko_evt_gatt_characteristic_value_id: // This event is generated when a characteristic value was received e.g. an indication
			if(evt->data.evt_gatt_characteristic_value.characteristic == tempCharacteristicHandle)
			{
				if(evt->data.evt_gatt_characteristic_value.att_opcode == 0x1d){
					struct gecko_msg_gatt_send_characteristic_confirmation_rsp_t *rsp =
					gecko_cmd_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
				}
				uint8_t tempData[5] = {0};
				memcpy(tempData, &evt->data.evt_gatt_characteristic_value.value.data[0], 5 );
				float temperature_c = UINT32_TO_FLT(*(uint32_t*)&tempData[1]);
				LOG_INFO("temp %.04f",temperature_c);
				char tempBuffer[10] = {0};
				snprintf(tempBuffer, sizeof(tempBuffer), "temp %.04f C",temperature_c);
				displayPrintf(DISPLAY_ROW_TEMPVALUE,tempBuffer);
			}
			break;


		case gecko_evt_le_connection_closed_id:
			gecko_cmd_system_set_tx_power(0);
		    /* Restart advertising after client has disconnected */
		    gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
		    LOG_INFO("Conection close here\n");
		    displayPrintf(DISPLAY_ROW_CONNECTION,"Discovering");
		    connection_flag = BLE_DISCONNECT;
		    break;

		case gecko_evt_hardware_soft_timer_id:
			displayUpdate();
			break;
	}
}



void gecko_server_update(struct gecko_cmd_packet* evt)
{
	int8_t rssi;

	gecko_update(evt);
	switch (BGLIB_MSG_ID(evt->header)){
		case gecko_evt_system_boot_id:

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
				if(connection_flag & BLE_CONNECT) state_machine();
			}

			break;

		case gecko_evt_le_connection_rssi_id:
			rssi = evt->data.evt_le_connection_rssi.rssi;
			uint16_t TX_Power = 0;

			if(rssi > -35)		TX_Power = LE_TX_MIN;
			else if(rssi > -45)	TX_Power= -200;
			else if(rssi > -55)	TX_Power = -150;
			else if(rssi > -65)	TX_Power = -50;
			else if(rssi > -75)	TX_Power = 0;
			else if(rssi > -85)	TX_Power = 50;
			else				TX_Power = LE_TX_MAX;

			LOG_INFO("rssi is %d => TX power is %d\n",rssi,TX_Power);
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

