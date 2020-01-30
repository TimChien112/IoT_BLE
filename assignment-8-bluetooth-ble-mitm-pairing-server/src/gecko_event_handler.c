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
#include "ble_device_type.h"
#include "stdbool.h"
#include "gatt_db.h"
#include "gpiointerrupt.h"
#include "em_core.h"


// Health Thermometer service UUID defined by Bluetooth SIG
const uint8_t thermoService[2] = { 0x09, 0x18 };
// Temperature Measurement characteristic UUID defined by Bluetooth SIG
const uint8_t thermoChar[2] = { 0x1c, 0x2a };

// BP0 service
const uint8_t BP0Service[16] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x01, 0x00, 0x00, 0x00 };
// BP0 characteristic
const uint8_t BP0Char[16] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x02, 0x00, 0x00, 0x00 };

#define MAX_NUM_SERVER	2
BT_SERVER_CONNECTION_T servers[MAX_NUM_SERVER] = {0};

//bond flag
bool waitingForConfirm = false;
bool bonding_complete = false;
//button state
uint8_t bt_state;

uint16_t tempCharacteristicHandle;
uint16_t BP0CharacteristicHandle;

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
			tmrResp = gecko_cmd_hardware_set_soft_timer(32576,0,0); // 1 sec

			// Start scanning - looking for thermometer devices
			gecko_cmd_le_gap_start_discovery(le_gap_phy_1m, le_gap_discover_generic);
			break;

		case gecko_evt_le_gap_scan_response_id:
				gecko_cmd_le_gap_end_procedure();
#if enable_bonding
				gecko_cmd_flash_ps_erase_all();
				gecko_cmd_sm_delete_bondings();
				/* enable bondable to accommodate certain mobile OS */
				gecko_cmd_sm_set_bondable_mode(1);
				gecko_cmd_sm_configure(0x0F, sm_io_capability_displayyesno); /* Numeric comparison */
#endif
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

#if enable_bonding

			gecko_cmd_sm_increase_security(evt->data.evt_le_connection_opened.connection);
#else
			// Discover Health Thermometer service on the slave device
			gecko_cmd_gatt_discover_primary_services_by_uuid(evt->data.evt_le_connection_opened.connection,
															 2,
															 thermoService);
#endif
			displayPrintf(DISPLAY_ROW_CONNECTION,"Connected");
			LOG_INFO("Connected here\n");
			break;


		case gecko_evt_gatt_service_id:		// This event is generated when a new service is discovered
			//LOG_INFO("Service here!!\n");
			servers[0].servicesHandle[servers[0].numOfServices] = evt->data.evt_gatt_service.service;
			memcpy(&servers[0].servicesUUID[servers[0].numOfServices].data[0],
					&evt->data.evt_gatt_service.uuid.data[0],
					evt->data.evt_gatt_service.uuid.len);
			LOG_INFO("Discovered Service Handle: %lu, UUID: 0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x \n",
					servers[0].servicesHandle[servers[0].numOfServices],
					servers[0].servicesUUID[servers[0].numOfServices].data[0],
					servers[0].servicesUUID[servers[0].numOfServices].data[1],
					servers[0].servicesUUID[servers[0].numOfServices].data[2],
					servers[0].servicesUUID[servers[0].numOfServices].data[3],
					servers[0].servicesUUID[servers[0].numOfServices].data[4],
					servers[0].servicesUUID[servers[0].numOfServices].data[5],
					servers[0].servicesUUID[servers[0].numOfServices].data[6],
					servers[0].servicesUUID[servers[0].numOfServices].data[7],
					servers[0].servicesUUID[servers[0].numOfServices].data[8],
					servers[0].servicesUUID[servers[0].numOfServices].data[9],
					servers[0].servicesUUID[servers[0].numOfServices].data[10],
					servers[0].servicesUUID[servers[0].numOfServices].data[11],
					servers[0].servicesUUID[servers[0].numOfServices].data[12],
					servers[0].servicesUUID[servers[0].numOfServices].data[13],
					servers[0].servicesUUID[servers[0].numOfServices].data[14],
					servers[0].servicesUUID[servers[0].numOfServices].data[15]);
			servers[0].numOfServices++;
			break;

		      // This event is generated when a new characteristic is discovered
		case gecko_evt_gatt_characteristic_id:
			//LOG_INFO("char here!!\n");
			if((memcmp(&(evt->data.evt_gatt_characteristic.uuid.data[0]),thermoChar,2) == 0))
			{
				tempCharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
				LOG_INFO("Temp char handle: %d\n",tempCharacteristicHandle);
				LOG_INFO("Temp char UUID: 0x%02x%02x\n",
						evt->data.evt_gatt_characteristic.uuid.data[0],
						evt->data.evt_gatt_characteristic.uuid.data[1]);
			}
			if((memcmp(&(evt->data.evt_gatt_characteristic.uuid.data[0]),BP0Char,16) == 0))
			{
				BP0CharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
				LOG_INFO("BP0 char handle: %d\n",BP0CharacteristicHandle);
				LOG_INFO("BP0 char UUID: 0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
						evt->data.evt_gatt_characteristic.uuid.data[0],
						evt->data.evt_gatt_characteristic.uuid.data[1],
						evt->data.evt_gatt_characteristic.uuid.data[2],
						evt->data.evt_gatt_characteristic.uuid.data[3],
						evt->data.evt_gatt_characteristic.uuid.data[4],
						evt->data.evt_gatt_characteristic.uuid.data[5],
						evt->data.evt_gatt_characteristic.uuid.data[6],
						evt->data.evt_gatt_characteristic.uuid.data[7],
						evt->data.evt_gatt_characteristic.uuid.data[8],
						evt->data.evt_gatt_characteristic.uuid.data[9],
						evt->data.evt_gatt_characteristic.uuid.data[10],
						evt->data.evt_gatt_characteristic.uuid.data[11],
						evt->data.evt_gatt_characteristic.uuid.data[12],
						evt->data.evt_gatt_characteristic.uuid.data[13],
						evt->data.evt_gatt_characteristic.uuid.data[14],
						evt->data.evt_gatt_characteristic.uuid.data[15]);
			}
			break;

		case gecko_evt_gatt_procedure_completed_id:
			// If service discovery finished
			LOG_INFO("Procedure complete error code:%x",evt->data.evt_gatt_procedure_completed.result);
			switch(servers[0].connectionState)
			{
				case STATE_DISCOVER_SERVICES:
					if(evt->data.evt_gatt_procedure_completed.result == 0)
					{
						servers[0].connectionState = STATE_DISCOVER_CHARACTERISTICS;
						LOG_INFO("state change \n");

					}
					if(servers[0].numOfServices == 2){
						struct gecko_msg_gatt_discover_characteristics_by_uuid_rsp_t *rsp = gecko_cmd_gatt_discover_characteristics_by_uuid(
												servers[0].connectionHandle,
												servers[0].servicesHandle[servers[0].numOfServices-1],
												16,
												BP0Char);

					}
					if(servers[0].numOfServices == 1){
						struct gecko_msg_gatt_discover_characteristics_by_uuid_rsp_t *rsp = gecko_cmd_gatt_discover_characteristics_by_uuid(
												servers[0].connectionHandle,
												servers[0].servicesHandle[servers[0].numOfServices-1],
												2,
												(const uint8_t*)thermoChar);
					}

					break;
			// If characteristic discovery finished
				case STATE_DISCOVER_CHARACTERISTICS:
					if(evt->data.evt_gatt_procedure_completed.result == 0){
						if(BP0CharacteristicHandle > 0 && servers[0].numOfServices == 2){
							servers[0].connectionState = STATE_ENABLE_NOTIFICATIONS;
							struct gecko_msg_gatt_set_characteristic_notification_rsp_t *rsp = gecko_cmd_gatt_set_characteristic_notification(
													servers[0].connectionHandle,
													tempCharacteristicHandle,
													gatt_indication);
//							LOG_INFO("Enable indicate result %02x \n",evt->data.rsp_gatt_set_characteristic_notification.result);
//							struct gecko_msg_gatt_set_characteristic_notification_rsp_t *rsp = gecko_cmd_gatt_set_characteristic_notification(
//													servers[0].connectionHandle,
//													BP0CharacteristicHandle,
//													gatt_indication);
							LOG_INFO("Enable indicate result %02x \n",rsp->result);
						}
						if(tempCharacteristicHandle > 0 && servers[0].numOfServices == 1){
							// Discover Health Thermometer service on the slave device
							servers[0].connectionState = STATE_DISCOVER_SERVICES;
							gecko_cmd_gatt_discover_primary_services_by_uuid(
													servers[0].connectionHandle,
													16,
													BP0Service);
						}
					}else {LOG_INFO("procedure completed error %d \n",evt->data.evt_gatt_procedure_completed.result);}
					break;
				case STATE_ENABLE_NOTIFICATIONS:
					LOG_INFO("Set notification for Button");
					servers[0].connectionState = STATE_ENABLE_NOTIFICATIONS2;
					struct gecko_msg_gatt_set_characteristic_notification_rsp_t *rsp = gecko_cmd_gatt_set_characteristic_notification(
							servers[0].connectionHandle,
							BP0CharacteristicHandle,
							gatt_indication);
					LOG_INFO("Enable indicate result %02x \n",rsp->result);

					break;
				case STATE_ENABLE_NOTIFICATIONS2:
					LOG_INFO("Set notification for temp. Ready");
					servers[0].connectionState = STATE_DATA_NOTIF_READY;
					break;

			}
			break;

		case gecko_evt_gatt_characteristic_value_id: // This event is generated when a characteristic value was received e.g. an indication
//			if(servers[0].connectionState != STATE_DATA_NOTIF_READY) break;
			LOG_INFO("characteristic handle = %d\n",evt->data.evt_gatt_characteristic_value.characteristic);
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
				char tempBuffer[20] = {0};
				snprintf(tempBuffer, sizeof(tempBuffer), "temp %.04f C",temperature_c);
				displayPrintf(DISPLAY_ROW_TEMPVALUE,tempBuffer);
			}
			if(evt->data.evt_gatt_characteristic_value.characteristic == BP0CharacteristicHandle)
			{
				if(evt->data.evt_gatt_characteristic_value.att_opcode == 0x1d){
					struct gecko_msg_gatt_send_characteristic_confirmation_rsp_t *rsp =
							gecko_cmd_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
				}
				bt_state = evt->data.evt_gatt_characteristic_value.value.data[0];
				displayPrintf(DISPLAY_ROW_ACTION,"server PB0:%u",bt_state);
				LOG_INFO("BP0 char value = %d\n",bt_state);
			}
			break;


		case gecko_evt_le_connection_closed_id:
			gecko_cmd_system_set_tx_power(0);
		    /* Restart advertising after client has disconnected */
		    gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
		    LOG_INFO("Conection close here\n");
		    displayPrintf(DISPLAY_ROW_CONNECTION,"Discovering");
		    connection_flag = BLE_DISCONNECT;
		    gecko_cmd_sm_delete_bondings();
		    LOG_INFO("Delete all bondings\n");
		    displayPrintf(DISPLAY_ROW_TEMPVALUE,"tempe unavailable");
		    break;

		case gecko_evt_hardware_soft_timer_id:
			displayUpdate();
			break;


#if enable_bonding
	    case gecko_evt_system_external_signal_id:
	    	  if(waitingForConfirm && (evt->data.evt_system_external_signal.extsignals & PB0_PRESSED))
	    	  {
	    		  CORE_CRITICAL_SECTION(
	    				  external_event &= ~PB0_PRESSED;
	    		  );
	    		  if(bt_state == 1){
					  displayPrintf(DISPLAY_ROW_ACTION,"PB0 pressed");
					  LOG_INFO("PB0 pressed: Confirming the bonding with the server...\n");
					  gecko_cmd_sm_passkey_confirm(servers[0].connectionHandle, 1);
	    		  }
	    	  }
	    	  break;

		case gecko_evt_sm_bonding_failed_id:
			  LOG_INFO("Bonding failed with the server\n");
			  displayPrintf(DISPLAY_ROW_CONNECTION,"Bonded failed!");
			  displayPrintf(DISPLAY_ROW_PASSKEY,"");
			  displayPrintf(DISPLAY_ROW_ACTION,"");
			  waitingForConfirm = false;
			  break;

		case gecko_evt_sm_bonded_id:
			  LOG_INFO("Bonding complete with the server\n");
			  displayPrintf(DISPLAY_ROW_CONNECTION,"Bonded");
			  displayPrintf(DISPLAY_ROW_PASSKEY,"");
			  displayPrintf(DISPLAY_ROW_ACTION,"");
			  waitingForConfirm = false;
			  bonding_complete = true;

			  LOG_INFO("Discovering primary services\n");
			  // Discover (Primary) Health Thermometer service on the slave device
			  gecko_cmd_gatt_discover_primary_services_by_uuid(
					  	  	  	  	  	  servers[0].connectionHandle,
										  2,
										  thermoService);
			  break;


		case gecko_evt_sm_confirm_passkey_id:
			  LOG_INFO("Confirm passkey\n");
			  LOG_INFO("Do you see the same passkey: %lu (y/n)?\n",
					  evt->data.evt_sm_confirm_passkey.passkey);
			  char PASSKEY[20] = {0};
			  snprintf(PASSKEY, sizeof(PASSKEY),"KEY:%lu",evt->data.evt_sm_confirm_passkey.passkey);
			  displayPrintf(DISPLAY_ROW_PASSKEY,PASSKEY);
			  displayPrintf(DISPLAY_ROW_ACTION,"Press PB0 to confirm");
			  displayPrintf(DISPLAY_ROW_CONNECTION,"Bonding...");
			  LOG_INFO("Press Button PB0 to confirm\n");
			  waitingForConfirm = true;
			  break;
#endif

	}
}

uint8_t connHandle = 0;

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
#if enable_bonding
			gecko_cmd_flash_ps_erase_all();
			gecko_cmd_sm_delete_bondings();
			/* enable bondable to accommodate certain mobile OS */
			gecko_cmd_sm_set_bondable_mode(1);

			gecko_cmd_sm_configure(0x0F, sm_io_capability_displayyesno); /* Numeric comparison */
#endif
			break;

		case gecko_evt_le_connection_opened_id:

			connHandle = evt->data.evt_le_connection_opened.connection;
			state_stop_flag = 0; //enable the state machine
#if enable_bonding
			gecko_cmd_sm_increase_security(evt->data.evt_le_connection_opened.connection);
#endif
			gecko_cmd_le_connection_set_parameters(evt->data.evt_le_connection_opened.connection, \
	  	  	  	  	  	  LE_MIN_CONNECTION_INTERVAL_COUNT, \
						  LE_MAX_CONNECTION_INTERVAL_COUNT, \
						  LE_SLAVE_LATENCY, \
						  LE_CONNECTION_TIMEOUT_MS);
			connection_flag = BLE_CONNECT;
			displayPrintf(DISPLAY_ROW_CONNECTION,"Connected");
			LOG_INFO("Connection open here\n");
			break;


		case gecko_evt_le_connection_closed_id:
			state_stop_flag = 1;	//stop the state machine
			displayPrintf(DISPLAY_ROW_TEMPVALUE,"tempe unavailable");
			gecko_cmd_system_set_tx_power(0);
		    /* Restart advertising after client has disconnected */
		    gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
		    LOG_INFO("Conection close here\n");
		    displayPrintf(DISPLAY_ROW_CONNECTION,"Advertising");
		    connection_flag = BLE_DISCONNECT;

		    LOG_INFO("Delete all bondings\n");
		    break;


		case gecko_evt_system_external_signal_id:
			//READ_TEMP();
			if((evt->data.evt_system_external_signal.extsignals & TEMPREAD_FLAG))
			{
				CORE_CRITICAL_SECTION(
						external_event &= ~TEMPREAD_FLAG;
				)
				if(bonding_complete) state_machine();
			}
			if(waitingForConfirm && (evt->data.evt_system_external_signal.extsignals & PB0_PRESSED))
			{
				CORE_CRITICAL_SECTION(
						external_event &= ~PB0_PRESSED;
				)
				if(bt_state == 1){
					  displayPrintf(DISPLAY_ROW_ACTION,"PB0 pressed");
					  LOG_INFO("PB0 pressed: Confirming the bonding with the client...\n");
					  gecko_cmd_sm_passkey_confirm(connHandle, 1);
				  }
			}
			else if( (evt->data.evt_system_external_signal.extsignals & PB0_PRESSED)){
				CORE_CRITICAL_SECTION(
						external_event &= ~PB0_PRESSED;
				)
				LOG_INFO("EXT button state %d\n",bt_state);
				uint8_t state = bt_state ? 0 : 1;
				gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_button_state, 1, &bt_state);
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

			//LOG_INFO("rssi is %d => TX power is %d\n",rssi,TX_Power);
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

#if enable_bonding
		case gecko_evt_sm_bonding_failed_id:
			LOG_INFO("Bonding failed with the client: %u\n", evt->data.evt_sm_bonding_failed.reason);
			static char bondingFailed[15]= {0};
			snprintf(bondingFailed, 10, "Bond Fail:%u",evt->data.evt_sm_bonding_failed.reason);
			displayPrintf(DISPLAY_ROW_CONNECTION,"Bonded failed!");
			displayPrintf(DISPLAY_ROW_PASSKEY,"");
			displayPrintf(DISPLAY_ROW_ACTION,"");
			waitingForConfirm = false;
			bonding_complete = false;
			gecko_cmd_le_connection_close(evt->data.evt_sm_bonding_failed.connection);
			break;

		case gecko_evt_sm_bonded_id:
			LOG_INFO("Bonding complete with the client\n");
			displayPrintf(DISPLAY_ROW_CONNECTION,"Connected/Bonded");
			displayPrintf(DISPLAY_ROW_PASSKEY,"");
			displayPrintf(DISPLAY_ROW_ACTION,"");
			waitingForConfirm = false;
			bonding_complete = true;
			break;

		case gecko_evt_sm_confirm_passkey_id:
			LOG_INFO("Confirm passkey\n");
			LOG_INFO("Do you see the same passkey on the tablet: %lu (y/n)?\n",
				  evt->data.evt_sm_confirm_passkey.passkey);
			char PASSKEY[20] = {0};
			snprintf(PASSKEY, sizeof(PASSKEY),"KEY:%lu",evt->data.evt_sm_confirm_passkey.passkey);
			displayPrintf(DISPLAY_ROW_PASSKEY,PASSKEY);
			displayPrintf(DISPLAY_ROW_ACTION,"Confirm with PB0");
			displayPrintf(DISPLAY_ROW_CONNECTION,"Bonding");
			waitingForConfirm = true;
		  break;
#endif

	}
}


#if enable_bonding
uint8_t external_event = 0;

void buttonPushedCallback(uint8 pin)
{
	if(pin == BSP_BUTTON0_PIN)
	{
		external_event |= PB0_PRESSED;
		LOG_INFO("button state change here \n");
		//LOG_INFO("button pressed here %d\n",bt_state);
		bt_state = GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN);
		gecko_external_signal(external_event);
	}
	if(pin == BSP_BUTTON1_PIN)
	{
		external_event |= PB1_PRESSED;
		gecko_external_signal(external_event);
	}
}
void enable_button_interrupts(void)
{
	GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInput, 0);
	GPIO_PinModeSet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, gpioModeInput, 0);

	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	GPIOINT_Init();
	/* configure interrupt for PB0 and PB1, both falling and rising edges */
	GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, true, true);

	/* register the callback function that is invoked when interrupt occurs */
	GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, buttonPushedCallback);
}
void disable_button_interrupts()
{
	GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeDisabled, 0);

	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

	/* configure interrupt for PB0 and PB1, both falling and rising edges */
	GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, false, false);
}
#endif
