/***************************************************************************//**
 * @file
 * @brief Silicon Labs BT Mesh Empty Example Project
 * This example demonstrates the bare minimum needed for a Blue Gecko BT Mesh C application.
 * The application starts unprovisioned Beaconing after boot
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include <stdlib.h>
#include <stdio.h>

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
#include <gecko_configuration.h>
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#include "src/ble_mesh_device_type.h"

#include <src/common.h>
#include "src/display.h"
#include "src/log.h"
#include "src/gpio.h"
#include "src/I2C.h"
#include "src/clock_init.h"
#include "src/state_machine_params.h"
#include "src/ble_stack_params.h"
#include "src/gecko_ble_errors.h"
#include "em_core.h"
#include "mesh_generic_model_capi_types.h"
#include "src/mesh_custom_model_map.h"
#include "src/presence_sensor/presence_sensor.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

// bluetooth stack heap
#define MAX_CONNECTIONS 2

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;
uint8_t external_event;
bool provisioning_complete = 0;
const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
  .bluetooth.sleep_clock_accuracy = 100,
  .bluetooth.linklayer_priorities = &linklayer_priorities,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
  .max_timers = 16,
};

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void mesh_native_bgapi_init(void);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

#if DEVICE_IS_ONOFF_PUBLISHER
/***************************************************************************//**
 * Initialize LPN functionality with configuration and friendship establishment.
 * (code taken from Silicon Labs switch example)
 ******************************************************************************/
void lpn_init(void)
{
	uint16 res;
	// Initialize LPN functionality.
	res = gecko_cmd_mesh_lpn_init()->result;
	if (res) {
		LOG_INFO("LPN init failed (0x%x)", res);
		return;
	}

	// Configure the lpn with following parameters:
	// - Minimum friend queue length = 2
	// - Poll timeout = 5 seconds
	res = gecko_cmd_mesh_lpn_configure(2, 5 * 1000)->result;
	if (res) {
		LOG_INFO("LPN conf failed (0x%x)", res);
		return;
	}

	LOG_INFO("trying to find friend...");
	res = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

	if (res != 0) {
		LOG_INFO("ret.code %x", res);
	}
}
#endif

/**
 * See main function list in soc-btmesh-switch project file
 */
void gecko_bgapi_classes_init_client_lpn(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	//mesh_native_bgapi_init();
	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	gecko_bgapi_class_mesh_generic_client_init();
	//gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	gecko_bgapi_class_mesh_lpn_init();
	//gecko_bgapi_class_mesh_friend_init();
}

/**
 * See light switch app.c file definition
 */
void gecko_bgapi_classes_init_server_friend(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	//mesh_native_bgapi_init();
	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	//gecko_bgapi_class_mesh_generic_client_init();
	gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	//gecko_bgapi_class_mesh_lpn_init();
	gecko_bgapi_class_mesh_friend_init();
}

/**
*	@brief 		Write and set device address
*/
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

#if DEVICE_IS_ONOFF_PUBLISHER
  sprintf(name, "PUBLISHER %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
#else
  sprintf(name, "SUBSCRIBER %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
#endif

  // write device name to the GATT database
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res) {
    LOG_INFO("gecko_cmd_gatt_server_write_attribute_value() failed, code %x", res);
  }
  displayPrintf(DISPLAY_ROW_CLIENTADDR, "Home Automation");
  displayPrintf(DISPLAY_ROW_NAME, "%s", name);
  displayPrintf(DISPLAY_ROW_BTADDR, "%x:%x:%x:%x:%x:%x", pAddr->addr[0], pAddr->addr[1], pAddr->addr[2], pAddr->addr[3], pAddr->addr[4], pAddr->addr[5]);
  LOG_INFO("device name set");
}

void gecko_main_init()
{
// Initialize device
  initMcu();
// Initialize board
  initBoard();
// Initialize application
  initApp();

  gpioInit();
// temperature sensor setting
  Clock_Init();
  i2c_Init();

//Initialize LCD
  displayInit();
//Log
  logFlush();
// Minimize advertisement latency by allowing the advertiser to always interrupt the scanner.
  linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

  gecko_stack_init(&config);

  if( DeviceUsesClientModel() ) {
	  gecko_bgapi_classes_init_client_lpn();
  } else {
	  gecko_bgapi_classes_init_server_friend();
  }

// Initialize coexistence interface. Parameters are taken from HAL config.
  gecko_initCoexHAL();

}

/**
*	@brief 		Handler for Friend node mesh event (Subscriber, Server model)
*/
void handle_gecko_server_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  switch (evt_id) {
    case gecko_evt_system_boot_id:
    	if (GPIO_PinInGet(gpioPortF, 6) == 0 || GPIO_PinInGet(gpioPortF, 7) == 0) {
    		gecko_cmd_flash_ps_erase_all();
    		gecko_cmd_hardware_set_soft_timer(32768*2, TIMER_ID_FACTORY_RESET, 1);
    		displayPrintf(DISPLAY_ROW_ACTION, "Factory Reset");
    		LOG_INFO("factory reset");
    	} else {
    		LOG_INFO("boot done");
    		struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
    		set_device_name(&pAddr->address);
    		gecko_cmd_mesh_node_init();
		}
    	break;

    case gecko_evt_mesh_node_initialized_id:
    	LOG_INFO("in mesh node initialized");

    	//load persistent data in the beginning
    	LOG_INFO("load persistent data",home_state_load());

    	struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

    	if (pData->provisioned) {
    		mesh_lib_init(malloc,free,9);
    		init_models();
    		gecko_cmd_mesh_generic_server_init();
    		//for friend functionality
    		LOG_INFO("Friend mode initialization");
    		uint16 res;
    		res = gecko_cmd_mesh_friend_init()->result;
    		if (res) {
    			LOG_INFO("Friend init failed 0x%x", res);
    		}
    		LOG_INFO("node is provisioned");
    		displayPrintf(DISPLAY_ROW_CONNECTION, "Provisioned");
    		provisioning_complete = 1;
    	} else {
    		LOG_INFO("node is unprovisioned");
    		displayPrintf(DISPLAY_ROW_CONNECTION, "Unprovisioned");
    		gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
    	}
    	break;

    case gecko_evt_mesh_node_provisioning_started_id:
    	displayPrintf(DISPLAY_ROW_ACTION, "Provisioning");
    	LOG_INFO("provisioning started");
    	break;

    case gecko_evt_mesh_node_provisioned_id:
		mesh_lib_init(malloc,free,9);
		init_models();
		gecko_cmd_mesh_generic_server_init();
		//for friend functionality
		LOG_INFO("Friend mode initialization");
		uint16 res;
		res = gecko_cmd_mesh_friend_init()->result;
		if (res) {
			LOG_INFO("Friend init failed 0x%x", res);
		}
		LOG_INFO("node is provisioned");
		provisioning_complete = 1;
		displayPrintf(DISPLAY_ROW_CONNECTION, "Provisioned");
    	break;

    case gecko_evt_mesh_node_provisioning_failed_id:
    	LOG_INFO("provisioning failed, code %x", evt->data.evt_mesh_node_provisioning_failed.result);
    	displayPrintf(DISPLAY_ROW_ACTION, "Provisioning Failed");
    	/* start a one-shot timer that will trigger soft reset after small delay of 2 seconds*/
    	gecko_cmd_hardware_set_soft_timer(32768*2, TIMER_ID_RESTART, 1);
    	provisioning_complete = 0;
    	break;

    	//while friend node receiving any known requests, this event will be triggered
    case gecko_evt_mesh_generic_server_client_request_id:
    	mesh_lib_generic_server_event_handler(evt);
    	break;

    case gecko_evt_hardware_soft_timer_id:
    	switch (evt->data.evt_hardware_soft_timer.handle) {
    		case DISPLAY_REFRESH:
    			displayUpdate();
    			break;
    		case LOG_REFRESH:
    			tickCount = tickCount + 10;
    			break;
    		case TIMER_ID_FACTORY_RESET:
				// reset the device to finish factory reset
				gecko_cmd_system_reset(0);
				break;

    		case TIMER_ID_RESTART:
    			// restart timer expires, reset the device
    			gecko_cmd_system_reset(0);
    			break;
    	}
    	break;

	case gecko_evt_le_connection_opened_id:
		LOG_INFO("in connection opened id");
		displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
		num_connections++;
		break;

    case gecko_evt_le_connection_closed_id:
      /* Check if need to boot to dfu mode */
      if (boot_to_dfu) {
        /* Enter to DFU OTA mode */
        gecko_cmd_system_reset(2);
      }
      LOG_INFO("in connection closed id");
      if (num_connections > 0) {
    	  if (--num_connections == 0) {
    		  displayPrintf(DISPLAY_ROW_CONNECTION, " ");
    	  }
      }
      break;
    case gecko_evt_mesh_friend_friendship_established_id:
    	LOG_INFO("evt gecko_evt_mesh_friend_friendship_established, lpn_address=%x\r\n", evt->data.evt_mesh_friend_friendship_established.lpn_address);
    	displayPrintf(DISPLAY_ROW_BTADDR2, "Friend");
        break;

    case gecko_evt_mesh_friend_friendship_terminated_id:
    	LOG_INFO("evt gecko_evt_mesh_friend_friendship_terminated, reason=%x\r\n", evt->data.evt_mesh_friend_friendship_terminated.reason);

        break;

	case gecko_evt_system_external_signal_id:
		//LOG_INFO("in server external signal id %d",evt->data.evt_system_external_signal.extsignals);
		if((evt->data.evt_system_external_signal.extsignals & TEMPREAD_FLAG))
		{
			CORE_CRITICAL_SECTION(
				external_event &= ~TEMPREAD_FLAG;
			)
			if(provisioning_complete) {
				state_machine();
			}
		}
		if ((evt->data.evt_system_external_signal.extsignals & BUTTON0_FLAG) != 0)
		{

			CORE_CRITICAL_SECTION(
				external_event &= ~BUTTON0_FLAG;
			)
			if(GPIO_PinInGet(gpioPortF,6) == 1)
			{
				LOG_INFO("released");
			}
			else if(GPIO_PinInGet(gpioPortF,6) == 0)
			{
				LOG_INFO("pressed");
			}

		}
		if ((evt->data.evt_system_external_signal.extsignals & F_PRESENCE_FLAG) != 0)
		{
			CORE_CRITICAL_SECTION(
				external_event &= ~F_PRESENCE_FLAG;
			)
			if(presence_order_flag & B_PRESENCE_FLAG)
			{
				LOG_INFO("go out");
				people_count --;
				if(people_count == 0 || people_count == 255){
					presence_state = false;
					displayPrintf(DISPLAY_ROW_ACTION, "Room clear");
					people_count = 0;
					gpioLed0SetOff();
				}
				else displayPrintf(DISPLAY_ROW_ACTION, "People in : %d",people_count);
				presence_order_flag = 0;
				state_store(PRESENCE_STATE);
				publish_room_state();
				LOG_INFO("Room state : %d people : %d\n",presence_state,people_count);
			}else{
				presence_order_flag = F_PRESENCE_FLAG;
			}

		}
		if ((evt->data.evt_system_external_signal.extsignals & B_PRESENCE_FLAG) != 0)
		{
			CORE_CRITICAL_SECTION(
				external_event &= ~B_PRESENCE_FLAG;
			)
				if(presence_order_flag & F_PRESENCE_FLAG)
				{
					LOG_INFO("get in");
					people_count ++;
					presence_state = true;
					presence_order_flag = 0;
					gpioLed0SetOn();
					state_store(PRESENCE_STATE);
					publish_room_state();
					displayPrintf(DISPLAY_ROW_ACTION, "People in : %d",people_count);
					LOG_INFO("Room state : %d people : %d\n",presence_state,people_count);
				}else{
					presence_order_flag = B_PRESENCE_FLAG;
				}
		}
		break;

	case gecko_evt_mesh_node_reset_id:
		LOG_INFO("in mesh node reset id");
		gecko_cmd_flash_ps_erase_all();
		// reset mesh node
		gecko_cmd_hardware_set_soft_timer(32768*2, TIMER_ID_RESTART, 1);
		break;

    case gecko_evt_gatt_server_user_write_request_id:
      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
        /* Set flag to enter to OTA mode */
        boot_to_dfu = 1;
        /* Send response to Write Request */
        gecko_cmd_gatt_server_send_user_write_response(
          evt->data.evt_gatt_server_user_write_request.connection,
          gattdb_ota_control,
          bg_err_success);

        /* Close connection to enter to DFU OTA mode */
        gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
      }
      break;
    default:
      break;
  }
}

#if DEVICE_IS_ONOFF_PUBLISHER
//publisher
void handle_gecko_client_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  switch (evt_id) {
    case gecko_evt_system_boot_id:
    	if (GPIO_PinInGet(gpioPortF, 6) == 0 || GPIO_PinInGet(gpioPortF, 7) == 0) {
    		gecko_cmd_flash_ps_erase_all();
    		gecko_cmd_hardware_set_soft_timer(32768*2, TIMER_ID_FACTORY_RESET, 1);
    		displayPrintf(DISPLAY_ROW_ACTION, "Factory Reset");
    		LOG_INFO("factory reset");
    	} else {
    		LOG_INFO("boot done");
    		struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
    		set_device_name(&pAddr->address);
    		gecko_cmd_mesh_node_init();
		}
    	break;

    case gecko_evt_mesh_node_initialized_id:
    	LOG_INFO("in mesh node initialized");

    	struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

    	if (pData->provisioned) {
    		mesh_lib_init(malloc,free,8);
    		gecko_cmd_mesh_generic_client_init();
    		lpn_init();
    		LOG_INFO("node is provisioned");
    		displayPrintf(DISPLAY_ROW_ACTION, "Provisioned");
    	} else {
    		LOG_INFO("node is unprovisioned");
    		displayPrintf(DISPLAY_ROW_ACTION, "Unprovisioned");
    		gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
    	}
    	break;

    case gecko_evt_mesh_node_provisioning_started_id:
    	displayPrintf(DISPLAY_ROW_ACTION, "Provisioning");
    	LOG_INFO("provisioning started");
    	break;

    case gecko_evt_mesh_node_provisioned_id:
    		mesh_lib_init(malloc,free,8);
    		gecko_cmd_mesh_generic_client_init();
    		lpn_init();

    		LOG_INFO("node is provisioned");
    		displayPrintf(DISPLAY_ROW_ACTION, "Provisioned");
    	break;

    case gecko_evt_mesh_node_provisioning_failed_id:
    	LOG_INFO("provisioning failed, code %x", evt->data.evt_mesh_node_provisioning_failed.result);
    	displayPrintf(DISPLAY_ROW_ACTION, "Provisioning Failed");
    	/* start a one-shot timer that will trigger soft reset after small delay of 2 seconds*/
    	gecko_cmd_hardware_set_soft_timer(32768*2, TIMER_ID_RESTART, 1);
    	break;

    case gecko_evt_hardware_soft_timer_id:
    	switch (evt->data.evt_hardware_soft_timer.handle) {
    		case DISPLAY_REFRESH:
    			displayUpdate();
    			break;
    		case LOG_REFRESH:
    			tickCount = tickCount + 10;
    			break;
    		case TIMER_ID_FACTORY_RESET:
				// reset the device to finish factory reset
				gecko_cmd_system_reset(0);
				break;

    		case TIMER_ID_RESTART:
    			// restart timer expires, reset the device
    			gecko_cmd_system_reset(0);
    			break;

			// case to find friend after particular interval
    		case TIMER_ID_FRIEND_FIND:
    		{
    			LOG_INFO("trying to find friend...");
    			uint16_t result;
    			result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;
    			if (result != 0) {
    				LOG_INFO("ret.code %x", result);
    			}
    		}
    		break;
    	}
    	break;

	case gecko_evt_le_connection_opened_id:
		LOG_INFO("in connection opened id");
		displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
		num_connections++;
		// turn off lpn feature after GATT connection is opened
		gecko_cmd_mesh_lpn_deinit();
		displayPrintf(DISPLAY_ROW_BTADDR2, "LPN off");
		break;

    case gecko_evt_le_connection_closed_id:
      /* Check if need to boot to dfu mode */
      if (boot_to_dfu) {
        /* Enter to DFU OTA mode */
        gecko_cmd_system_reset(2);
      }
      LOG_INFO("in connection closed id");
      if (num_connections > 0) {
    	  if (--num_connections == 0) {
    		  displayPrintf(DISPLAY_ROW_CONNECTION, " ");
    		  lpn_init();
    	  }
      }
      break;

    case gecko_evt_mesh_lpn_friendship_established_id:
    	LOG_INFO("friendship established");
    	displayPrintf(DISPLAY_ROW_BTADDR2, "LPN");
    	break;

    case gecko_evt_mesh_lpn_friendship_failed_id:
    	LOG_INFO("friendship failed");
    	displayPrintf(DISPLAY_ROW_BTADDR2, "no friend");
    	// try again in 2 seconds
    	gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FRIEND_FIND, 1);
    	break;

    case gecko_evt_mesh_lpn_friendship_terminated_id:
    	LOG_INFO("friendship terminated");
    	displayPrintf(DISPLAY_ROW_BTADDR2, "friend lost");
    	if (num_connections == 0) {
    		// try again in 2 seconds
    		gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FRIEND_FIND, 1);
    	}
    	break;


	case gecko_evt_system_external_signal_id:
		LOG_INFO("in external signal id");
		if ((evt->data.evt_system_external_signal.extsignals & BUTTON1_FLAG) != 0)
		{
			struct mesh_generic_request req;
			uint16_t resp;
			uint8_t transition = 0;
			uint8_t delay = 0;
			trid++;

		//TEST brightness model with button 1 4/24 Tim (works!)
			req.kind = brightness_request;
			if(GPIO_PinInGet(gpioPortF,7) == 1)
			{
				req.brightness_level = 2000;
				LOG_INFO("released");
			}
			else if(GPIO_PinInGet(gpioPortF,7) == 0)
			{
				req.brightness_level = 3000;
				LOG_INFO("pressed");
			}
			resp = mesh_lib_generic_client_publish(BRIGHTNESS_LPN_MODEL_ID, 0, trid, &req, transition, delay, 0);
			BTSTACK_LOG_RESULT(mesh_lib_generic_client_publish,resp);
		}

		if ((evt->data.evt_system_external_signal.extsignals & BUTTON0_FLAG) != 0)
		{
			struct mesh_generic_request req;
			uint16_t resp;
			uint8_t transition = 0;
			uint8_t delay = 0;
			req.kind = button_request;
			trid++;

			if(GPIO_PinInGet(gpioPortF,6) == 1)
			{
				req.button_state = WINDOW_ON;
				LOG_INFO("released / open the window");
			}
			else if(GPIO_PinInGet(gpioPortF,6) == 0)
			{
				req.button_state = WINDOW_OFF;
				LOG_INFO("pressed / close the window");
			}
			resp = mesh_lib_generic_client_publish(BUTTON_LPN_MODEL_ID, 0, trid, &req, transition, delay, 0);

			if (resp) {
				LOG_INFO("publish fail");
			} else {
				LOG_INFO("Transaction ID = %u", trid);
			}
		}
		break;

	case gecko_evt_mesh_node_reset_id:
		LOG_INFO("in mesh node reset id");
		gecko_cmd_flash_ps_erase_all();
		// reset mesh node
		gecko_cmd_hardware_set_soft_timer(32768*2, TIMER_ID_RESTART, 1);
		break;

    case gecko_evt_gatt_server_user_write_request_id:
      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
        /* Set flag to enter to OTA mode */
        boot_to_dfu = 1;
        /* Send response to Write Request */
        gecko_cmd_gatt_server_send_user_write_response(
          evt->data.evt_gatt_server_user_write_request.connection,
          gattdb_ota_control,
          bg_err_success);

        /* Close connection to enter to DFU OTA mode */
        gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
      }
      break;
    default:
      break;
  }
}
#endif

/**
*
*	@brief 		initialize the model handler with request callback function
*
*/
static void init_models(void)
{
	LOG_INFO("init model here!\n");
	mesh_lib_generic_server_register_handler(BUTTON_FRIEND_MODEL_ID,
												0,
												sm_request,
												sm_change);
	mesh_lib_generic_server_register_handler(BRIGHTNESS_FRIEND_MODEL_ID,
												0,
												br_request,
												br_change);

}

/**
*
*	@brief 		Callback function while Friend node receive generic on/off request from LPN, and change the window state according to received value
*
*/
static void sm_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{

	if(request->button_state == BUTTON_0_RELEASE){
		displayPrintf(DISPLAY_ROW_TEMPVALUE, "Window Closed");
		window_state = BUTTON_0_RELEASE;
		state_store(WINDOW_STATE);
		LOG_INFO("Window Closed : %d",window_state);

	}
	else if(request->button_state == BUTTON_0_PRESS){
		displayPrintf(DISPLAY_ROW_TEMPVALUE, "Window Opened");
		window_state = BUTTON_0_PRESS;
		state_store(WINDOW_STATE);
		LOG_INFO("Window Opened : %d",window_state);
	}
}

static void sm_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
	LOG_INFO("PB0 State Changed");
}

/**
*
*	@brief 		Callback function while Friend node receive lightness request from LPN, and change the lightness according to received value
*
*/
static void br_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
	displayPrintf(DISPLAY_ROW_LPN, "BR: %d (%%)",request->level);
	if(request->level >=50) gpioLed1SetOff();
	else gpioLed1SetOn();
	lightness_level = request->level;
	state_store(LIGHTNESS_STATE);
}

static void br_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
	LOG_INFO("BR State Changed");
}

/**
*
*	@brief 		Publish room state to LPN with server on/off model
*
*/
void publish_room_state(void)
{
	errorcode_t resp;

	room_state.kind = mesh_generic_state_on_off;
	room_state.on_off.on = presence_state;
	LOG_INFO("room state request send %d\r\n",room_state.on_off.on);

	resp = mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
				  	  	  	  	  	  	  	  	0,
		                                        &room_state,
		                                        0,
		                                        0);
	if (resp) {
		LOG_ERROR("Generic server update failed, code = %x\r\n", resp);
	}
	else {
		LOG_INFO("Generic server updated\r\n");
		resp = mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
													0,
													mesh_generic_state_on_off);

		if (resp) {
			LOG_ERROR("Publish failed, code = %x\r\n", resp);
		} else {
			LOG_INFO("Published room state \n", resp);
		}
	}
}


/**
*	@brief 		Store states and lightness value to flash memory
*
*	@param		uint8_t state_index		index of model whose data need to be stored
*
*	@return   	0 if all states and value are saved successfully
*				1 one of these persistent data saving is failed
*/
int state_store(uint8_t state_index )
{
  struct gecko_msg_flash_ps_save_rsp_t* pSave;
  if (state_index == LIGHTNESS_STATE)
  {
	  pSave = gecko_cmd_flash_ps_save(0x4004, 2, (const uint8*)&lightness_level);
	  LOG_INFO("lightness value stored %d\n",lightness_level);
	  if (pSave->result) {
		printf("Lightness state store : PS save failed, code %x\r\n", pSave->result);
		return(-1);
	  }
  }else if (state_index == PRESENCE_STATE)
  {
	  pSave = gecko_cmd_flash_ps_save(0x400c, 1, (const uint8*)&people_count);
	  LOG_INFO("Room state value stored %d\n",people_count);
	  if (pSave->result) {
		printf("Room state store : PS save failed, code %x\r\n", pSave->result);
		return(-1);
	  }
  }if (state_index == WINDOW_STATE)
  {
	  pSave = gecko_cmd_flash_ps_save(0x4010, 1, (const uint8*)&window_state);
	  LOG_INFO("Window state value stored %d\n",window_state);
	  if (pSave->result) {
		printf("Window state store : PS save failed, code %x\r\n", pSave->result);
		return(-1);
	  }
  }
  return 0;
}



/**
*	@brief 		Recover states and lightness value from flash memory
*
*	@param
*
*	@return   	0 if all states and value are loaded successfully
*				1 one of these persistent data loading is failed
*/
static int home_state_load(void)
{
  struct gecko_msg_flash_ps_load_rsp_t* pLoad;
  LOG_INFO("persistent data load");


  //load lightness level
  pLoad = gecko_cmd_flash_ps_load(0x4004);
  if (pLoad->result) {
    memset(&lightness_level, 0, 2);
    return -1;
  }
  memcpy(&lightness_level, pLoad->value.data, pLoad->value.len);
  LOG_INFO("Lightness data load %d",lightness_level);
  displayPrintf(DISPLAY_ROW_LPN, "BR: %d (%%)",lightness_level);


  //load presence state
  pLoad = gecko_cmd_flash_ps_load(0x400c);
  if (pLoad->result) {
    memset(&people_count, 0, 1);
    return -1;
  }
  memcpy(&people_count, pLoad->value.data, pLoad->value.len);
  LOG_INFO("Room state load %d",people_count);
  if(people_count == 0){
	displayPrintf(DISPLAY_ROW_ACTION, "Room clear");
	presence_state = 0;
	gpioLed0SetOff();
}
  else{
	gpioLed0SetOn();
	displayPrintf(DISPLAY_ROW_ACTION, "People in : %d",people_count);
  }

  //load window state
  pLoad = gecko_cmd_flash_ps_load(0x4010);
  if (pLoad->result) {
    memset(&presence_state, 0, 1);
    return -1;
  }
  memcpy(&window_state, pLoad->value.data, pLoad->value.len);
  LOG_INFO("Window state load %d",window_state);
  if(window_state)displayPrintf(DISPLAY_ROW_TEMPVALUE, "Window Opened");
  else displayPrintf(DISPLAY_ROW_TEMPVALUE, "Window Opened");
  return 0;
}
