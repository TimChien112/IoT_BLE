#include <stdbool.h>
#include <native_gecko.h>
#include "log.h"
#include "ble_mesh_device_type.h"

extern void gecko_main_init();
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
extern void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

int main(void)
{

  // Initialize stack
  gecko_main_init();

  logInit();
  bool device_type = DeviceUsesClientModel();
  /* Infinite loop */
  while (1) {
	struct gecko_cmd_packet *evt = gecko_wait_event();
	bool pass = mesh_bgapi_listener(evt);
	if (pass) {
		if(device_type){
			handle_gecko_client_event(BGLIB_MSG_ID(evt->header), evt);
		}
		else{
			handle_gecko_server_event(BGLIB_MSG_ID(evt->header), evt);
		}
	}
  };
}
