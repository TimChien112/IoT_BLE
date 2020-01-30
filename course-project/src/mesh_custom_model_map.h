/*
 * mesh_custom_model_map.h
 *
 *  Created on: Apr 25, 2019
 *      Author: danwa
 */

#ifndef SRC_MESH_CUSTOM_MODEL_MAP_H_
#define SRC_MESH_CUSTOM_MODEL_MAP_H_


// customized models for push button
// will be implemented for sensor in future
#define BUTTON_FRIEND_MODEL_ID                                                 	0x1000
/**
 * 0x1001 is MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID.  Reuse this model ID for button request
 */
#define BUTTON_LPN_MODEL_ID                                                    	0x1001
#define button_request mesh_generic_request_on_off
#define button_state	on_off

#define BRIGHTNESS_FRIEND_MODEL_ID                                             	0x1300
/**
 * 0x1302 is MESH_LIGHTING_LIGHTNESS_CLIENT_MODEL_ID
 */
#define BRIGHTNESS_LPN_MODEL_ID                                                	0x1302
#define brightness_request mesh_lighting_request_lightness_actual
#define brightness_level lightness

#define SMOKE_FRIEND_MODEL_ID                                                  	0x1002
#define SMOKE_LPN_MODEL_ID                                                     	0x1003
#define smoke_request mesh_lighting_request_lightness_actual
#define smoke_level lightness

#define BUTTON_0_RELEASE                                                         0x00
#define BUTTON_0_PRESS                                                           0x01
#define BUTTON_1_RELEASE                                                         0x02
#define BUTTON_1_PRESS                                                           0x03

#define WINDOW_ON		                                                         0x00
#define WINDOW_OFF	                                                             0x01


#endif /* SRC_MESH_CUSTOM_MODEL_MAP_H_ */
