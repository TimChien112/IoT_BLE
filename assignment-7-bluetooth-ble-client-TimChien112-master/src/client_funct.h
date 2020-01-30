/*
 * client_funct.h
 *
 *  Created on: 2019¦~2¤ë27¤é
 *      Author: tim01
 */

#ifndef SRC_CLIENT_FUNCT_H_
#define SRC_CLIENT_FUNCT_H_
#include <stdint.h>

// Init connection properties
void initProperties(void);

// Parse advertisements looking for advertised Health Thermometer service
uint8_t findServiceInAdvertisement(uint8_t *data, uint8_t len);

uint8_t findIndexByConnectionHandle(uint8_t connection);

void addConnection(uint8_t connection, uint16_t address);

void removeConnection(uint8_t connection);

#endif /* SRC_CLIENT_FUNCT_H_ */
