/*
 * I2C.h
 *
 *  Created on: 2019¦~2¤ë27¤é
 *      Author: tim01
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

void I2CRead_start(void);
void I2CWrite_start(void);

// parameters for I2C transfer function
	uint8_t write_buffer_data;
	uint16_t write_buffer_len;
	uint8_t read_buffer_data[2];
	uint16_t read_buffer_len;


#endif /* SRC_I2C_H_ */
