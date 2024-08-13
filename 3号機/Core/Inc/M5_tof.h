/*
 * M5_tof.h
 *
 *  Created on: Jul 2, 2024
 *      Author: fukuj
 */

#ifndef INC_M5_TOF_H_
#define INC_M5_TOF_H_

#include "stm32f7xx_hal.h"

I2C_HandleTypeDef *tof_I2C;
#define tof_ADDR 						0x52
#define VL53L0X_REG_SYSRANGE_START 		0x00
#define VL53L0X_REG_RESULT_RANGE_STATUS	0x14
uint8_t buf[16];
uint16_t distance;

void tof_init(I2C_HandleTypeDef *handler);
void write_byte_data_at(uint8_t reg , uint8_t data);
void read_block_data_at(uint8_t reg , int size);
uint16_t convuint16_t(int lsb , int msb);


#endif /* INC_M5_TOF_H_ */
