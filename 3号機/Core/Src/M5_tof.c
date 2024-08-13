/*
 * M5_tof.c
 *
 *  Created on: Jul 2, 2024
 *      Author: fukuj
 */

#include "M5_tof.h"

void tof_init(I2C_HandleTypeDef *handler){
	tof_I2C = handler;
}

void write_byte_data_at(uint8_t reg , uint8_t data){
	uint8_t val[1];
	val[0] = data;
	HAL_I2C_Mem_Write(tof_I2C , tof_ADDR , reg , I2C_MEMADD_SIZE_8BIT , val , 1 , 1000);
}

uint8_t read_byte_data_at(uint8_t reg){
	uint8_t value;
	HAL_I2C_Mem_Read(tof_I2C , tof_ADDR , reg , I2C_MEMADD_SIZE_8BIT , &value , 1 , 1000);
	return value;
}

void read_block_data_at(uint8_t reg , int size){
	HAL_I2C_Mem_Read(tof_I2C , tof_ADDR , reg , I2C_MEMADD_SIZE_8BIT , buf , size , 1000);
}

uint16_t convuint16(int lsb , int msb){
	return ((msb & 0xFF) << 8 | (lsb & 0xFF));
}

void tof_read(){
	write_byte_data_at(VL53L0X_REG_SYSRANGE_START , 0x01);

	uint8_t val = 0;
	int cnt = 0;
	while(cnt < 100){
		HAL_Delay(10);
		val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
		if(val & 0x01){
			break;
		}else{
			cnt++;
		}
	}

	read_block_data_at(0x14 , 12);
	distance = convuint16(buf[11] , buf[10]);
}
