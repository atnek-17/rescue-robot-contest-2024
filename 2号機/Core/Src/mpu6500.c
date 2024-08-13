/*
 * mpu6500.c
 *
 *  Created on: Nov 28, 2023
 *      Author: aiueo
 */
#include "mpu6500.h"
#include "main.h"
#include "stdio.h"

void mpu6500_init(I2C_HandleTypeDef *handler)
{

	mpu_I2C = handler;
	//HAL_I2C_Mem_Read(mpu_I2C , MPU_ADDRESS , REG_WHO_AM_I , 1 , &status , 1 , 1000);

	uint8_t temp_data = 0x00;
	HAL_I2C_Mem_Write(mpu_I2C , MPU_ADDRESS << 1  , PWR_MGMT_1 , 1 , &temp_data , 1 , 100);

	temp_data = FS_GYRO_250 ;
	HAL_I2C_Mem_Write(mpu_I2C , MPU_ADDRESS << 1 , REG_CONFIG_GYRO , 1 , &temp_data , 1 , 100); //configuration gyroscope

	temp_data = FS_ACC_2G;
	HAL_I2C_Mem_Write(mpu_I2C , MPU_ADDRESS << 1 , REG_CONFIG_ACC , 1 , &temp_data , 1 , 100);	//configuration accer

}

void mpu6500_read()
{
	uint8_t buf[14];


	HAL_I2C_Mem_Read(mpu_I2C , MPU_ADDRESS << 1  , REG_DATA , 1 , buf , 14 , 1000);
	x_acc_raw = buf[0] << 8 | buf[1];
	y_acc_raw = buf[2] << 8 | buf[3];
	z_acc_raw = buf[4] << 8 | buf[5];

	x_gyro_raw = buf[8] << 8 | buf[9];
	y_gyro_raw = buf[10] << 8 | buf[11];
	z_gyro_raw = buf[12] << 8 | buf[13];


	x_acc = (float)x_acc_raw / 16384;
	y_acc = (float)y_acc_raw / 16384;
	z_acc = (float)z_acc_raw / 16384;

	x_gyro = (float)x_gyro_raw / 131 ;
	y_gyro = (float)y_gyro_raw / 131 ;
	z_gyro = (float)z_gyro_raw / 131 ;

}

void mpu6500_get_deg(){
	mpu6500_read();
	deg_X = atan2(y_acc , z_acc) * rad_to_deg ;
}
