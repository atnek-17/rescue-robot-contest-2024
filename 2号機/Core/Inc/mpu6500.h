/*
 * mpu6500.h
 *
 *  Created on: Nov 28, 2023
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include "stm32f7xx_hal.h"

I2C_HandleTypeDef *mpu_I2C;

#define MPU_ADDRESS 0x68		//i2c slave address of the mpu-6500 AD0 = 0

#define FS_GYRO_250	0x00
#define FS_GYRO_500 0x08
#define FS_GYRO_1000 0x10
#define FS_GYRO_2000 0x18

#define FS_ACC_2G 0x00
#define FS_ACC_4G 0x08
#define FS_ACC_8G 0x10
#define FS_ACC_16G 0x18

#define REG_CONFIG_GYRO 0x1B
#define REG_CONFIG_ACC 0x1C
#define PWR_MGMT_1 0x6B
#define REG_DATA 0x3B
#define REG_WHO_AM_I 0x70

int16_t x_acc_raw;
int16_t y_acc_raw;
int16_t z_acc_raw ;
int16_t x_gyro_raw;
int16_t y_gyro_raw;
int16_t z_gyro_raw;
float x_acc;
float y_acc;
float z_acc;
float x_gyro;
float y_gyro;
float z_gyro;

int deg_X;
#define  rad_to_deg  180/3.1415;

void mpu6500_init(I2C_HandleTypeDef *handler);
void mpu6500_read();
void mpu6500_get_deg();

#endif /* INC_MPU6500_H_ */
