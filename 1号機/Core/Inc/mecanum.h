/*
 * mecanum.h
 *
 *  Created on: 2024/06/10
 *      Author: aiueo
 */

#ifndef INC_MECANUM_H_
#define INC_MECANUM_H_

#include "stm32f7xx_hal.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

void motor_1(uint16_t ina , uint16_t inb);
void motor_2(uint16_t ina , uint16_t inb);
void motor_3(uint16_t ina , uint16_t inb);
void motor_4(uint16_t ina , uint16_t inb);

void mecanum(uint8_t command , uint16_t speed , uint16_t seigyoR , uint16_t seigyoL);

#endif /* INC_MECANUM_H_ */
