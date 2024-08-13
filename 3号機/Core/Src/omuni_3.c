/*
 * omuni_3.c
 *
 *  Created on: 2024/06/09
 *      Author: aiueo
 */

#include "omuni_3.h"

void motor_1(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,ina);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,inb);
}
void motor_2(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,ina);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,inb);
}
void motor_3(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,ina);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,inb);
}


void omuni(uint8_t command , uint16_t speed){
	switch(command){
	case 0:
		motor_1(0,0);
		motor_2(0,0);
		motor_3(0,0);

		break;
	case 1:
		motor_1(0,0);
		motor_2(0 ,speed);
		motor_3(speed,0 );

	  break;

	case 2:
		motor_1(0,0);
		motor_2(speed,0 );
		motor_3(0 ,speed);

	  break;

	case 3:
		motor_1(0,speed);
		motor_2(speed * 0.5 ,0);
		motor_3(speed * 0.5 ,0);

	  break;

	case 4:
		motor_1(speed,0);
		motor_2(0,speed * 0.5 );
		motor_3(0,speed * 0.5 );

		break;
	case 5:
		motor_1(speed,0 );
		motor_2(speed,0 );
		motor_3(speed,0 );

		break;

	case 6:
		motor_1(0 ,speed);
		motor_2(0 ,speed);
		motor_3(0 ,speed);

		break;
	default:
		motor_1(0,0);
		motor_2(0,0);
		motor_3(0,0);
	}
}


