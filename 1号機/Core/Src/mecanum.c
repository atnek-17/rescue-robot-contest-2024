/*
 * mecanum.c
 *
 *  Created on: 2024/06/10
 *      Author: aiueo
 */

#include "mecanum.h"


void motor_1(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,ina);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,inb);
}
void motor_2(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,inb);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,ina);
}
void motor_3(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,ina);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,inb);
}
void motor_4(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,inb);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,ina);
}

void mecanum(uint8_t command , uint16_t speed , uint16_t seigyoR , uint16_t seigyoL){
	switch(command){
	case 0:			//stop
		motor_1(0,0);
		motor_2(0,0);
		motor_3(0,0);
		motor_4(0,0);

		break;

	case 1:			//forward
		motor_1(speed + seigyoL,0);
		motor_2(speed + seigyoR ,0);
		motor_3(speed + seigyoL,0);
		motor_4(speed + seigyoR ,0);

		break;

	case 2:			//back
		motor_1(0,speed + seigyoR);
		motor_2(0,speed + seigyoL);
		motor_3(0,speed + seigyoR);
		motor_4(0,speed + seigyoL);
		break;

	case 3:			//migi heikou idou
		motor_1(speed,0);
		motor_2(0,speed);
		motor_3(0,speed);
		motor_4(speed,0);
		break;

	case 4:			//hidari heikou idou
		motor_1(0,speed);
		motor_2(speed,0);
		motor_3(speed,0);
		motor_4(0,speed);
		break;

	case 5:			// migi naname mae
		motor_1(speed,0);
		motor_2(0,0);
		motor_3(0,0);
		motor_4(speed,0);
		break;

	case 6:			// hidari naname mae
		motor_1(0,0);
		motor_2(speed,0);
		motor_3(speed,0);
		motor_4(0,0);
		break;

	case 7:			// migi naname usiro
		motor_1(0,speed);
		motor_2(0,0);
		motor_3(0,0);
		motor_4(0,speed);
		break;

	case 8:			// hidari naname usiro
		motor_1(0,0);
		motor_2(0,speed);
		motor_3(0,speed);
		motor_4(0,0);
		break;

	case 9:			// migi senkai
		motor_1(speed,0);
		motor_2(0,speed);
		motor_3(speed,0);
		motor_4(0,speed);
		break;

	case 10:		// hidari senkai
		motor_1(0,speed);
		motor_2(speed,0);
		motor_3(0,speed);
		motor_4(speed,0);
		break;
	default :
		motor_1(0,0);
		motor_2(0,0);
		motor_3(0,0);
		motor_4(0,0);
	}
}

