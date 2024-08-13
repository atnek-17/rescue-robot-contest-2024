/*
 * Vcc3r_uart.c
 *
 *  Created on: 2024/03/27
 *      Author: aiueo
 */

#include "Vcc3r_uart.h"

uint8_t cmd[7];

void vcc3r_init(UART_HandleTypeDef *handler){
	huart_vcc3r = handler;
}

void vcc3r_pan_move(uint8_t command){
	uint8_t cmd[7];
	cmd[0] = 0xFF;
	cmd[1] = 0x30;
	cmd[2] = 0x30;
	cmd[3] = 0x00;
	cmd[4] = 0x53;
	if(command == 1){			//migi
		cmd[5] = 0x31;
	}else if(command == 2){			//hidari
		cmd[5] == 0x32;
	}
	cmd[6] = 0xEF;

	HAL_UART_Transmit(huart_vcc3r , (uint8_t *)cmd , 7 , 0xF);
}

void vcc3r_tilt_move(uint8_t command){
	uint8_t cmd[7];
	cmd[0] = 0xFF;
	cmd[1] = 0x30;
	cmd[2] = 0x30;
	cmd[3] = 0x00;
	cmd[4] = 0x53;
	if(command == 1){			//ue
		cmd[5] = 0x33;
	}else if(command == 2){			//sita
		cmd[5] == 0x34;
	}
	cmd[6] = 0xEF;

	HAL_UART_Transmit(huart_vcc3r , (uint8_t *)cmd , 7 , 0xF);
}

void vcc3r_home(){
	uint8_t cmd[6];
	cmd[0] = 0xFF;
	cmd[1] = 0x30;
	cmd[2] = 0x30;
	cmd[3] = 0x00;
	cmd[4] = 0x57;
	cmd[5] = 0xEF;

	HAL_UART_Transmit(huart_vcc3r , (uint8_t *)cmd , 6 , 0xF);
}

void vcc3r_stop(){
	uint8_t cmd[7];
	cmd[0] = 0xFF;
	cmd[1] = 0x30;
	cmd[2] = 0x30;
	cmd[3] = 0x00;
	cmd[4] = 0x53;
	cmd[5] = 0x30;
	cmd[6] = 0xEF;

	HAL_UART_Transmit(huart_vcc3r , (uint8_t *)cmd , 7 , 0xF);
}

void vcc3r_zoom(uint8_t command){
	uint8_t cmd[7];
	cmd[0] = 0xFF;
	cmd[1] = 0x30;
	cmd[2] = 0x30;
	cmd[3] = 0x00;
	cmd[4] = 0xA2;
	if(command == 1){
		cmd[5] = 0x31;
	}else if(command == 2){
		cmd[5] = 0x32;
	}
	cmd[6] = 0xEF;

	HAL_UART_Transmit(huart_vcc3r , (uint8_t *)cmd , 7 , 0xF);
}
