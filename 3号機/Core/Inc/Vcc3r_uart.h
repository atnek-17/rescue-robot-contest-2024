/*
 * Vcc3r_uart.h
 *
 *  Created on: 2024/03/27
 *      Author: aiueo
 */

#ifndef INC_VCC3R_UART_H_
#define INC_VCC3R_UART_H_

#include "stm32f7xx_hal.h"

UART_HandleTypeDef *huart_vcc3r;

void vcc3r_init(UART_HandleTypeDef *handler);
void vcc3r_pan_move(uint8_t command);
void vcc3r_tilt_move(uint8_t command);
void vcc3r_home();
void vcc3r_stop();
void vcc3r_zoom(uint8_t command);

#endif /* INC_VCC3R_UART_H_ */
