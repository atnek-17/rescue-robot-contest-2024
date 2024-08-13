/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SO1602A.h"
#include "mpu6500.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool status_1 = 0;				//toggle switch
bool status_2 = 0;
bool status_3 = 0;
bool limit_1 = 0;				//arm dasu
bool limit_2 = 0;				//arm simau
bool limit_3 = 0;				//arm up
bool SW_1 = 0;					//takuto switch
bool SW_2 = 0;
bool SW_3 = 0;
bool SW_4 = 0;
bool RST  = 0;					//reset signal


#define Servo_low 25			//MG996R , flash hobby , ds3218
#define Servo_high 125			//0.5ms - 2.5ms
uint16_t speed = 0;
uint16_t normal = 750;
uint16_t B_dash = 999;
uint16_t test_speed = 0;
//int enc_counter_1 = 0;
//int enc_counter_2 = 0;

#define servo1_init 190					//arm right open close
#define servo2_init 90					//arm left open close
#define servo3_init 130					//arm right up down
#define servo4_init 130					//arm left up down
#define servo6_init 180					//camera arm rotation
#define servo7_init 60					//camera arm up down
//#define servo8_init 60

uint16_t servo1_kakudo = servo1_init * 4;		//initialize kakudo
uint16_t servo2_kakudo = servo2_init * 4;
uint16_t servo3_kakudo = servo3_init * 4;
uint16_t servo4_kakudo = servo4_init * 4;
uint16_t servo6_kakudo = servo6_init * 4;
uint16_t servo7_kakudo = servo7_init * 4;
//uint16_t servo8_kakudo = servo8_init * 4;
//uint16_t servo5_kakudo = 90;


char str[10];
char str_1[5];
char str_2[5];
char str_3[5];
char rxbuf[1];


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_4){
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4) == 0){
			status_3 = 1;
		}else {
			status_3 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_5){
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5) == 0){
			status_2 = 1;
		}else{
			status_2 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_2){
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) == 0){
			status_1 = 1;
		}else {
			status_1 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_1){
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_1) == 1){
			limit_1 = 1;					//NO
		}else{
			limit_1 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_15){
		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_15) == 1){
			limit_2 = 1;					//NO
		}else{
			limit_2 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_0){
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_0) == 1){
			limit_3 = 0;					// NC
		}else{
			limit_3 = 1;
		}
	}else if(GPIO_Pin == GPIO_PIN_3){
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_3) == 1){
			SW_1 = 1;
		}else{
			SW_1 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_6){
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6) == 1){
			SW_2 = 1;
		}else{
			SW_2 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_7){
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_7) == 1){
			SW_3 = 1;
		}else{
			SW_3 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_8){
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_8) == 1){
			SW_4 = 1;
		}else{
			SW_4 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_10){
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10) == 1){
			RST = 1;
		}else{
			RST = 0;
		}
	}
}

void motor_4(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,ina);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,inb);
}

void motor_3(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,inb);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,ina);
}

void motor_2(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,ina);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,inb);
}

void motor_sol(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,inb);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,ina);
}

void motor_1(uint16_t ina , uint16_t inb){
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,ina);
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,inb);
}
long map(float x , float in_min , float in_max , float out_min , float out_max)
{
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void servo_1(uint16_t pulse){				//kyujo arm
	long pulse_1 = map(pulse , 0 , 270 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,pulse_1);
}
void servo_2(uint16_t pulse){
	long pulse_1 = map(pulse , 0 , 270 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,pulse_1);
}
void servo_3(uint16_t pulse){				//kyujo arm
	long pulse_1 = map(pulse , 0 , 270 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pulse_1);
}
void servo_4(float pulse){					//kyujo arm
	long pulse_1 = map(pulse , 0 , 270 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pulse_1);
}
void servo_5(float pulse){					//kyujo arm
	long pulse_1 = map(pulse , 0 , 270 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,pulse_1);
}

void servo_6(uint16_t pulse){			//camera arm no dai  0 ~ 90
	long pulse_1 = map(pulse , 0 , 270 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,pulse_1);
}
void servo_7(uint16_t pulse){			//camera arm camera rotation
	long pulse_1 = map(pulse , 0 , 270 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,pulse_1);
}
/*
void servo_8(uint16_t pulse){
	long pulse_1 = map(pulse , 0 , 270 , Servo_low , Servo_high);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pulse_1);
}
*/

void servo_init(){
	  servo_1(servo1_init);
	  servo_3(servo2_init);
	  servo_4(servo3_init);
	  servo_5(servo4_init);
	  //servo_8(servo8_init);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM4_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Receive_DMA(&huart1,(uint8_t *)rxbuf,1);			//raspi uart
  HAL_UART_Receive_DMA(&huart2,(uint8_t *)rxbuf,1);			//TPIP  uart

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);		//servo
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);		//dc
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
/*
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
*/
  //HAL_TIM_Base_Start_IT(&htim6);

  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,SET);

  oled_Init(&hi2c1);
  mpu6500_init(&hi2c2);

  OLED_Init();
  set_cursor(1,0);
  Puts("select mode");
  set_cursor(2,0);
  Puts("1.UART 2.set 3.test");
  HAL_Delay(2000);

  servo_6(servo6_init);
  servo_7(servo7_init);
  servo_init();
  //enc_counter_1 = 0;
  //enc_counter_2 = 0;
/*
  while(RST != 1 && SW_1 != 1){
	  RST 	= HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10);
	  SW_1 	= HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_3);
	  HAL_Delay(10);
  }
  */
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //number = atoi(&rxbuf[1]);
	  //sprintf(aiueo,"%d",number);
	  sprintf(str_1,"%d",limit_1);
	  sprintf(str_2,"%d",limit_2);
	  sprintf(str_3,"%d",limit_3);


	  if(status_1 == 1){
		  OLED_Init();
		  set_cursor(1,0);
		  Puts("status:uart mode");
		  set_cursor(2,0);
		  Puts("Received:");
		  set_cursor(2,10);
		  Puts(&rxbuf[0]);
		  set_cursor(2,12);			//limit switch debug
		  Puts(str_1);				//limit_1
		  set_cursor(2,13);
		  Puts(str_2);				//limit_2
		  set_cursor(2,14);
		  Puts(str_3);				//limit_3

		  if(rxbuf[0] < 95){
			  speed = B_dash;		//b_dash mode
		  }else{
			  speed = normal;		//normal speed
		  }
/*
		  if(RST){
			  HAL_NVIC_SystemReset();	//stm32 reset when communication cut off
		  }
*/
		  if(rxbuf[0] == 'k'){
			  motor_1(0,0);
			  motor_2(0,0);
			  motor_3(0,0);
			  motor_4(0,0);
		  }else if(rxbuf[0] == 'c' || rxbuf[0] == 'C'){
			  motor_1(0,speed);
			  motor_2(0,speed);		//forward
		  }else if(rxbuf[0] == 'g' || rxbuf[0] == 'G'){
			  motor_1(speed,0);
			  motor_2(speed,0);		//back
		  }else if(rxbuf[0] == 'i' || rxbuf[0] == 'I'){
			  motor_1(speed,0);
			  motor_2(0,speed);		//turn right
		  }else if(rxbuf[0] == 'j' || rxbuf[0] == 'J'){
			  motor_1(0,speed);
			  motor_2(speed,0);		//turn left
		  }else if(rxbuf[0] == 'l'){		//arm dasu
			  if(limit_1 == 0){
				  motor_3(500,0);
			  }else{
				  motor_3(0,0);
			  }
		  }else if(rxbuf[0] == 'm'){		//arm simau

			  if(limit_2 == 0){
				  motor_3(0,500);
			  }else{
				  motor_3(0,0);
			  }
		  }else if(rxbuf[0] == 'n'){	//arm sageru
			  motor_4(500,0);
		  }else if(rxbuf[0] == 'o'){	//arm ageru
			  if(limit_3 == 0){
				  motor_4(0,500);
			  }else {
				  motor_4(0,0);
			  }

		  }else if(rxbuf[0] == 'p'){
			  for(;servo1_kakudo > 760 ; servo1_kakudo--){
				  servo_1(servo1_kakudo / 4);
				  HAL_Delay(5);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }

		  }else if(rxbuf[0] == 'q'){
			  for(;servo1_kakudo < 1080 ; servo1_kakudo++){
				  servo_1(servo1_kakudo / 4);
				  HAL_Delay(5);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
		  }else if(rxbuf[0] == 'r'){
			  for(;servo2_kakudo > 0 ; servo2_kakudo--){
				  servo_3(servo2_kakudo/4);
				  HAL_Delay(2);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
		  }else if(rxbuf[0] == 's'){
			  for(;servo2_kakudo < 360 ; servo2_kakudo++){
				  servo_3(servo2_kakudo / 4);
				  HAL_Delay(2);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
	  	  }else if(rxbuf[0] == 't'){		//servo3_init : 145 * 4 = 580  10do zureru to +40do  580 + 40 = 620
			  for(;servo3_kakudo < 560 ; servo3_kakudo++){
				  servo_4(servo3_kakudo / 4);
				  servo_5((int)map(servo3_kakudo , 280 , 560 , 760 , 480) / 4);
				  HAL_Delay(5);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
	  	  }else if(rxbuf[0] == 'u'){		//80do ageru to 145do kara mainasu 80do = 65do * 4 = 260
			  for(;servo3_kakudo > 280 ; servo3_kakudo--){		//servo4_init = 110 * 4 = 440 , 30 ~ 120 , 120 ~ 480
				  servo_4(servo3_kakudo / 4);
				  servo_5((int)map(servo3_kakudo , 280 , 560 , 780 , 480) / 4);
				  HAL_Delay(5);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
		  }else if(rxbuf[0] == 'v'){
			  servo_init();
			  servo1_kakudo = servo1_init * 4;		//initialize kakudo
			  servo2_kakudo = servo2_init * 4;
			  servo3_kakudo = servo3_init * 4;
			  servo4_kakudo = servo4_init * 4;

		  }else if(rxbuf[0] == 'x'){			//arm close douji
			  for(;servo1_kakudo < 1080 ; servo1_kakudo++){
				  servo_1(servo1_kakudo / 4);
				  servo_3((int)map(servo1_kakudo , 760 , 1080 , 360 , 0) / 4);
				  HAL_Delay(6);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
		  }else if(rxbuf[0] == 'w'){			//arm open douji
			  for(;servo1_kakudo > 760 ; servo1_kakudo--){
				  servo_1(servo1_kakudo / 4);
				  servo_3((int)map(servo1_kakudo , 760 , 1089 , 360 , 0) / 4);
				  HAL_Delay(6);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
		  }else if(rxbuf[0] == 'y'){			//camera arm right rotation
			  for(;servo6_kakudo > 360 ; servo6_kakudo--){
				  servo_6(servo6_kakudo / 4);
				  HAL_Delay(6);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
		  }else if(rxbuf[0] == 'z'){			//camera arm left rotation
			  for(;servo6_kakudo < 1080 ; servo6_kakudo++){
				  servo_6(servo6_kakudo / 4);
				  HAL_Delay(6);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
		  }else if(rxbuf[0] == 'L'){			//camera arm up
			  for(;servo7_kakudo > 0 ; servo7_kakudo--){
				  servo_7(servo7_kakudo / 4);
				  HAL_Delay(6);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
		  }else if(rxbuf[0] == 'M'){			//camera arm down
			  for(;servo7_kakudo < 240 ; servo7_kakudo++){
				  servo_7(servo7_kakudo / 4);
				  HAL_Delay(6);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
		  }else if(rxbuf[0] == '1'){			//camera arm down
			  motor_sol(800,0);
		  }else if(rxbuf[0] == '2'){			//camera arm down
			  motor_sol(0,0);
		  }
  	  }else if(status_2 == 1){

		  sprintf(str_1,"%d",test_speed);

		  OLED_Init();
		  set_cursor(1,0);
		  Puts("setting speed");
		  set_cursor(2,0);
		  Puts("speed = ");
		  set_cursor(2,10);
		  Puts(str_1);

		  if(SW_3 == 1 && SW_1 == 1){
		  	  motor_3(500,0);
		  }else if(SW_3 == 1 && SW_2 == 1){
			  motor_3(0,500);
		  }else if(SW_4 == 1 && SW_1 == 1){
		  	  motor_4(500,0);
		  }else if(SW_4 == 1 && SW_2 == 1){
		  	  motor_4(0,500);
		  }else if(SW_1 == 1){
			  test_speed += 50;

		  }else if(SW_2 == 1){
			  test_speed -= 50;

		  }else if(SW_3){
			  motor_sol(800,0);
		  }else {
			  motor_3(0,0);
			  motor_4(0,0);
			  motor_sol(0,0);
		  }

	  }else if(status_3 == 1){
		  mpu6500_get_deg();
		  sprintf(str,"%d",deg_X);
		  OLED_Init();
		  set_cursor(1,0);
		  Puts("status:Gyro test");
		  set_cursor(2,0);
		  Puts("degree:");
		  set_cursor(2,9);
		  Puts(str);

		  if(SW_1 == 1){
			  OLED_Init();
			  set_cursor(1,0);
			  Puts("test:servo_2");

			  for(;servo2_kakudo < 720 ; servo2_kakudo++){
				  servo_4(servo2_kakudo / 4);
				  HAL_Delay(2);
				  if(SW_1 == 0){
					  break;
				  }
			  }
		  }else if(SW_2 == 1){
			  for(;servo2_kakudo > 0 ; servo2_kakudo--){
				  servo_4(servo2_kakudo / 4);
				  HAL_Delay(2);
				  if(SW_2 == 0){
					  break;
				  }
			  }
		  }else if(SW_3 == 1){
			  for(;servo1_kakudo < 1080 ; servo1_kakudo++){
				  servo_1(servo1_kakudo / 4);
				  HAL_Delay(2);
				  if(SW_3 == 0){
					  break;
				  }
			  }
		  }else if(SW_4 == 1){
			  for(;servo1_kakudo > 0 ; servo1_kakudo--){
				  servo_1(servo1_kakudo / 4);
				  HAL_Delay(2);
				  if(SW_4 == 0){
					  break;
				  }
			  }
		  }
	  }else if(SW_1 == 1){			//forward test
/*		  enc_counter_1 = TIM2 -> CNT;
		  enc_counter_2 = TIM8 -> CNT;
		  sprintf(str_2,"%d",enc_counter_1);
		  sprintf(str_3,"%d",enc_counter_2);
*/
		 OLED_Init();
		 set_cursor(1,0);
		 Puts("right:");
		 set_cursor(1,8);
		 Puts(str_2);
		 set_cursor(2,0);
		 Puts("left");
		 set_cursor(2,8);
		 Puts(str_3);

		 motor_1(test_speed , 0);
		 motor_2(test_speed , 0);


	  }else if(SW_2 == 1){			//back test
/*		  enc_counter_1 = TIM2 -> CNT;
		  enc_counter_2 = TIM8 -> CNT;
		  sprintf(str_2,"%d",enc_counter_1);
		  sprintf(str_3,"%d",enc_counter_2);
*/
		 OLED_Init();
		 set_cursor(1,0);
		 Puts("right:");
		 set_cursor(1,8);
		 Puts(str_2);
		 set_cursor(2,0);
		 Puts("left");
		 set_cursor(2,8);
		 Puts(str_3);

		 motor_1(0,test_speed);
		 motor_2(0,test_speed);

	  }else if(SW_3 == 1){
		  OLED_Init();
		  set_cursor(1,0);
		  Puts("status: senkai1");
		  motor_1(0,test_speed);
		  motor_2(test_speed,0);


	  }else if(SW_4 == 1){
		  OLED_Init();
		  set_cursor(1,0);
		  Puts("status: senkai2");
		  motor_1(test_speed ,0 );
		  motor_2(0 , test_speed);

	  }else if(status_1 + status_2 + status_3 == 0){
		  OLED_Init();
		  set_cursor(1,0);
		  Puts("select mode");
		  set_cursor(2,0);
		  Puts("1.UART 2.st 3.tst");

		  motor_1(0,0);
		  motor_2(0,0);
		  motor_3(0,0);
		  motor_4(0,0);
	  }

	  HAL_Delay(5);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4320-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2160-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 108-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 108-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10800-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 108-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 108-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PF12 PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG3 PG6 
                           PG7 PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
