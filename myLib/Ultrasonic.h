/**
  ******************************************************************************
  * @file    Ultrasonic.h
  * @author  Kiran Dawadi
  * @brief   STM32Fxx Devices Compatible SR-04 Ultrasonic Library
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral’s registers hardware
  *
  ******************************************************************************
	**/
	#include "stm32f7xx.h"
	#include "stm32f7xx_hal.h"

	/*************Private Variables****************/
uint32_t time, backup;
double Distance;
double Obstacle;
#define Trig_Pin_No                ((uint16_t)0x0040U)  /* Pin 6 selected    */		/*Get Definitions from stm32f7xx_hal_gpio.h*/
#define Echo_Pin_No                ((uint16_t)0x0080U)  /* Pin 7 selected    */
#define Pin_Ports      ((GPIO_TypeDef *) GPIOF_BASE)
//#define Echo_Pin_Port 			((GPIO_TypeDef *) GPIOF_BASE);
//#define Trig_high() HAL_GPIO_WritePin(GPIOF,Trig,GPIO_PIN_SET);
//#define Trig_low() HAL_GPIO_WritePin(GPIOF,Trig,GPIO_PIN_RESET);
GPIO_PinState state;
	
	/*********************Functions***************************/
static void Trigger(int usec);
double Get_Ultrasonic_Distance(void);

