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
	
#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include "stm32f7xx.h"
#include "dwt_stm32_delay.h"
	

	
	/*********************Functions***************************/
static void Trigger(int usec);
double Get_Ultrasonic_Distance(void);

#endif

