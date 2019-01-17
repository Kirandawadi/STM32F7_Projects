#include "cmsis_os.h"
#include "stm32f7xx_hal.h"              // Keil::Device:STM32Cube HAL:Common

void  multiplex_Thread (void const *argument);

osThreadDef(multiplex_Thread, osPriorityNormal, 6, 0);
osThreadId T_mux1;																			
osThreadId T_mux2;
osThreadId T_mux3;
osThreadId T_mux4;
osThreadId T_mux5;
osThreadId T_mux6;

osSemaphoreId semMultiplex;						//declare the Semaphore
osSemaphoreDef(semMultiplex);
osSemaphoreId semSignal;
osSemaphoreDef(semSignal);

void Led_On(uint32_t argument);
void Led_Off(uint32_t argument);

/*******************************************************************
Create a thread and use the Semaphore to control the number of threads 
running simultaneously
********************************************************************/

void multiplex_Thread (void const *argument) 
{
	for (;;) 
	{
    osSemaphoreWait(semMultiplex,osWaitForever);
		Led_On((uint32_t)argument);
		osDelay(300);
		Led_Off((uint32_t)argument);
		osDelay(300);
		osSemaphoreRelease(semMultiplex);
	}
}

/*----------------------------------------------------------------------------
 Create the Semaphore and launch the threads
 *---------------------------------------------------------------------------*/
void Create (void) 
{              
	semMultiplex = osSemaphoreCreate(osSemaphore(semMultiplex), 1);	
	T_mux1 = osThreadCreate(osThread(multiplex_Thread),(void *)1UL);
	T_mux2 = osThreadCreate(osThread(multiplex_Thread),(void *)2UL);                  
}

void Led_On(uint32_t argument)
{
	if(argument == 1)
		HAL_GPIO_WritePin(GPIOI,GPIO_PIN_1,GPIO_PIN_SET);
	else if(argument == 2)
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
}

void Led_Off(uint32_t argument)
{
		if(argument == 1)
		HAL_GPIO_WritePin(GPIOI,GPIO_PIN_1,GPIO_PIN_RESET);
	else if(argument == 2)
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
}
	

