
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "stm32f7xx_hal.h"              // Keil::Device:STM32Cube HAL:Common


void led_Thread1 (void const *argument);
void led_Thread2 (void const *argument);
void Create(void);
osThreadDef(led_Thread1, osPriorityAboveNormal, 1, 0);			//note the raised priority for led_thread 1
osThreadDef(led_Thread2, osPriorityNormal, 1, 0);

osThreadId T_ledOn;
osThreadId T_ledOff;
/*----------------------------------------------------------------------------
  Define the semaphore
 *---------------------------------------------------------------------------*/	
osSemaphoreId sem1;									
osSemaphoreDef(sem1);
/*----------------------------------------------------------------------------
  Wait to acquire a semaphore token from sem1 then flash LED 1
 *---------------------------------------------------------------------------*/
void led_Thread1 (void const *argument) 
{
	for (;;) 
	{
		osSemaphoreWait(sem1, osWaitForever);
		HAL_GPIO_WritePin(GPIOI,GPIO_PIN_1,GPIO_PIN_SET);                  
		osDelay(500);
		HAL_GPIO_WritePin(GPIOI,GPIO_PIN_1,GPIO_PIN_RESET);  
	}
}
/*----------------------------------------------------------------------------
  Flash LED 2 and 'release' a semaphore token to sem1
 *---------------------------------------------------------------------------*/
void led_Thread2 (void const *argument) 
{
	for (;;) 
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);		
		osSemaphoreRelease(sem1);
		osDelay(500);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);	
		osDelay(500);
	}
}

void Create (void) 
{
	sem1 = osSemaphoreCreate(osSemaphore(sem1), 0);	
	T_ledOff = osThreadCreate(osThread(led_Thread2), NULL);
	T_ledOn = osThreadCreate(osThread(led_Thread1), NULL);
}

