#include "Ultrasonic.h"
#include "dwt_stm32_delay.h"

static void Trigger(int usec)
{
	/************Generate a 10us pulse on Trig Pin***************************/
		HAL_GPIO_WritePin(Pin_Ports,Trig_Pin_No,GPIO_PIN_SET);
		DWT_Delay_us(usec);
		HAL_GPIO_WritePin(Pin_Ports,Trig_Pin_No,GPIO_PIN_RESET);
		/**************************************************************************/
}

double Get_Ultrasonic_Distance(void)
{
	Trigger(10);
	
	while((HAL_GPIO_ReadPin(Pin_Ports,Echo_Pin_No))!=GPIO_PIN_SET);
		time = 0;
		
			while((HAL_GPIO_ReadPin(Pin_Ports,Echo_Pin_No))!=GPIO_PIN_RESET)
			{
				time++;
			}
			backup = time;
			Distance = (double)backup/115;
			
			return Distance;
}


	