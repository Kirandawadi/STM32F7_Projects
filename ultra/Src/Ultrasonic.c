#include "Ultrasonic.h"

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
