#include "NRF_config.h"

HAL_StatusTypeDef check = HAL_ERROR;

void Send_Command(uint8_t reg,uint8_t data)
{
	reg=W_REGISTER+reg;
	DWT_Delay_us(1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
	DWT_Delay_us(1);
	HAL_SPI_Transmit(&hspi5,&reg,sizeof(reg),5);
	DWT_Delay_us(1);
	HAL_SPI_Transmit(&hspi5,&data,sizeof(data),5);
	DWT_Delay_us(1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
}

uint8_t *Write_TO_Nrf(uint8_t Read_Write,uint8_t reg,uint8_t *val,uint8_t ant_val)
{
	
	if (Read_Write ==W)
	{
		reg=W_REGISTER + reg;
	}
	static uint8_t ret[32];

	DWT_Delay_us(1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
	DWT_Delay_us(1);
	check = HAL_SPI_Transmit(&hspi5,&reg,sizeof(reg),5);
	if(check == HAL_OK)
		HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_1);
	DWT_Delay_us(1);
	
	int i;
	for (i=0;i<ant_val;i++)
	{
		if (Read_Write==R && reg!=W_TX_PAYLOAD)
		{
			check = HAL_SPI_Transmit(&hspi5,&ret[i],sizeof(ret[i]),5);
				if(check == HAL_OK)
					HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_1);
			DWT_Delay_us(1);
		}
		else
		{
			check = HAL_SPI_Transmit(&hspi5,&val[i],sizeof(val[i]),5);
				if(check == HAL_OK)
					HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_1);
			DWT_Delay_us(1);
		}
	}
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
	return ret;
}


void NRF_Init(void)
{
	DWT_Delay_us(100);
	uint8_t val[5];
	//works only if the address of the sender is equal to the sender
	val[0]=0x01;
	Write_TO_Nrf(W,EN_AA,val,1);
	
	//no of retries 
	val[0]=0x2F;
	Write_TO_Nrf(W,SETUP_RETR,val,1);
	
	
	val[0]=0x01;
	Write_TO_Nrf(W,EN_RXADDR,val,1);
	
	
	val[0]=0x03;
	Write_TO_Nrf(W,SETUP_AW,val,1);
//RF channel setup  choose frequency 
	val[0]=0x01;
	Write_TO_Nrf(W,RF_CH,val,1);
	//RF setup choose power mode and data speed
	val[0]=0x07;
	Write_TO_Nrf(W,RF_SETUP,val,1);
	
	//RF address byte 
	int i;
	for (i=0;i<5;i++)
	{
		val[i]=0x12;
	}
	Write_TO_Nrf(W,RX_ADDR_P0,val,5);
	
	//TX address byte 
	
	for (i=0;i<5;i++)
	{
		val[i]=0x12;
	}
	Write_TO_Nrf(W,TX_ADDR,val,5);
	
	//payload width setup 
	val[0]=5;
	Write_TO_Nrf(W,RX_PW_P0,val,1);
	
	//config register setup 
	val[0]=0x1E;
	Write_TO_Nrf(W,CONFIG,val,1);
	
	DWT_Delay_us(100);

}
void Transmit_Data(uint8_t *W_Buff) 
{
	Write_TO_Nrf(R,FLUSH_TX,W_Buff,0);//sends 0xE1 to flush the registry from old data 
	Write_TO_Nrf(R,W_TX_PAYLOAD,W_Buff,5);//sends the data in W_Buff to the NRF

	DWT_Delay_us(10);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);//CE setting high==transmit 
	
	DWT_Delay_us(1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);//CE settig high==transmit 
	
	DWT_Delay_us(10);

}

//GPIOC 12 is CE		GPIOC 7
//GPIOC 11 is CSN		GPIOC 6

