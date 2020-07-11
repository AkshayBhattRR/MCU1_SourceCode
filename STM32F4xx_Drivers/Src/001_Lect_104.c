/*
 * 001_Lect_104.c
 *
 *  Created on: Apr 12, 2020
 *      Author: aksha
 */

#include"stm32f407xx.h"

void delay (void)
{
	for (uint32_t i = 0; i<500000/2;i++);
}

int main(void)
{
	GPIO_Handle_t GPIOled1; //LED PD12

	GPIOled1.pGPIOx = GPIOD;
	GPIOled1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOled1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOled1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOled1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_Handle_t GPIOUSR;//USR button which is connected to PA0

	GPIOUSR.pGPIOx = GPIOA;
	GPIOUSR.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOUSR.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOUSR.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOUSR.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;



	//GPIO_PeriClockControl(GPIOD, ENABLE); later included in the Init function itself
	//GPIO_PeriClockControl(GPIOA, ENABLE);


	GPIO_Init(&GPIOled1);
	GPIO_Init(&GPIOUSR);



	while(1)
	{
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}

		//delay();
	}

	return 0;
}

