/*
 * 001_105.c
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
	GPIO_Handle_t GPIOUSR; //Button PB12

	GPIOUSR.pGPIOx = GPIOB;
	GPIOUSR.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOUSR.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOUSR.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIOUSR.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GPIOUSR);

	GPIO_Handle_t GPIOled1; //LED PA8

	GPIOled1.pGPIOx = GPIOA;
	GPIOled1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIOled1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOled1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOled1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST	;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOled1);

	while(1)
	{
		if ((GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12)) == 0)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}

	}

	return 0;
}
