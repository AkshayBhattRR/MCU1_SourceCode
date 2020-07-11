/*
 * 001ledtoggle.c
 *
 *  Created on: Apr 12, 2020
 *      Author: aksha
 */

#include"stm32f407xx.h"

void delay (void)
{
	for (uint32_t i = 0; i<500000;i++);
}

int main(void)
{
	GPIO_Handle_t GPIOled1;

	GPIOled1.pGPIOx = GPIOD;
	GPIOled1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOled1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOled1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOled1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	

	GPIO_PeriClockControl(GPIOD, ENABLE);


	GPIO_Init(&GPIOled1);


	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}


	while(1);
	return 0;
}

