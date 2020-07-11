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
	GPIOled1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIOled1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOled1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOled1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_Handle_t GPIOled2;

	GPIOled2.pGPIOx = GPIOC;
	GPIOled2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIOled2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOled2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOled2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_Handle_t GPIOled3;

	GPIOled3.pGPIOx = GPIOA;
	GPIOled3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIOled3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled3.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOled3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOled3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_Handle_t GPIOled4;

	GPIOled4.pGPIOx = GPIOC;
	GPIOled4.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIOled4.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled4.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOled4.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOled4.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIOled1);
	GPIO_Init(&GPIOled2);
	GPIO_Init(&GPIOled3);
	GPIO_Init(&GPIOled4);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_4);
		delay();
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_11);
		delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_9);
		delay();
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_7);
	}


	while(1);
	return 0;
}

