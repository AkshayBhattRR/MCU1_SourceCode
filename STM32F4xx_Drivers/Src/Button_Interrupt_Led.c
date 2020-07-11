/*
 * main.c
 *
 *  Created on: Apr 13, 2020
 *      Author: aksha
 */
#include <string.h>
#include "stm32f407xx.h"

void delay (void)
{
	for (uint32_t i = 0; i<500000/2;i++);
}

int main()
{
	/* ------------------Button interrupt setting--------------*/
	GPIO_Handle_t GPIOUSR; //Button PD6
	memset(&GPIOUSR,0,sizeof(GPIOUSR));

	GPIOUSR.pGPIOx = GPIOD;
	GPIOUSR.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIOUSR.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOUSR.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIOUSR.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOUSR);


	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI10);

	/* ------------------Led setting--------------*/
	GPIO_Handle_t GPIOled1; //LED PA8
	memset(&GPIOled1,0,sizeof(GPIOled1));

	GPIOled1.pGPIOx = GPIOA;
	GPIOled1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIOled1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOled1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOled1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOled1);

	while (1) {
		// Wait for button press
	}
	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay();
	//handle the interrupt
	GPIO_IRQHandling(GPIO_PIN_NO_6);// provide the pin number
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
}
