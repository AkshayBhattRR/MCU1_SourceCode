/*
 * 015usart_tx.c
 *
 *  Created on: 21 Jun 2020
 *      Author: aksha
 */
#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"

//extern void initialise_monitor_handles();

char msg[1024] = "UART Tx testing..\n";

USART_Handle_t usart2_handle;

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USARTx_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USARTx_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USARTx_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USARTx_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USARTx_Config.USART_ParityControl = USART_PARITY_DISABLE;
	usart2_handle.USARTx_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_pins;

	usart_pins.pGPIOx = GPIOA;
	usart_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_pins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;


	//USART2 TX
	usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usart_pins);

	//USART2 RX
	usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_pins);
}



void GPIO_ButtonInit()
{
	GPIO_Handle_t GPIOUSR;	//USR button which is connected to PA0

	GPIOUSR.pGPIOx = GPIOA;
	GPIOUSR.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOUSR.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOUSR.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;
	GPIOUSR.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&GPIOUSR);
}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main()
{
	//initialise_monitor_handles();
	printf("Application is running\n");


	GPIO_ButtonInit();//USR Button init
	USART2_GPIOInit();//GPIO pin init for USART2 Peripheral
	USART2_Init();//USART2 Peripheral init
	USART_PeripheralControl(USART2, ENABLE);

	while(1)
	{
		//wait till button is depressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//To avoid button de-bouncing
		delay();

		// Send Data
		USART_SendData(&usart2_handle, (uint8_t*) msg, strlen(msg));
	}

	return 0;

}

