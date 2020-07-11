/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: 3 May 2020
 *      Author: aksha
 */
#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"



#define MY_ADDR		0x61 //NOTE: In I2C manual there are some reserved addresses which you cannot use.
#define SLAVE_ADDR	0x68 //Arduino address
I2C_Handle_t I2C1Handle;
uint8_t some_data[] = "Sunday\n"; //NOTE:Arduino 'wire' library has a limit of 32bytes in single I2C transmission



void delay (void)
{
	for (uint32_t i = 0; i<500000/2;i++);
}

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = AF4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//Scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//Sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}


void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;

	I2C1Handle.I2C_Config_t.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config_t.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C1Handle.I2C_Config_t.I2C_FMDutyCycle = I2C_FM_DUTY_2;//Not really used as we are working in SM mode
	I2C1Handle.I2C_Config_t.I2C_DeviceAddress = MY_ADDR;//Not really used since we are working in master mode

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit()
{
	GPIO_Handle_t GPIOUSR;	//USR button which is connected to PA0

	memset(&GPIOUSR, 0, sizeof(GPIOUSR));

	GPIOUSR.pGPIOx = GPIOA;
	GPIOUSR.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOUSR.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOUSR.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;
	GPIOUSR.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&GPIOUSR);
}

int main()
{


	GPIO_ButtonInit();//USR Button init
	I2C1_GPIOInits();//GPIO Pins init
	I2C1_Inits();//I2C1 Peripheral init

	I2C_PeripheralControl(I2C1, ENABLE);//I2C Peripheral enable

	while(1)
	{
		//Wait for button press
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();	//to avoid button de-bouncing issues add some delay

		//Send some data
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*) some_data),SLAVE_ADDR, DISABLE);

	}


	return 0;
}

