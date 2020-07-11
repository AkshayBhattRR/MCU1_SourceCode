/*
 * 013i2c_slave_tx_string.c
 *
 *  Created on: 17 May 2020
 *      Author: aksha
 */

#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"

//extern void initialise_monitor_handles();


#define SLAVE_ADDR	0x69
#define MY_ADDR		SLAVE_ADDR

I2C_Handle_t I2C1Handle;
uint8_t Tx_buffer[32] = "Hello from the other side...";

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
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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
	I2C1Handle.I2C_Config_t.I2C_DeviceAddress = MY_ADDR;//Slave Address

	I2C_Init(&I2C1Handle);
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


int main()
{
	//uint8_t commandcode;
	//uint8_t len;

	//initialise_monitor_handles();
	printf("Application is running\n");


	GPIO_ButtonInit();//USR Button init
	I2C1_GPIOInits();//GPIO Pins init
	I2C1_Inits();//I2C1 Peripheral init

	//I2C IRQ Configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);//I2C Peripheral enable + ACK Enable

	while(1);

}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	static uint8_t commandCode = 0;
	static uint8_t Cnt = 0;

	if (AppEv == I2C_EV_DATA_REQ)
	{
		//Master wants some data. Slave to send this
		if(commandCode == 0x51)//Send word length
		{
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buffer));
		}else if(commandCode == 0x52)
		{
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buffer[Cnt++]);
		}
	}else if(AppEv == I2C_EV_DATA_RCV)
	{
		//Master has send some data. Slave to read it
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}else if (AppEv == I2C_ERROR_AF)
	{
		//Slave Tx has completed
		//Master must have sent NACK thats why AF waas triggered by the Slave
		commandCode = 0xFF; //Resetting command Code for next Rx
		Cnt = 0;

	}else if (AppEv == I2C_EV_STOP)
	{
		//Slave data reception is completed
		//Master has ended I2C communication with Slave
		//Slave to do nothing
		//In driver.c, SR1 cleared + CR1 written so Slave device's STOPF has been reset
	}
}


