/*
 * spi_tx_testing.c
 *
 *  Created on: Apr 15, 2020
 *      Author: aksha
 */

/*
 * PB14 -->SPI2_MISO
 * PB15 -->SPI2_MOSI
 * PB13 -->SPI2_SCLK
 * PB12 -->SPI2_NSS
 * ALT FUNC MODE -->5
 */
#include <string.h>
#include "stm32f407xx.h"
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//NSS
	//SPIPins->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}

int main(void)
{
	SPI2_GPIOInits();// initialising GPIO pins to behave as SPI2
	SPI2_Inits();//initialising SPI peripheral parameters
	SPI_SSIConfig(SPI2, ENABLE);//this makes NSS signal pulled to high so that when SSI peripheral is enabled (with SSM=1) this will not generate MODF fault
	SPI_PeripheralControl(SPI2, ENABLE);//enables SPI2 peripheral

	char user_data[] = "Hello World!";

	SPI_SendData(SPI2, (uint8_t*)user_data, (uint32_t)strlen(user_data));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
