/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 14 Apr 2020
 *      Author: aksha
 */

#include "stm32f407xx_spi_driver.h"

/*
 * @ Prototype for 'helper' functions (these exist only within this file and should not be accessible by the main program.
 */
static void spi_txe_interrupt_handle(SPI_Handle_t* pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t* pHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t* pHandle);

/**************************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			- This function enables or disables clock for the given SPI port
 *
 * @param[in]		- base address of the SPI peripheral
 * @param[in]		- ENABLE or DISBALE generic macros
 *
 * @return			- none
 *
 * @note			- For generic macros refer to MCU specific header file
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
 {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}

/**************************************************************
 * @fn				- SPI_Init
 *
 * @brief			- This function initialises the selected SPI peripheral with parameters defined in SPI_Config(hich lies within SPI_Handle_t)
 *
 * @param[in]		- Base address of SPI_Handle_t
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//enabling the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//Configuring the SPI CR1 register first

	uint32_t tempreg = 0;

	//1. Configuring the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode <<2 ;

	//2. Configuring bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << 15);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1<<15);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << 15);
		//RXONLY mode must be set
		tempreg |= (1 << 10);
	}

	//3. Configuring the BAUD rate
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3 ;

	//4. Configuring DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11 ;

	//5. Configuring Clock Polarity
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	//6. Configuring Clock phase
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. Configuring Software Slave Management
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << 9;

	pSPIHandle->pSPIx->CR1 = tempreg;


}

/**************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			- This function resets the registers of selected SPI peripheral
 *
 * @param[in]		- Base address of SPI peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 */


void SPI_DeTnit(SPI_RegDef_t *pSPIx)


 {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	}

}

/**************************************************************
 * @fn				- SPI_GetFlagStatus
 *
 * @brief			-
 *
 * @param[in]		- Base address of SPI peripheral
 * @param[in]		- FlagName
 *
 * @return			- none
 *
 * @note			- none
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			- This function transmits the data stored in TxBuffer
 *
 * @param[in]		- Base address of SPI peripheral
 * @param[in]		- Base address of TxBuffer (Data to be transmitted)
 * @param[in]		- number of bytes to be transmitted
 *
 * @return			- none
 *
 * @note			- This is a blocking call or polling type API function
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. wait until TXE is set
		//One way of doing this - while(!(pSPIx->SR & (0x1 << 1)));
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit data format
			//1. load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			//2. decrease the length
			Len --;
			Len --;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit data format
			pSPIx->DR = *(pTxBuffer);
			Len --;
			pTxBuffer++;
		}
	}

}

/**************************************************************
 * @fn				- SPI_SendDataIT
 *
 * @brief			- Only operational during an interrupt
 *
 * @param[in]		- Base address of SPI Handle
 * @param[in]		- Base address of TxBuffer (Data to be transmitted)
 * @param[in]		- number of bytes to be transmitted
 *
 * @return			- none
 *
 * @note			-
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if (state!= SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer and Len information in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as Busy in transmission so that no other code can access the current SPI peripheral
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3.Enable the TXIE bit to allow interrupt generation
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}

	return state;
}



/*SPI Receive Data*/
/**************************************************************
 * @fn				- SPI_ReceiveData
 *
 * @brief			- This function retrieves the data into RxBuffer
 *
 * @param[in]		- Base address of SPI peripheral
 * @param[in]		- Base address of RxBuffer
 *
 * @return			- none
 *
 * @note			- none
 */


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0) {
		//1. wait until RXNE register is full (set)
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))  //Not sure about this?
				{
			// 16 bit data format
			//1. Read the data from the DR
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			//2. decrease the length
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		} else {
			//8 bit data format
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}

}

/**************************************************************
 * @fn				- SPI_ReceiveDataIT
 *
 * @brief			- Only operational during an interrupt
 *
 * @param[in]		- Base address of SPI Handle
 * @param[in]		- Base address of RxBuffer (Data to be transmitted)
 * @param[in]		- number of bytes to be transmitted
 *
 * @return			- none
 *
 * @note			-
 */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_RX)
	{
		//1. Save the Rx buffer and Len information in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as Busy in transmission so that no other code can access the current SPI peripheral
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3.Enable the TXIE bit to allow interrupt generation
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	}

	return state;
}

/**************************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			- Sets or resets SPE register in CR1
 *
 * @param[in]		- Base address of SPI peripheral
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @note			- none
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi )
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/**************************************************************
 * @fn				- SPI_SSIConfig
 *
 * @brief			-
 *
 * @param[in]		- Base address of SPI peripheral
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @note			- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}

/**************************************************************
 * @fn				- SPI_SSOEConfig
 *
 * @brief			-
 *
 * @param[in]		- Base address of SPI peripheral
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @note			- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
			pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		}else
		{
			pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}
}
/**************************************************************
 * @fn				- SPI_IRQInterruptConfig
 *
 * @brief			-
 *
 * @param[in]		- IRQ number (SPI related)
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @note			- For IRQ numbers, refer to macros in device specific header file
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		if (IRQNumber < 32) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber <64) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber > 63 && IRQNumber < 96) {
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else {
		if (IRQNumber < 32) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber > 63 && IRQNumber < 96) {
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}

	}

}
/**************************************************************
 * @fn				- SPI_IRQPriorityConfig
 *
 * @brief			-
 *
 * @param[in]		- IRQ number (SPI related)
 * @param[in]		- Priority (between 0 and 16)
 *
 * @return			- none
 *
 * @note			- For priority numbers, refer to macros in device specific header file
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - PR_BITS_NOT_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);

}
/**************************************************************
 * @fn				- SPI_SSOEConfig
 *
 * @brief			-
 *
 * @param[in]		- Base address of SPI peripheral
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @note			- none
 */
void SPI_IRQHandling(SPI_Handle_t* pHandle)
{
	uint8_t temp1, temp2;
	//Check if its a TXE related interrupt
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1  && temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}

	//Check if its a RXE related interrupt
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);
	}

	//Check if its OVR flag related interrupt
	temp1 = pHandle->pSPIx->SR & (1<< SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		spi_ovr_interrupt_handle(pHandle);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t* pHandle)
{
	if (pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16 bit data format
		//1. load the data into the DR
		pHandle->pSPIx->DR = *((uint16_t*) pHandle->pTxBuffer);
		//2. decrease the length
		pHandle->TxLen--;
		pHandle->TxLen--;
		(uint16_t*) pHandle->pTxBuffer++;
	} else {
		//8 bit data format
		pHandle->pSPIx->DR = *(pHandle->pTxBuffer);
		pHandle->TxLen--;
		pHandle->pTxBuffer++;
	}

	for (uint32_t i = 0; i<500000/2;i++);//delay function ~200ms

	if(!pHandle->TxLen)
	{//This means TxLen = 0 and therefore a close out of transmission related registers needs to be done now
		SPI_CloseTransmisson(pHandle);
		//Inform the application that the interrupt handling is now over
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX_CMPLT);

	}

}
static void spi_rxne_interrupt_handle(SPI_Handle_t* pHandle)
{
	if (pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16 bit data format
		//1. read the data off the DR
		*((uint16_t*) pHandle->pRxBuffer) = pHandle->pSPIx->DR;
		//2. decrease the length
		pHandle->RxLen--;
		pHandle->RxLen--;
		//3. Increment the pointer by 2 bytes
		(uint16_t*) pHandle->pRxBuffer++;

	} else {
		//8 bit data format
		*(pHandle->pRxBuffer) = pHandle->pSPIx->DR;
		pHandle->RxLen--;
		pHandle->pRxBuffer++;
	}

	if (!pHandle->RxLen) {//This means RxLen = 0 and therefore a close out of rx related registers needs to be done now
		SPI_CloseReception(pHandle);
		//Inform the application that the interrupt handling is now over
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);

	}
}
static void spi_ovr_interrupt_handle(SPI_Handle_t* pHandle)
{
	SPI_ClearOVRFlag(pHandle->pSPIx);

	//2. Let the application know about the state
	SPI_ApplicationEventCallback(pHandle, SPI_EVENT_OVR_ERR );

}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint32_t temp;//////////////////////////Tutor kept this as uint8_t?????
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//The user application is expected to over ride this
}
