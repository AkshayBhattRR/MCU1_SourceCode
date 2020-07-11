/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: 6 Jun 2020
 *      Author: aksha
 */

#include "stm32f407xx_usart_driver.h"

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - Sets the USART_BRR register for USART peripheral to set the requried baudrate
 *
 * @param[in]         - Reg Def for peripheral (USART)
 * @param[in]         - Required baudrate
 *
 * @return            - None
 *
 * @Note              - The API sets the Mantissa and Fraction part of the BRR register

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8 (equation has been multiplied by 100 to convert to whole number)
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16 (equation has been multiplied by 100 to convert to whole number)
	  usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}
/////////////////////////////Peripheral Clock setup////////////////////////////////////
/**************************************************************
 * @fn				- USART_PeriClockControl
 *
 * @brief			- This function enables or disables clock for the given USART/UART port
 *
 * @param[in]		- Base address of the USART/UART peripheral
 * @param[in]		- ENABLE or DISBALE generic macros
 *
 * @return			- none
 *
 * @note			- For generic macros refer to MCU specific header file
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		if (pUSARTx == USART1) {
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_EN();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_EN();
		} else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		} else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		} else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}

	} else {
		if (pUSARTx == USART1) {
			USART1_PCLK_DI();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_DI();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_DI();
		} else if (pUSARTx == UART4) {
			UART4_PCLK_DI();
		} else if (pUSARTx == UART5) {
			UART5_PCLK_DI();
		} else if (pUSARTx == USART6) {
			USART6_PCLK_DI();
		}
	}
}

////////////////////////////////////Init and DeInit///////////////////////////////////////
/**************************************************************
 * @fn				- USART_Init
 *
 * @brief			- This function initialises the selected USART/UART peripheral with parameters defined in USART_Config_t(which lies within USART_Handle_t)
 *
 * @param[in]		- Base address of USART_Handle_t
 *
 * @return			- none
 *
 * @note			- none
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	//Temporary variable
	uint32_t tempreg = 0;

	//Implement the code to enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	/******************************** Configuration of CR1******************************************/

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (pUSARTHandle->USARTx_Config.USART_Mode == USART_MODE_ONLY_RX) {
		//Implement the code to enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	} else if (pUSARTHandle->USARTx_Config.USART_Mode == USART_MODE_ONLY_TX) {
		//Implement the code to enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);

	} else if (pUSARTHandle->USARTx_Config.USART_Mode == USART_MODE_TXRX) {
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

	//Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USARTx_Config.USART_WordLength << USART_CR1_M;

	//Configuration of parity control bit fields
	if (pUSARTHandle->USARTx_Config.USART_ParityControl == USART_PARITY_EN_EVEN) {
		//Implement the code to enable the parity control
		tempreg |= (1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	} else if (pUSARTHandle->USARTx_Config.USART_ParityControl
			== USART_PARITY_EN_ODD) {
		//Implement the code to enable the parity control
		tempreg |= (1 << USART_CR1_PCE);

		//Implement the code to enable ODD parity
		tempreg |= (1 << USART_CR1_PS);

	}

	//Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

	tempreg = 0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USARTx_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/

	tempreg = 0;

	//Configuration of USART hardware flow control
	if (pUSARTHandle->USARTx_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
		//Implement the code to enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);

	} else if (pUSARTHandle->USARTx_Config.USART_HWFlowControl
			== USART_HW_FLOW_CTRL_RTS) {
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	} else if (pUSARTHandle->USARTx_Config.USART_HWFlowControl
			== USART_HW_FLOW_CTRL_CTS_RTS) {
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USARTx_Config.USART_Baud);
}

void USART_DeInit(USART_Handle_t *pUSARTHandle);

//////////////////////////////////////Send and Receive data/////////////////////////////
/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         - Peripheral base address
 * @param[in]         - Base address of Tx buffer
 * @param[in]         - Buffer length
 *
 * @return            - none
 *
 * @Note              - Blocking API

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USARTx_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         - Peripheral base address
 * @param[in]         - Base address of Rx buffer
 * @param[in]         - Buffer length
 *
 * @return            - none
 *
 * @Note              - Blocking API

 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	//Loop over until "Len" number of bytes are transferred
	for (uint32_t i = 0; i < Len; i++) {
		//Implement the code to wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if (pUSARTHandle->USARTx_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if (pUSARTHandle->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;

			} else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);

				//Increment the pRxBuffer
				pRxBuffer++;
			}
		} else {
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if (pUSARTHandle->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data
				//read 8 bits from DR
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);
			}

			else {
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = pUSARTHandle->pUSARTx->DR & ((uint8_t) 0X7F);
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - Used to send data using USART related interrupts
 *
 * @param[in]         - USART Handle
 * @param[in]         - Pointer to Tx buffer
 * @param[in]         - Length (bytes) of message to be transmitted
 *
 * @return            - Tx state
 *
 * @Note              - None

 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;
	if(txstate!= USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Implement code to enable interrupt for Transmit Data Register empty (TXE bit)
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//Implement code to enable interrupt for Transmission Complete (TC)
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}
	return txstate;
}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - Used to receive data using USART related interrupts
 *
 * @param[in]         - USART Handle
 * @param[in]         - Pointer to Rx buffer
 * @param[in]         - Length (bytes) of message to be received
 *
 * @return            - Rx state
 *
 * @Note              - None

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate!= USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}


/////////////////////////////////////////////////IRQ Config. and ISR Handling/////////////////////////////////
/**************************************************************
 * @fn				- USART_IRQInterruptConfig
 *
 * @brief			-
 *
 * @param[in]		- IRQ number (USART related)
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @note			- For IRQ numbers, refer to macros in device specific header file
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		if (IRQNumber < 32) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
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
 * @fn				- USART_IRQPriorityConfig
 *
 * @brief			-
 *
 * @param[in]		- IRQ number (USART related)
 * @param[in]		- Priority (between 0 and 15)
 *
 * @return			- none
 *
 * @note			- For priority numbers, refer to macros in device specific header file
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - PR_BITS_NOT_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1, temp2, temp3;
	uint16_t* pdata;
	/*****************Check for TC FLAG********************/
	//Check if TC bit is set
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);

	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if(temp1 && temp2)
	{
		//Close transmission and call application callback if TxLen is 0

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			if(!pUSARTHandle->TxLen)
			{
				//The flag is cleared by writing 0 to the bit
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;


				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);

			}
		}
	}

	/***********************Check for TXE Flag*******************/
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if (temp1 && temp2)
	{
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data untill TxLen is 0
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART Wordlength for 8 bit or 9 bit, in one frame
				if(pUSARTHandle->USARTx_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//9 bit wordlength: load the DR with 2 bytes but after masking all except the first 9 bits
					pdata = (uint16_t*)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x1FF);

					//check for parity enabled
					if(pUSARTHandle->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity, therefore 9 bits will be transmitted. therefore remove 2 8 bit buffers
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -=2;

					}
					else // Parity bit is used so actually 8 bits will only be sent as the 9th bit will be replaced by the parity bit

					{
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen --;
					}

				}else // 8 bit frame
				{
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen --;

				}
			}
			if (pUSARTHandle->TxLen == 0)
			{
				//Implement the code to clear the TXEIE bit (disable the interrupt for TXE flag)
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}

		}
	}

	/*****************Check for RXNE flag*******************************/
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if (temp1 && temp2)
	{
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				//CHECK IF WE ARE GOING TO RECEIVE 9 BITS OF DATA OR 8 BITS
				if(pUSARTHandle->USARTx_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//9 bits Rx enabled
					//Check the parity control
					if(pUSARTHandle->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//9 bits will be received
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -=2;

					}

					else//parity is used
					{
						*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR &(uint8_t)0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -=1;
					}

				}
				else //8 bits data will be received
				{
					//check if parity control is used
					if(pUSARTHandle->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity so 8 bits data expected
						*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR &(uint8_t)0xFF);
					}
					else //parity is used so 7 bits data expected
					{
						*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR &(uint8_t)0x7F);
					}

					//Now increment the pointer to buffer and decrement the length
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen -=1;
				}
			}
			if(!pUSARTHandle->RxLen)
			{
				//Disable THE rxne interrupt
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);

			}
		}
	}

	/*****************************CHeck for CTS Flag**********************/
	//NOTE: CTS feature is not applicable for USART4 and USART5 peripherals

	temp1 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_SR_CTS);
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);
	temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if (temp1 && temp2 && temp3)
	{
		//Clear the flag by writting 0 to it
		pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_CTS);

		//Tell the application that the interrupt was generated because of this flag
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

	/****************************Check for IDLE flag*********************/
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if(temp1 && temp2)
	{
		//As per the RM, the flag is cleared by a read to SR (which we did just now, see above lines, and followed by a read to the DR register WHICH WE ARE NOT DOING HERE?)
		temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

		//Tell the application
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	/********************************Check for Overrun detection flag*****************/
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2)
	{
		//Tell the App which will then clear the ORE FLag
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

	/*********************************Check for Error Flag********************************/
	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	//We dont discuss multibuffer communication in this course. please refer to the RM
	//The below code will get executed in only if multibuffer mode is used.

		temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

		if(temp2)
		{
			temp1 = pUSARTHandle->pUSARTx->SR;
			if(temp1 & ( 1 << USART_SR_FE))
			{
				/*
					This bit is set by hardware when a de-synchronization, excessive noise or a break character
					is detected. It is cleared by a software sequence (an read to the USART_SR register
					followed by a read to the USART_DR register).
				*/
				USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
			}

			if(temp1 & ( 1 << USART_SR_NF))
			{
				/*
					This bit is set by hardware when noise is detected on a received frame. It is cleared by a
					software sequence (an read to the USART_SR register followed by a read to the
					USART_DR register).
				*/
				USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
			}

			if(temp1 & ( 1 << USART_SR_ORE) )
			{
				USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
			}
		}


}

///////////////////////Other peripheral control APIs////////////////////////////////
/**************************************************************
 * @fn				- USART_PeripheralControl
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
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	} else {
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/**************************************************************
 * @fn				- USART_GetFlagStatus
 *
 * @brief			-
 *
 * @param[in]		- Base address of USART peripheral
 * @param[in]		- FlagName
 *
 * @return			- none
 *
 * @note			- none
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if (pUSARTx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**************************************************************
 * @fn				- USART_ClearFlag
 *
 * @brief			-
 *
 * @param[in]		- Base address of USART peripheral
 * @param[in]		- FlagName
 *
 * @return			- none
 *
 * @note			- none
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~(StatusFlagName);
}

/*********************************************************************
 * @fn      		  - USART_ApplicationEventCallback
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{

}

