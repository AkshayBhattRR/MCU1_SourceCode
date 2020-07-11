/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 26 Apr 2020
 *      Author: aksha
 */
//Q - In I2C_ClearADDRFlag, ADDR Flag is not cleared when RxLen > 1
#include "stm32f407xx_i2c_driver.h"

//Prototypes for internal functions (for this file only)
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2C);

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2C, uint8_t Slave_Addr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2C, uint8_t Slave_Addr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

//Internal functions
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2C)
{
	pI2C->CR1 |= (1 << I2C_CR1_START);
};

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2C)
{

	pI2C->CR1 |= (1 << I2C_CR1_STOP);
};

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2C, uint8_t Slave_Addr)
{
	Slave_Addr = Slave_Addr << 1;
	Slave_Addr &= ~(1);
	pI2C->DR = Slave_Addr;
};

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2C, uint8_t Slave_Addr)
{
	Slave_Addr = Slave_Addr << 1;
	Slave_Addr |= 0x1;
	pI2C->DR = Slave_Addr;
};


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//Check its master
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//Check its in Rx
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//Check RxLen is 1
			if(pI2CHandle->RxSize == 1)
			{
				//Disable ACK first and then Clear ADDR Flag
				I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void) dummy_read;
			}

		}else
		{
			// BUSY in Tx then directly clear the ADDR Flag
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void) dummy_read;
		}
	}else
	{
		//In slave mode directly clear the ADDR Flag
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void) dummy_read;
	}



};


/**************************************************************
 * @fn				- I2C_PeriClockControl
 *
 * @brief			- This function enables or disables clock for the given I2C port
 *
 * @param[in]		- base address of the I2C peripheral
 * @param[in]		- ENABLE or DISBALE generic macros
 *
 * @return			- none
 *
 * @note			- For generic macros refer to MCU specific header file
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

/**************************************************************
 * @fn				- I2C_AckControl
 *
 * @brief			- This function sets the ACK flag.
 *
 * @param[in]		- Base address of I2C_Handle_t
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @note			- This must be done only when the PE has been enabled
 */

void I2C_AckControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI) {
	if (ENorDI == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}
/**************************************************************
 * @fn				- I2C_Init
 *
 * @brief			- This function initialises the selected I2C peripheral with parameters defined in I2C_Config(which lies within I2C_Handle_t)
 *
 * @param[in]		- Base address of I2C_Handle_t
 *
 * @return			- none
 *
 * @note			- none
 */
 void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//1.enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//2. ACK bit set
	//tempreg |= (pI2CHandle->I2C_Config_t.I2C_ACKControl) << I2C_CR1_ACK;
	//pI2CHandle->pI2Cx->CR1 |= tempreg;

	//3.FREQ bits setting
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);


	//4. Storing 7-bit (not using a 10-bit address in this tutorial) in the OAR-1
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config_t.I2C_DeviceAddress << 1;
	tempreg |= (1 <<14); //A note in ref manual says this bit 'should be always set'
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//5. CCR calculations
	tempreg = 0;
	uint16_t ccr_bits = 0;
	if (pI2CHandle->I2C_Config_t.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Standard mode (CCR = T/(2*Tpclk). Therefore CCR = Freq Pclk/(2 * Freq of I2C clck)
		ccr_bits = RCC_GetPCLK1Value()/(2*pI2CHandle->I2C_Config_t.I2C_SCLSpeed);
		tempreg |= (ccr_bits & 0xFFF);

	}else
	{//Fast Mode
		tempreg |= (1 << 15);//enabling bit for fastmode
		tempreg |= pI2CHandle->I2C_Config_t.I2C_FMDutyCycle << 14;
		if(pI2CHandle->I2C_Config_t.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_bits = RCC_GetPCLK1Value()/(3*pI2CHandle->I2C_Config_t.I2C_SCLSpeed);
		}else
		{
			ccr_bits = RCC_GetPCLK1Value()/(25*pI2CHandle->I2C_Config_t.I2C_SCLSpeed);
		}
		tempreg |= (ccr_bits & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//6. TRISE calculations
	tempreg = 0;
	if (pI2CHandle->I2C_Config_t.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{//Standard Mode
		//Trise = 1000 nanoseconds
		tempreg = (RCC_GetPCLK1Value())/1000000U;
		tempreg++;

	}
	else
	{//Fast Mode
		//Trise = 300nanoseconds
		tempreg = (RCC_GetPCLK1Value()*300)/1000000000U;
		tempreg++;
	}

	pI2CHandle->pI2Cx->TRISE = 0;
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F) ;


}

/**************************************************************
 * @fn				- I2C_DeInit
 *
 * @brief			- This function resets the registers of selected I2C peripheral
 *
 * @param[in]		- Base address of I2C peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 */
void I2C_DeTnit(I2C_RegDef_t *pI2Cx)
{
	//To do just like for GPIO driver, just need to reset the registers
}




/**************************************************************
 * @fn				- I2C_MasterSendData
 *
 * @brief			- This function sends data (including slave address) when the device is in Master and Tx mode
 *
 * @param[in]		- I2CHandle (consists of I2C RegDef and I2C Config structures)
 * @param[in]		- Pointer to Tx buffer
 * @param[in]		- Data length in bytes
 * @param[in]		- 8 bit slave address
 * @param[in]		- Repeated Start Enable or Disable
 *
 * @return			- None
 *
 * @note			- Refer to header file for various macros
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t I2C_RS_EnorDI)
{
	//1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.
	//   Confirm that the start generation has been completed by checking that the flag (SB) is set
	//Note: Until SB is cleared, the SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );


	//3. Send the address of the slave with read/write bit (8th bit) set to write(0)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that the address phase is completed by checking the ADDR flag in SR1.
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Read the SR1 and SR2 registers to clear the ADDR bit
	//Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the data until Len becomes 0
	while (Len > 0)
	{
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));//wait till TXE is set

		//uint16_t temp = (uint16_t)*pTxBuffer;
		//temp &= ~(0xFF << 8);
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;

	}

	//7. When Length becomes 0 wait for TXE=1 and BTF=1 before generating STOP condition
	//NOTE: TXE = 1 and BTF=1 means that both SR and DR are empty and next transmission should begin
	//When BTF = 1, SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not wait for the completion of stop condition.
	//NOTE: Generating STOP condition automatically clears the BTF
	if (I2C_RS_EnorDI == DISABLE)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t Len, uint8_t SlaveAddr, uint8_t I2C_RS_EnorDI) {
	//1.Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.confirm that start generation was completed
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. send the address of the slave with r/nw bit set to R(1) (8bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address is complete by checking ADDR flag in SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	if (Len == 1)

	{

		// Read a single byte from Slave

		// Disable Acking
		I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//3.wait until RXNE = 1
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//Generate STOP condition
		if (I2C_RS_EnorDI == DISABLE)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//Read data in the buffer DR
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}

	if (Len > 1)

	{
		//1. Clear ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle);

		//2. read the data until Len becomes zero
		for (uint32_t i = Len; i > 0; i--) {
			//wait until RXNE becomes 1
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if (i == 2)	//If last 2 bytes are remaining
			{
				//Disable ACKing
				I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);

				//Generate stop condition
				if (I2C_RS_EnorDI == DISABLE)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

			}

			//read the data from DR
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}
	}

	//re-enable ACKing
	if (pI2CHandle->I2C_Config_t.I2C_ACKControl == ENABLE)
	{
		I2C_AckControl(pI2CHandle->pI2Cx, ENABLE);
	}
}


/**************************************************************
 * @fn				- I2C_IRQInterruptConfig
 *
 * @brief			-
 *
 * @param[in]		- IRQ number (I2C related)
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @note			- For IRQ numbers, refer to macros in device specific header file
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn				- I2C_IRQPriorityConfig
 *
 * @brief			-
 *
 * @param[in]		- IRQ number (I2C related)
 * @param[in]		- Priority (between 0 and 15)
 *
 * @return			- none
 *
 * @note			- For priority numbers, refer to macros in device specific header file
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - PR_BITS_NOT_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);

}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxLen > 0) {
		//Write in DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//Decrement TxLen
		pI2CHandle->TxLen--;

		//Increment buffer
		pI2CHandle->pTxBuffer++;
	}
}
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//1. Size ==1
	if (pI2CHandle->RxSize == 1) {
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	//2. Size > 1
	if (pI2CHandle->RxSize > 1) {
		if (pI2CHandle->RxLen == 2)
		{
			//Disable ACKing
			I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);
		}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	//3. Special case: Len ==0
	if (pI2CHandle->RxLen == 0) {
		//Close the I2C data reception and notify the application

		//1. generate the stop condition
		if (pI2CHandle->Sr == DISABLE) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}
		//2. Close the rx
		I2C_CloseReceiveData(pI2CHandle);
		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Disable the interrupts
	pI2CHandle->pI2Cx->CR2 = ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 = ~(1 << I2C_CR2_ITEVTEN);

	//Clear out the handle
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;

	//Enable the ACK
	if(pI2CHandle->I2C_Config_t.I2C_ACKControl == ENABLE)
	{
		I2C_AckControl(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Disable the interrupts
	pI2CHandle->pI2Cx->CR2 = ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 = ~(1 << I2C_CR2_ITEVTEN);

	//Clear out the handle
	pI2CHandle->TxLen = 0;
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;


	//Donot need to touch the ACK as you do not disable the ACK in Tx mode
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;

}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;

}
/**************************************************************
 * @fn				- I2C_EV_IRQHandling
 *
 * @brief			- This function is the first layer of IRQ handling and it finds out the cause for I2C event triggered IRQ
 * 						and then performs the necessary action
 *
 * @param[in]		- Base address for I2C Handle
 *
 * @return			- none
 *
 * @note			- none
 */

void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle)
{
	//Interrupt handling for both master and slave mode

	uint32_t temp1,temp2,temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	//Check for SB triggered interrupt
	//Note: This flag is only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	if (temp1 && temp3)
	{
		//In this block execute the address phase (with read or write)
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxState ==I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}

	//Check for ADDR triggered interrupt
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if (temp1 && temp3)
	{
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//Check for STOPF triggered interrupt
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if (temp1 && temp3)
	{
		//Only applicable for slave mode. For master this will never be set
		//To clear it : 1) Read SR1 (this step already done above) 2) write in CR1
		//For CR1 writing, just do an OR operation with 0x0000
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application about that STOP from master is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}

	//Check for BTF triggered interrupt
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)//Make sure its Tx related event
		{
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))//Double sure the BTF bit set happened after TXE set
			{
				if (pI2CHandle->TxLen == 0)
				{
					if (pI2CHandle->Sr == DISABLE)
					{
						//Generate Stop
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//Reset all members elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					//Notify the application that the tx is completed
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
	} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
	{
		//Do nothing
	}


	//Check for RxNE triggered interrupt
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if (temp1 && temp2 && temp3)
	{
		//Check that device is in Master mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//Check device in Rx mode
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				/*Do data reception now*/
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else //Slave mode
		{
			if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))//Checking that the device is in Rx mode
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}

	}

	//Check for TxE	triggered interrupt
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if (temp1 && temp2 && temp3)
	{
		//Master mode
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//We should do data transmission
			//but only if App state is busy in tx
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				//Transfer the data now
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}

		}else//Slave mode
		{
			if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))//Checking that the device is in Tx mode
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}


		}

	}

}

/**************************************************************
 * @fn				- I2C_ER_IRQHandling
 *
 * @brief			- This function is the first layer of IRQ handling and it finds out the cause for I2C interrupt triggered IRQ
 * 						and then performs the necessary action
 *
 * @param[in]		- Base address for I2C Handle
 *
 * @return			- none
 *
 * @note			- none
 */
void I2C_ER_IRQHandling(I2C_Handle_t* pI2CHandle)
{
	uint32_t temp1, temp2;

	//know the status of ITEREN control bit in the CR2
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);

	/*************BUS ERROR**********************/
	temp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR);
	if (temp1 && temp2) {
		//Implement code to clear the bus error (by reseting the BUSERR bit in SR1)
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		//Notify the app about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	/***********ARBIRTRATION ERROR**********/
	temp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO);

	if (temp1 && temp2) {
		//Implement code to clear the bus error (by reseting the BUSERR bit in SR1)
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//Notify the app about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	/************ACK FAILURE ERROR***********/
	temp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF);

	if (temp1 && temp2) {
		//Implement code to clear the bus error (by reseting the BUSERR bit in SR1)
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//Notify the app about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	/**************OVERRUN/UNDERRUN ERROR******************/
	temp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR);

	if (temp1 && temp2) {
		//Implement code to clear the bus error (by reseting the BUSERR bit in SR1)
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//Notify the app about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	/**************TIMEOUT ERROR***************/
	temp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT);

	if (temp1 && temp2) {
		//Implement code to clear the bus error (by reseting the BUSERR bit in SR1)
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//Notify the app about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}


/**************************************************************
 * @fn				- I2C_MasterSendDataIT
 *
 * @brief			- This function carries out necessary configuration to set the peripheral ready for master Tx interupt mode
 *
 * @param[in]		- pointer to I2CHandle
 * @param[in]		- Pointer to Tx buffer
 * @param[in]		- Data length in bytes
 * @param[in]		- 8 bit slave address
 * @param[in]		- Repeated Start Enable or Disable
 *
 * @return			- Communication state
 *
 * @note			- Refer to i2c driver header file for communication states
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t I2C_RS_EnorDI)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) //Means the state is 'ready'
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = I2C_RS_EnorDI;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

/**************************************************************
 * @fn				- I2C_MasterReceiveDataIT
 *
 * @brief			- This function carries out necessary configuration to set the peripheral ready for master Rx interupt mode
 *
 * @param[in]		- Pointer to I2CHandle
 * @param[in]		- Pointer to Rx buffer
 * @param[in]		- Data length in bytes
 * @param[in]		- 8 bit slave address
 * @param[in]		- Repeated Start Enable or Disable
 *
 * @return			- Communication state
 *
 * @note			- Refer to i2c driver header file for communication states
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t I2C_RS_EnorDI)
{
	uint8_t busystate = pI2CHandle->TxRxState;

		if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) //Means the state is 'ready'
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->RxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = Len;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = I2C_RS_EnorDI;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

		}

		return busystate;
}

/**************************************************************
 * @fn				- I2C_GetFlagStatus
 *
 * @brief			-
 *
 * @param[in]		- Base address of I2C peripheral
 * @param[in]		- FlagName
 *
 * @return			- none
 *
 * @note			- none
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**************************************************************
 * @fn				- I2C_PeripheralControl
 *
 * @brief			- Sets or resets PE register in CR1
 *
 * @param[in]		- Base address of I2C peripheral
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @note			- none
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi )
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		I2C_AckControl(pI2Cx, ENABLE);//Enable ACK here as it cannot be set before PE = 1
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/**************************************************************
 * @fn				- I2C_SlaveEnableDisableCallbackEvents
 *
 * @brief			- Only for usage in Slave Mode. Sets/resets I2C Interrupt related bits
 *
 * @param[in]		- Base address of I2C peripheral
 * @param[in]		- ENABLE or DISABLE command for I2C interrupts related bits
 *
 * @return			- none
 *
 * @note			- none
 */


void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2C, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		//Implement the code to enable ITBUFEN Control Bit
		pI2C->CR2 |= (1 << I2C_CR2_ITBUFEN);
		//Implement the code to enable ITEVTEN Control Bit
		pI2C->CR2 |= (1 << I2C_CR2_ITEVTEN);
		//Implement the code to enable ITERREN Control Bit
		pI2C->CR2 |= (1 << I2C_CR2_ITERREN);
	}else
	{
		//Disable all the interrupts as above
		pI2C->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2C->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2C->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}

}

/**************************************************************
 * @fn				- I2C_ApplicationEventCallback
 *
 * @brief			- Can be over written in main application based on user's requriements
 *
 * @param[in]		- Base address of I2C Handle
 * @param[in]		- Event state
 *
 * @return			- none
 *
 * @note			- none
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	//The user application is expected to over ride this
}
