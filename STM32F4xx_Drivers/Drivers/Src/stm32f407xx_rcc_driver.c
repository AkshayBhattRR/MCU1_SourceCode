/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: 21 Jun 2020
 *      Author: aksha
 */

#include "stm32f407xx_rcc_driver.h"
uint16_t AHB_PreScaler[] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[] = {2,4,8,16};

/**************************************************************
 * @fn				- RCC_GetPCLK1Value
 *
 * @brief			- This function calculates APB1 clock frequency
 *
 * @param[in]		- none
 *
 * @return			- Frequency of peripheral bus for APB1
 *
 * @note			- Refer to clock tree for frequency calculation
 */

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1;
	uint32_t SystemClk;
	uint8_t clksrc, temp, ahb_ps, apb1_ps;

	clksrc =(RCC->CFGR >> 2)&(0x3);

	if (clksrc == 0)
	{
		//HSI
		SystemClk = 16000000; //16MHz
	} else if (clksrc == 1)
	{
		//HSE Oscillator
		SystemClk = 8000000; //16MHz
	} else if (clksrc == 2)
	{
		//PLL used as system clock. (NA in this course)
	}

	// Determination of AHB prescaler
	temp = (RCC->CFGR >> 4) & (0xF);
	if (temp < 8)
	{
		ahb_ps = 1;
	} else
	{
		ahb_ps = AHB_PreScaler[temp-8];
	}

	//Determination of APB1 prescaler
	temp = (RCC->CFGR >> 10) & (0x7);
	if (temp < 4)
	{
		apb1_ps = 1;
	}else
	{
		apb1_ps = APB_PreScaler[temp-4];
	}

	pclk1 = (SystemClk/ahb_ps)/apb1_ps;

	return pclk1;

}

/**************************************************************
 * @fn				- RCC_GetPCLK2Value
 *
 * @brief			- This function calculates APB2 clock frequency
 *
 * @param[in]		- none
 *
 * @return			- Frequency of APB2
 *
 * @note			- Refer to clock tree for frequency calculation
 */

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2;
	uint32_t SystemClk;
	uint8_t clksrc, temp, ahb_ps, apb2_ps;

	clksrc =(RCC->CFGR >> 2)&(0x3);

	if (clksrc == 0)
	{
		//HSI
		SystemClk = 16000000; //16MHz
	} else if (clksrc == 1)
	{
		//HSE Oscillator
		SystemClk = 8000000; //16MHz
	} else if (clksrc == 2)
	{
		//PLL used as system clock. (NA in this course)
	}

	// Determination of AHB prescaler
	temp = (RCC->CFGR >> 4) & (0xF);
	if (temp < 8)
	{
		ahb_ps = 1;
	} else
	{
		ahb_ps = AHB_PreScaler[temp-8];
	}

	//Determination of APB2 prescaler
	temp = (RCC->CFGR >> 13) & (0x7);
	if (temp < 4)
	{
		apb2_ps = 1;
	}else
	{
		apb2_ps = APB_PreScaler[temp-4];
	}

	pclk2 = (SystemClk/ahb_ps)/apb2_ps;

	return pclk2;

}
