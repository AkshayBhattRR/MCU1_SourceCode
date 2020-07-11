/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: 21 Jun 2020
 *      Author: aksha
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_
#include "stm32f407xx.h"

/*
 * Peripheral 1 clock frequency calculation
 */
uint32_t RCC_GetPCLK1Value(void);

/*
 * Peripheral 2 clock frequency calculation
 */
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
