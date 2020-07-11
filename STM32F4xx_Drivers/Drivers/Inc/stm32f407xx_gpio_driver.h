/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Apr 10, 2020
 *      Author: aksha
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_
#include"stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;			/* <Possible values from @Possible Pin Numbers>*/
	uint8_t GPIO_PinMode;			/* <Possible values from @Possible GPIO modes>*/
	uint8_t GPIO_PinSpeed;			/* <Possible values from @Possible GPIO output speeds>*/
	uint8_t GPIO_PinPuPdControl;	/* <Possible values from @Possible GPIO PUPD config>*/
	uint8_t GPIO_PinOPType;			/* <Possible values from @Possible GPIO output types>*/
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx; //This holds the base address of GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; //This holds the GPIO pin configuration
}GPIO_Handle_t;

/*@Possible Pin Numbers
 *
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*@Possible GPIO modes
 *
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define	GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*@Possible GPIO output types
 *
 */
#define	GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*@Possible GPIO output speeds
 *
 */
#define GPIO_SPEED_LOW		0
#define	GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*@Possible GPIO PUPD config
 *
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*@Possible ALT FUN modes
 *
 */
#define AF0					0
#define AF1					1
#define AF4					4 //I2C
#define AF5					5 //SPI
/***************************APIS supported by this driver**************************/
/*Peripheral clock set-up*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*Init and De-Init*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeTnit(GPIO_RegDef_t *pGPIOx);

/*Date read and write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*IRQ Configuration and ISR Handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);





#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
