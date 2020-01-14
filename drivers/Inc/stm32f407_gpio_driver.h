/*
 * stm32f407_gpio_driver.h
 *
 *  Created on: Dec 10, 2019
 *      Author: Oguz CAN
 */
#ifndef INC_STM32F407_GPIO_DRIVER_H_
#define INC_STM32F407_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/**
 * GPIO pin configuration structure
 */
typedef struct {
	uint8_t GPIO_PinNumber;				//See @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;				//See @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;				//See @GPIO_SPEED
	uint8_t GPIO_PinPuPdControl;		//See @GPIO_PUPD
	uint8_t GPIO_PinOPType;				//See @PIN_OUTPUT_MODES
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/**
 * Handle structure for a GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;				//Pointer to hold the base address of GPIO peripheral
	GPIO_PinConfig_t GPIO_PinConfig;	//GPIO pin configuration settings
}GPIO_Handle_t;

/*************************************************************GPIOx CONFIGURATION MACROS****************************************************************/

/**
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15


/**
 * @GPIO_PIN_MODES
 * Possible GPIO pin modes
 */
#define GPIO_MODE_IN 	 	0	//GPIO input mode
#define GPIO_MODE_OUT	 	1	//GPIO output mode
#define GPIO_MODE_ALTFN  	2	//GPIO alternate function mode
#define GPIO_MODE_ANALOG 	3	//GPIO analog mode
#define GPIO_MODE_IT_FT		4	//GPIO falling edge interrupt mode
#define GPIO_MODE_IT_RT		5	//GPIO rising edge interrupt mode
#define GPIO_MODE_IT_RFT	6	//GPIO rising and falling edge interrupt mode


/**
 * @PIN_OUTPUT_MODES
 * Possible GPIO output modes
 */
#define GPIO_OP_TYPE_PP		0	//GPIO output type push pull
#define GPIO_OP_TYPE_OD		1	//GPIO output type open drain

/**
 * @GPIO_SPEED
 * Possible GPIO output speeds
 * Refer to the vendor manual for actual speed multipliers
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


/**
 * @GPIO_PUPD
 * GPIO pin pull-up & pull-down configuration macros
 */
#define GPIO_NO_PUPD		0	//No pull-up/pull-down
#define GPIO_PIN_PU			1	//Pull-up
#define GPIO_PIN_PD			2	//Pull-down

/***********************************************************************APIs*************************************************************************/

/**
 * GPIO peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/**
 * GPIO initialize and de-initialize
 */
void GPIO_Init 		(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit 	(GPIO_RegDef_t *pGPIOx);

/**
 * Data read and write
 */
uint8_t	 GPIO_ReadFromInputPin	(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort	(GPIO_RegDef_t *pGPIOx);
void	 GPIO_WriteToOutputPin	(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void 	 GPIO_WriteToOutputPort	(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void 	 GPIO_ToggleOutputPin	(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig	(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQHandling			(uint8_t PinNumber);
void GPIO_IRQPriorityConfig 	(uint8_t IRQNumber, uint8_t IRQPriority);

#endif /* INC_STM32F407_GPIO_DRIVER_H_ */
