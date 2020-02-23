/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: Dec 10, 2019
 *      Author: Oguz CAN
 */

#include "stm32f407_gpio_driver.h"

/***************************************************************************
 * @fn					- GPIO_BASEADDR_TO_CODE
 *
 * @brief				- Returns a code for every GPIO port to be used in GPIO_Init () when
 * 						  interrupt mode is selected
 *
 * @param[in]			- GPIO structure that holds relevant information about pins
 *
 * @return				- Decoded integer from peripheral address determining the port
 * 						  which a pin is belongs to - 0 for Port A, 1 for port B etc.
 *
 * @Note				- none
 */
uint8_t GPIO_BASEADDR_TO_CODE(GPIO_Handle_t *pGPIOHandle) {

	if (pGPIOHandle->pGPIOx == GPIOA) {
		return 0;
	} else if (pGPIOHandle->pGPIOx == GPIOB) {
		return 1;
	} else if (pGPIOHandle->pGPIOx == GPIOC) {
		return 2;
	} else if (pGPIOHandle->pGPIOx == GPIOD) {
		return 3;
	} else if (pGPIOHandle->pGPIOx == GPIOE) {
		return 4;
	} else if (pGPIOHandle->pGPIOx == GPIOF) {
		return 5;
	} else if (pGPIOHandle->pGPIOx == GPIOG) {
		return 6;
	} else if (pGPIOHandle->pGPIOx == GPIOH) {
		return 7;
	} else if (pGPIOHandle->pGPIOx == GPIOI) {
		return 8;
	} else {
		return 255;
	}
}


/***************************************************************************
 * @fn					- GPIO_PeriClockControl
 *
 * @brief				- Enables or disables clock for a given GPIO port
 *
 * @param[in]			- GPIOx base address
 * @param[in]			- "ENABLE" or "DISABLE" the clock for the port
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi){
	if (EnOrDi) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	}

	else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}
}

/***************************************************************************
 * @fn					- GPIO_Init
 *
 * @brief				- Configures pins'
 * 							1- Mode
 * 							2- Speed
 * 							3- Pull-up/Pull-down settings
 * 							4- Output type
 * 							5- Alternate functionality if selected
 *
 * @param[in]			- GPIO structure that holds relevant information about pins
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_Init (GPIO_Handle_t *pGPIOHandle){
	//Enable port clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//////////////////////////////////////1- Configure pin mode////////////////////////////////////////////////////////
	uint32_t temp = 0; //temporary register space to configure
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));					 //Clearing bits of interest by bitwise "and" with b00 (inverted from b11)
		pGPIOHandle->pGPIOx->MODER |= temp; 																		 //Setting bits of interest by bitwise "or"

	}

	else {
			//TO DO: Complete interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			/*1 Configure FTSR (Falling trigger selection register)*/
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Set falling edge register
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Reset rising edge register
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			/*1 Configure RTSR (Falling trigger selection register)*/
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Set rising edge register
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Reset falling edge register

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			/*1 Configure both RTSR and FTSR*/
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Set rising edge register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Set falling edge register

		}

		/*2 Configure the GPIO port selection in SYSCFG_EXTICR*/
		uint8_t temp1, temp2;		//variables to hold which EXTICR register to manipulate and which bits in that register
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;		//Find which EXTICR register to manipulate
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;		//Find which bits in the register to manipulate
		uint8_t portcode;
		portcode = GPIO_BASEADDR_TO_CODE (pGPIOHandle);
		//uint8_t portcode = GPIO_BASEADDR_TO_CODE (pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();											//Enable peripheral clock before configuring it's registers
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);
		//SYSCFG->EXTICR[temp1] |= (portcode << (temp2 * 4));			//Multiply temp2 with 4 since each exti configuration takes 4-bits

		/*3 Enable the exti interrupt delivery using IMR*/
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}




	temp = 0;//Set the temporary register to 0 so as not to configure unwanted bits next time we use it

	///////////////////////////////////////2- Configure pin speed//////////////////////////////////////////////////////
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//////////////////////////////////3- Configure pull-up/pull-down///////////////////////////////////////////////////
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/////////////////////////////////////4- Configure output type///////////////////////////////////////////////////////
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	///////////////////////////////////5- Configure alternate functionality if selected////////////////////////////////
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;					//This will return 0 if pin number is smaller than 8 and return 1 if it is higher
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;					//With this we can find where in the register we should modify
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}


/***************************************************************************
 * @fn					- GPIO_DeInit
 *
 * @brief				- Resets pin configurations in a GPIO port
 *
 * @param[in]			- GPIO structure that holds relevant information about pins
 *
 * @return				- none
 *
 * @Note				- Refer to RCC AHB1 peripheral reset register for GPIOs (RCC_AHB1RSTR)
 */
void GPIO_DeInit 	(GPIO_RegDef_t *pGPIOx){
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}


/**
 * Data read and write
 */
/***************************************************************************
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				- Reads from desired pin
 *
 * @param[in]			- GPIO structure that holds relevant information about pins
 * @param[in]			- GPIO pin number
 *
 * @return				- 0 or 1
 *
 * @Note				- none
 */
uint8_t	 GPIO_ReadFromInputPin	(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}



/***************************************************************************
 * @fn					- GPIO_ReadFromInputPort
 *
 * @brief				- Reads from desired port
 *
 * @param[in]			- GPIO structure that holds relevant information about pins
 *
 * @return				- All pin values from the port in 16-bit unsigned integer
 *
 * @Note				- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}


/***************************************************************************
 * @fn					- GPIO_WriteToOutputPin
 *
 * @brief				- Writes to desired pin
 *
 * @param[in]			- GPIO structure that holds relevant information about pins
 * @param[in]			- GPIO pin number
 * @param[in]			- Value to be written to the pin
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_WriteToOutputPin	(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/***************************************************************************
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				- Writes to desired port
 *
 * @param[in]			- GPIO structure that holds relevant information about pins
 * @param[in]			- Value to be written to the port
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_WriteToOutputPort	(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}


/***************************************************************************
 * @fn					- GPIO_ToggleOutputPin
 *
 * @brief				- Toggles the pin output
 *
 * @param[in]			- GPIO structure that holds relevant information about pins
 * @param[in]			- Pin number
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_ToggleOutputPin	(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}



/***************************************************************************
 * @fn					- GPIO_IRQInterruptConfig
 *
 * @brief				- Enables or disables the relevant interrupt
 *
 * @param[in]			- IRQ number to be configured
 * @param[in]			- Enables or disables the interrupt
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_IRQInterruptConfig		(uint8_t IRQNumber, uint8_t EnOrDi){
	//TO DO: Create NVIC structure, tidy this
	if (EnOrDi) {
		if (IRQNumber <= 31) {								//Configure ISER0 - 0, 31
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {		//Configure ISER1 - 32, 63
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {		//Configure ISER2 - 64, 95
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	} else {
		if (IRQNumber <= 31) {								//Configure ICER0 - 0, 31
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {		//Configure ICER1 - 32, 63
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		} else if (IRQNumber >= 64 && IRQNumber < 96) {		//Configure ICER2 - 64, 95
			*NVIC_ICER2 |= (1 << IRQNumber % 32);
		}
	}
}


/***************************************************************************
 * @fn					- GPIO_IRQPriorityConfig
 *
 * @brief				- Configures interrupt priority
 *
 * @param[in]			- IRQ number to be configured
 *
 * @return				- none
 *
 * @Note				- In ARM Cortex M(0, 0+, 3, 4) interrupt priority bits are implemented
 * 						  in the most-significant bits of the priority configuration registers
 * 						  in the NVIC. In this case (STM32F407VGT6U (STM32F4DISCOVERY board))
 * 						  only 4 bits are implemented and least significant 4 bits are irrelevant.
 * 						  !!Trying to write in these don't care bits as well as implemented bits
 * 						  at the same time will cause bugs in the code!!
 * @Note				- Interrupt priorities will default to 0 after reset. Interrupts can share
 * 						  priority numbers.
 */
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint8_t IRQPriority){
	//Find correct IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


/***************************************************************************
 * @fn					- GPIO_IRQHandling
 *
 * @brief				- Clears interrupt pending so that program can return
 * 						  to normal operation or interrupt in interest can
 * 						  trigger again
 *
 * @param[in]			- Interrupt enabled pin
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_IRQHandling (uint8_t PinNumber) {
	//Clear EXTI PR register of the pin
	if (EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1<< PinNumber);
	}
}
