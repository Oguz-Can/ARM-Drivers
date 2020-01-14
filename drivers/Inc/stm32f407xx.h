/*
 * stm32f407xx.h
 *
 *  Created on: Dec 9, 2019
 *      Author: Oguz CAN
 */
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include "stddef.h"

/***************************************************PROCCESSOR SPECIFIC MACROS***********************************************************/
/**
 * NVIC ISERx register addresses
 */
#define NVIC_ISER0		((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1		((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2		((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3		((volatile uint32_t *)0xE000E10C)


/**
 * NVIC ISERx register addresses
 */
#define NVIC_ICER0			((volatile uint32_t *)0XE000E180)
#define NVIC_ICER1			((volatile uint32_t *)0XE000E184)
#define NVIC_ICER2			((volatile uint32_t *)0XE000E188)
#define NVIC_ICER3			((volatile uint32_t *)0XE000E18C)

#define NVIC_IPR_BASEADDR	((volatile uint32_t *)0XE000E400)

/**
 * Number of bits implemented in IPR register sections
 */
#define NO_PR_BITS_IMPLEMENTED	4 //STM32F407VGT6U


/******************************************************MCU SPECIFIC MACROS***************************************************************/
/**
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR 				0x08000000U			//Flash memory address
#define SRAM1_BASEADDR				0x20000000U			//SRAM1 memory address
#define SRAM2_BASEADDR				0x2001C000U			//SRAM2 memory address (starts after 112KB of SRAM1)
#define ROM							0x1FFF0000U			//System read only memory address
#define SRAM 						SRAM1_BASEADDR		//SRAM1 and SRAM2 are back to back and acts as system RAM


/**
 * Base addresses of bus interfaces
 */
#define PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x50000000U


/**
 * Base addresses of peripherals on AHB1 bus
 */
#define RCC_BASEADDR				(AHB1PERIPH_BASE + 0x3800U)

#define GPIOA_BASEADDR				 AHB1PERIPH_BASE
#define GPIOB_BASEADDR				(AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASE + 0x2000U)


/**
 * Base addresses of peripherals on APB1 bus
 */
#define I2C1_BASEADDR				(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR				(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR				(APB1PERIPH_BASE + 0x5C00U)
#define SPI2_BASEADDR				(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR				(APB1PERIPH_BASE + 0x3C00U)
#define USART2_BASEADDR				(APB1PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR				(APB1PERIPH_BASE + 0x4800U)
#define UART4_BASEADDR				(APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASEADDR				(APB1PERIPH_BASE + 0x5000U)

/**
 * Base addresses of peripherals on APB2 bus
 */
#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x3C00U)
#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x3000U)
#define SPI4_BASEADDR				(APB2PERIPH_BASE + 0x3400U)
#define SPI5_BASEADDR				(APB2PERIPH_BASE + 0x5000U)
#define SPI6_BASEADDR				(APB2PERIPH_BASE + 0x5400U)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASE + 0x3800U)
#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR				(APB2PERIPH_BASE + 0x1400U)


/**
 * IRQ (Interrupt Request) Numbers
 * TO DO: Complete for other vector table entries
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51


/**
 * NVIC priority macros
 */
#define NVIC_IRQ_PRI0		(uint8_t)0x0
#define NVIC_IRQ_PRI1		(uint8_t)0x1
#define NVIC_IRQ_PRI2		(uint8_t)0x2
#define NVIC_IRQ_PRI3		(uint8_t)0x3
#define NVIC_IRQ_PRI4		(uint8_t)0x4
#define NVIC_IRQ_PRI5		(uint8_t)0x5
#define NVIC_IRQ_PRI6		(uint8_t)0x6
#define NVIC_IRQ_PRI7		(uint8_t)0x7
#define NVIC_IRQ_PRI8		(uint8_t)0x8
#define NVIC_IRQ_PRI9		(uint8_t)0x09
#define NVIC_IRQ_PRI10		(uint8_t)0x0A
#define NVIC_IRQ_PRI11		(uint8_t)0x0B
#define NVIC_IRQ_PRI12		(uint8_t)0x0C
#define NVIC_IRQ_PRI13		(uint8_t)0x0D
#define NVIC_IRQ_PRI14		(uint8_t)0x0E
#define NVIC_IRQ_PRI15		(uint8_t)0x0F




/*********************************************PERIPHERAL REGISTER DEFINITION STRUCTURES***************************************/

/*GPIO Register Structure*/
typedef struct GPIO_RegDef_t {		//DO NOT CHANGE THE PLACE OR TYPE OF VARIABLES - YOU MAY BREAK REGISTER OFFSET POSITIONS
	volatile uint32_t MODER;		//GPIO port mode register - Input, General Purpose Output, Alternate Function, Analog
	volatile uint32_t OTYPER;		//Port output mode register - Output push-pull, Output open-drain
	volatile uint32_t OSPEEDR;		//Port output speed register - Low speed, Medium speed, High speed, Very high speed
	volatile uint32_t PUPDR;		//Port pull-up/pull-down register - No pull-up/pull-down, Pull-up, Pull-down, Reserved
	volatile uint32_t IDR;			//Port input data register - Read only and can only be accessed in word mode
	volatile uint32_t ODR;			//Port output data register - Can be read and written by software
	volatile uint32_t BSRR;			//Port bit set/reset register - Write only and read operations return 0x0000
	volatile uint32_t LCKR;			//Port configuration lock register - Each lock bit freezes a specific configuration register (cntrl & alt regs)
	volatile uint32_t AFR [2];		//Alternate function low (AFRL [0]) and high registers (AFRL [1]) - Selects between 16 alternate function options
} GPIO_RegDef_t;


/*RCC Register Structure*/
typedef struct {					//DO NOT CHANGE THE PLACE OR TYPE OF VARIABLES - YOU MAY BREAK REGISTER OFFSET POSITIONS
	volatile uint32_t CR;			//Clock control register
	volatile uint32_t PLLCFGR;		//PLL configuration register
	volatile uint32_t CFGR;			//Clock configuration register
	volatile uint32_t CIR;			//Clock interrupt register
	volatile uint32_t AHB1RSTR;		//AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;		//AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;		//AHB3 peripheral reset register
	uint32_t RESERVED0;				//RESERVED BY VENDOR
	volatile uint32_t APB1RSTR;		//APB1 peripheral reset register
	volatile uint32_t APB2RSTR;		//APB2 peripheral reset register
	uint32_t RESERVED1 [2];			//RESERVED BY VENDOR
	volatile uint32_t AHB1ENR;		//AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;		//AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;		//AHB3 peripheral clock enable register
	uint32_t RESERVED2;				//RESERVED BY VENDOR
	volatile uint32_t APB1ENR;		//APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;		//APB2 peripheral clock enable register
	uint32_t RESERVED3 [2];			//RESERVED BY VENDOR
	volatile uint32_t AHB1LPENR;	//AHB2 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;	//AHB3 peripheral clock enable in low power mode register
	volatile uint32_t AHB3LPENR;	//AHB4 peripheral clock enable in low power mode register
	uint32_t RESERVED4;				//RESERVED BY VENDOR
	volatile uint32_t APB1LPENR;	//APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;	//APB2 peripheral clock enable in low power mode register
	uint32_t RESERVED5 [2];			//RESERVED BY VENDOR
	volatile uint32_t BDCR;			//Backup domain control register
	volatile uint32_t CSR;			//Clock control & status register
	uint32_t RESERVED6 [2];			//RESERVED BY VENDOR
	volatile uint32_t SSCGR;		//Spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;	//PLLI2S configuration register
	volatile uint32_t PLLSAICFGR;	//PLL configuration register
	volatile uint32_t DCKCFGR;		//Dedicated Clock Configuration Register
} RCC_RegDef_t;


/*EXTI Register Structure*/
typedef struct {
	volatile uint32_t IMR;			//Interrupt mask register
	volatile uint32_t EMR;			//Event mask register
	volatile uint32_t RTSR;			//Rising trigger selection register
	volatile uint32_t FTSR;			//Falling trigger selection register
	volatile uint32_t SWIER;		//Software interrupt event register
	volatile uint32_t PR;			//Pending register
} EXTI_RegDef_t;

/*SYSCFG Register Structure*/
typedef struct{
	volatile uint32_t MEMRMP;		//Memory re-map register
	volatile uint32_t PMC;			//Peripheral mode configuration register
	volatile uint32_t EXTICR[4];	//External interrupt configuration register
	uint32_t RESERVED [2];			//RESERVED BY VENDOR (EXTICR4 offset = 0x14, CMPCR offset = 0x20, both are 32-bit)
	volatile uint32_t CMPCR;		//Compensation cell control register
} SYSCFG_RegDef_t;


/*SPI Register Structure*/
typedef struct {
	volatile uint32_t CR1;			//Control register 1
	volatile uint32_t CR2;			//Control register 2
	volatile uint32_t SR;			//Status register
	volatile uint32_t DR;			//Data register
	volatile uint32_t CRCPR;		//CRC polynomial register
	volatile uint32_t RXCRCR;		//Reception CRC
	volatile uint32_t TXCRCR;		//Transmission CRC
	volatile uint32_t I2SCFGR;		//Configuration register
	volatile uint32_t I2SPR;		//Prescaler register
} SPI_RegDef_t;

/*I2C Register Structure*/
typedef struct{
	volatile uint32_t CR1;			//Control register 1
	volatile uint32_t CR2;			//Control register 2
	volatile uint32_t OAR1;			//Own address register 1
	volatile uint32_t OAR2;			//Own address register 2
	volatile uint32_t DR;			//Data register
	volatile uint32_t SR1;			//Status register 1
	volatile uint32_t SR2;			//Status register 2
	volatile uint32_t CCR;			//Clock control register
	volatile uint32_t TRISE;		//Maximum rise time in Fm/Sm mode (master mode)
	volatile uint32_t FLTR;			//Analog and digital noise filter configuration
} I2C_RegDef_t;

/**
 * Peripheral definitions (Peripheral base addresses type casted to xxxx_RegDef_t)
 */
#define RCC		((RCC_RegDef_t *) RCC_BASEADDR)

#define GPIOA	((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD 	((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE 	((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF 	((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG 	((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t *) GPIOH_BASEADDR)
#define GPIOI 	((GPIO_RegDef_t *) GPIOI_BASEADDR)

#define SYSCFG	((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

#define EXTI	((EXTI_RegDef_t *) EXTI_BASEADDR)

#define SPI1	((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4	((SPI_RegDef_t *)SPI4_BASEADDR)
#define SPI5	((SPI_RegDef_t *)SPI5_BASEADDR)
#define SPI6	((SPI_RegDef_t *)SPI6_BASEADDR)

#define I2C1    ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2	((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3	((I2C_RegDef_t *)I2C3_BASEADDR)

/***************************CLOCK ENABLE MACROS*****************************/

/**
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))


/**
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))


/**
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()		(RCC->APB2ENR |= (1 << 21))

/**
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/**
 * Clock enable macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))


/***************************CLOCK DISABLE MACROS*****************************/

/**
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))


/**
 * Clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))


/**
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 21))

/**
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/**
 * Clock disable macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))


/****************************************************************************/

/**
 * Reset macros for GPIOx peripherals
 * Registers should not be kept at reset state
 */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)


/**
 * Reset macros for SPIx peripherals
 * Registers should not be kept at reset state
 */
#define SPI1_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)
#define SPI5_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~(1 << 20)); }while(0)
#define SPI6_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~(1 << 21)); }while(0)



/**
 * Reset macros for I2Cx peripherals
 * Registers should not be kept at reset state
 */
#define I2C1_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)


/*
#define GPIO_BASEADDR_TO_CODE(x)	   ((x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 :\
										(x == GPIOI) ? 8 : 0)
										*/

/****************************************************************************/

/***************************SPI bit positions********************************/
/**
 * Bit postion definitions for SPI_CR1
 */
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL 		1
#define SPI_CR1_MSTR 		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15


/**
 * Bit position definitions of SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/**
 * Bit position definitions of SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


/***************************I2C bit positions********************************/
/**
 * Bit positions for I2C_CR1
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15


/**
 * Bit positions for I2C_CR2
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_UTBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12


/**
 * Bit positions for I2C_SR1
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15


/**
 * Bit positions for I2C_SR2
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8


/**
 * Bit positions for I2C_CCR
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15



/**
 * Generic macros
 */
#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET


#endif /* INC_STM32F407XX_H_ */
