/*
 * stm32f407_SPI_driver.h
 *
 *  Created on: Dec 23, 2019
 *      Author: Oguz CAN
 */

#ifndef INC_STM32F407_SPI_DRIVER_H_
#define INC_STM32F407_SPI_DRIVER_H_

#include "stm32f407xx.h"

/**
 * SPI Configuration structure
 */
typedef struct {
	uint8_t SPI_DeviceMode;		//Master/Slave see @SPI_DeviceMode
	uint8_t SPI_BusConfig;		//Full Duplex ,Half Duplex, Simplex see @SPI_BusConfig
	uint8_t SPI_SclkSpeed;		//Bus clock prescalar, not used in I2S see @SPI_SclkSpeed
	uint8_t SPI_DFF;			//Transmission packet size see @SPI_DFF
	uint8_t SPI_CPOL;			//Clock polarization see @SPI_CPOL
	uint8_t SPI_CPHA;			//Clock phase see @SPI_CPHA
	uint8_t SPI_SSM;			//Slave select mode see @SPI_SSM
} SPI_Config_t;



/**
 * SPI Handle structure
 */
typedef struct {
	SPI_RegDef_t *pSPIx;			//Base address of SPIx
	SPI_Config_t SPIConfig;			//Definitions for SPI interrupt
	uint8_t *pTxBuffer;				//Tx buffer address defined by application
	uint8_t *pRxBuffer;				//Rx buffer address defined by application
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
} SPI_Handle_t;


/**
 * @SPI_DeviceMode macros
 */
#define SPI_DEVICE_MODE_SLAVE  0
#define SPI_DEVICE_MODE_MASTER 1

/**
 * @SPI_BusConfig macros
 */
#define SPI_BUS_CONFIG_FD		0
#define SPI_BUS_CONFIG_HD		1
#define SPI_BUS_CONFIG_S_RXONLY	2 //TX only is simply full duplex with MISO disconnected

/**
 * @SPI_SclkSpeed macros
 */
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7


/**
 * @SPI_DFF macros
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1


/**
 * @SPI_CPOL macros
 */
#define SPI_CPOL_LOW	0
#define SPI_CPOL_HIGH	1


/**
 * @SPI_CPHA macros
 */
#define SPI_CPHA_LOW	0
#define SPI_CPHA_HIGH	1


/**
 * @SPI_SSM macros
 */
#define SPI_SSM_DI		0
#define SPI_SSM_EN		1

/**
 * SPI_SR flag definitions
 */
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG		(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG		(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG		(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG		(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG		(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG		(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG		(1 << SPI_SR_FRE)


/**
 * SPI application states
 */
#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/**
 * Macros for SPI call-back function
 */
#define SPI_EVENT_TX_CMPLT	0
#define SPI_EVENT_RX_CMPLT	1
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4
#define SPI_EVENT_MODF_ERR	5
#define SPI_EVENT_FRE_ERR	6

/**
 * SPI peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t * pSPIx, uint8_t EnOrDi);

/**
 * SPI flag function
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/**
 * SPI Internal slave select control
 */
void SPI_SSIConfig (SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/**
 * SPI slave select output control
 */
void SPI_SSOEConfig (SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/**
 * SPI initialize and de-initialize
 */
void SPI_Init 		(SPI_Handle_t *pSPIHandle);
void SPI_DeInit 	(SPI_RegDef_t *pSPIx);

/**
 * SPI enable/disable
 */
void SPI_PeripheralControl (SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/**
 * Data send and receive
 */
void SPI_SendData		(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData 	(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/**
 * Interrupt based data send and receive
 */
uint8_t SPI_SendDataIT		(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT 	(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/**
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig			(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQHandling				(SPI_Handle_t *pHandle);
void SPI_IRQPriorityConfig 			(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_ApplicationEventCallback 	(SPI_Handle_t *pHandle, uint8_t AppEvent);

#endif /* INC_STM32F407_SPI_DRIVER_H_ */
