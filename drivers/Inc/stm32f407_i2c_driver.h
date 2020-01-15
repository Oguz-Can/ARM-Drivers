/*
 * stm32f407_i2c_driver.h
 *
 *  Created on: Jan 6, 2020
 *      Author: Oguz CAN
 */

#ifndef INC_STM32F407_I2C_DRIVER_H_
#define INC_STM32F407_I2C_DRIVER_H_

#include "stm32f407xx.h"
/**
 * I2C Configuration structure
 */
typedef struct {
	uint32_t I2C_SCLSpeed;		//Communication speed see @I2C_SCLSpeed
	uint8_t  I2C_DeviceAddress;	//Defined by user
	uint8_t  I2C_ACKControl;	//Enable or disable acknowledge bit see @I2C_AckControl
	uint16_t I2C_FMDutyCycle;	//In fast mode duty cycle can be configured see @I2C_FMDutyCycle
} I2C_Config_t;


/**
 * I2C Handle structure
 */
typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
} I2C_Handle_t;


/**
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_FM2K	200000


/**
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE	ENABLE
#define I2C_ACK_DISABLE	DISABLE

/**
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1


/**
 * I2C_SR1 bit position definitions
 */
#define I2C_SR1_SB 			0
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
 * I2C_SR1 flag definitions
 */
#define I2C_FLAG_SB 		(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10		(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RxNE		(1 << I2C_SR1_RxNE)
#define I2C_FLAG_TxE		(1 << I2C_SR1_TxE)
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF			(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR		(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR		(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT	(1 << I2C_SR1_SMBALERT)

/**
 * I2C peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t * pI2Cx, uint8_t EnOrDi);

/**
 * I2C flag function
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/**
 * I2C initialize and de-initialize
 */
void I2C_Init 		(I2C_Handle_t *pI2CHandle);
void I2C_DeInit 	(I2C_RegDef_t *pI2Cx);

/**
 * I2C enable/disable
 */
void I2C_PeripheralControl (I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/**
 * Data send and receive
 */
void I2C_MasterSendData (I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
/**
 * Interrupt based data send and receive
 */
uint8_t I2C_SendDataIT		(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t I2C_ReceiveDataIT 	(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len);

/**
 * IRQ configuration and ISR handling
 */
void I2C_IRQInterruptConfig			(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig 			(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_ApplicationEventCallback 	(I2C_Handle_t *pHandle, uint8_t AppEvent);

#endif /* INC_STM32F407_I2C_DRIVER_H_ */
