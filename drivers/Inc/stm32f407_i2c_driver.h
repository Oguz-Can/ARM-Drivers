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
	uint8_t  I2C_ACKControl;	//Enable or disable acknowledge bit
	uint16_t I2C_FMDutyCycle;	//In fast mode duty cycle can be configured see @I2C_FMDutyCycle
} I2C_Config_t;

/**
 * I2C Handle structure
 */
typedef struct{
	I2C_RegDef_t *pI2Cx;		//Registers
	I2C_Config_t I2C_Config;	//Configurations
	uint8_t *pTxBuffer;			//Application Tx buffer address
	uint8_t *pRxBuffer;			//Application Rx buffer address
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;			//Communication state
	uint8_t DevAddr;			//Device/slave address
	uint32_t RxSize;
	uint8_t Sr;					//Repeated start value
} I2C_Handle_t;

/**
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_FM2K	200000

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
 * I2C repeated start macros
 */
#define I2C_ENABLE_SR	SET
#define I2C_DISABLE_SR	RESET

/**
 * I2C status macros
 */
#define I2C_READY		0
#define I2C_BUSY_IN_TX	1
#define I2C_BUSY_IN_RX	2

/**
 * Application call back macros
 */
#define I2C_EV_RX_CMPLT		0
#define I2C_EV_TX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_EV_DATA_REQ		3
#define I2C_EV_DATA_RCV		4
#define I2C_ERROR_BERR		5
#define I2C_ERROR_ARLO		6
#define I2C_ERROR_AF		7
#define I2C_ERROR_OVR		8
#define I2C_ERROR_TIMEOUT	9

/**
 * I2C peripheral clock setup
 */
void I2C_PeriClockControl (I2C_RegDef_t * pI2Cx, uint8_t EnOrDi);

/**
 * Acknowledgement control
 */
void I2C_AckControl (I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/**
 * I2C flag function
 */
bool I2C_GetFlagStatus (I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/**
 * I2C initialize and de-initialize
 */
void I2C_Init (I2C_Handle_t *pI2CHandle);
void I2C_DeInit (I2C_RegDef_t *pI2Cx);

/**
 * I2C enable/disable
 */
void I2C_PeripheralControl (I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_GenerateStop(I2C_RegDef_t *pI2Cx);

/**
 * Data send and receive
 */
void I2C_MasterSendData (I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, bool Sr);
void I2C_MasterReceiveData (I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, bool Sr);

void I2C_SlaveSendData (I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData (I2C_RegDef_t *pI2Cx);

/**
 * Interrupt based data send and receive
 */
uint8_t I2C_MasterSendDataIT (I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, bool Sr);
uint8_t I2C_MasterReceiveDataIT (I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, bool Sr);

/**
 * IRQ configuration and ISR handling
 */
void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig (uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_ApplicationEventCallback (I2C_Handle_t *pI2CHandle, uint8_t AppEvent); //Should be implemented in application
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_SlaveCallbackEventControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/**
 * Disable I2C interrupts
 */
void I2C_CloseSendData (I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

#endif /* INC_STM32F407_I2C_DRIVER_H_ */
