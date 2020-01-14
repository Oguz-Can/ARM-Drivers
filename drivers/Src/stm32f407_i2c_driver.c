/*
 * stm32f407_i2c_driver.c
 *
 *  Created on: Jan 7, 2020
 *      Author: Oguz CAN
 */

#include "stm32f407_i2c_driver.h"

/***************************************************************************
 * @fn					- I2C_PeriClockControl
 *
 * @brief				- Enables or disables clock for a given I2C
 *
 * @param[in]			- I2Cx base address
 * @param[in]			- "ENABLE" or "DISABLE" command for the I2Cx clock
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if (EnOrDi) {
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


//TO DO: implement PLL clock calculation
/***************************************************************************
 * @fn					- RCC_GetPLLOutputClock
 *
 * @brief				- Helper function to calculate PLL frequency
 *
 * @param[in]			- none
 *
 * @return				- PLL clock
 *
 * @Note				- none
 */
static uint32_t RCC_GetPLLOutputClock(){
	uint32_t PLLclock = 0;
	return PLLclock;
}


/***************************************************************************
 * @fn					- RCC_GetPCLK1
 *
 * @brief				- Helper function to calculate I2C clock speed
 *
 * @param[in]			- none
 *
 * @return				- PCLK1
 *
 * @Note				- none
 */
static uint32_t RCC_GetPCLK1() {
	uint32_t systemclck = 0;	//Systick
	uint32_t ahbspeed = 0;		//AHB bus speed
	uint32_t pclk1 = 0;			//APB bus speed which determines I2C peripheral clock

	/*Get systick speed*/
	uint8_t clcksrc = (RCC->CFGR >> 2) & 0x3; //Bring clock source bits to lsb position and mask

	if (clcksrc == 0) { 						//HSI clock
		systemclck = 16000000;					//stm32f407 HSI is 16 MHz
	} else if (clcksrc == 1) {					//HSE clock
		systemclck = 8000000;					//Discovery board has a 8 MHz crystal
	} else if (clcksrc == 2) {					//PLL clock
		systemclck = RCC_GetPLLOutputClock();	//Calculate PLL output
	} else {
		systemclck = 0;
	}

	/*Get AHB bus speed*/
	uint8_t AHBpresc = (RCC->CFGR >> 4) & 0xF; //Bring clock prescaler bits to lsb position and mask

	if (AHBpresc < 8) {
		ahbspeed = systemclck;
	} else if (AHBpresc == 8) {
		ahbspeed = systemclck / 2;
	} else if (AHBpresc == 9) {
		ahbspeed = systemclck / 4;
	} else if (AHBpresc == 10) {
		ahbspeed = systemclck / 8;
	} else if (AHBpresc == 11) {
		ahbspeed = systemclck / 16;
	} else if (AHBpresc == 12) {
		ahbspeed = systemclck / 64;
	} else if (AHBpresc == 13) {
		ahbspeed = systemclck / 128;
	} else if (AHBpresc == 14) {
		ahbspeed = systemclck / 256;
	} else if (AHBpresc == 15) {
		ahbspeed = systemclck / 512;
	}

	/*Get APB bus speed therefore I2C peripheral speed*/
	uint8_t APBpresc = (RCC->CFGR >> 10) & 0x7;

	if (APBpresc < 4){
		pclk1 = ahbspeed;
	} else if (APBpresc == 4){
		pclk1 = ahbspeed / 2;
	} else if (APBpresc == 5){
		pclk1 = ahbspeed / 4;
	} else if (APBpresc == 6){
		pclk1 = ahbspeed / 8;
	} else if (APBpresc == 7){
		pclk1 = ahbspeed / 16;
	}

	return pclk1;
}
/***************************************************************************
 * @fn					- I2C_Init
 *
 * @brief				- Configures I2C peripheral
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t temp = 0;		//Temporarily holds register configurations
	uint16_t ccrValue = 0;	//CRR field of I2C_CRR register

	//Set ack control bit
	temp |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 |= temp;
	temp = 0;

	//Set FREQ bits
	temp = RCC_GetPCLK1() / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (temp & 0x3F); //Mask bits so we don't overwrite
	temp = 0;

	//Set 7-bit slave address
	temp = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	temp |= (1 << 14);	//For reasons unknown this bit of the OAR1 register supposed to be maintained by software as 1 (see RM0090 reference manual)
	pI2CHandle->pI2Cx->OAR1 |= temp;
	temp = 0;

	//Calculate and set clock control
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){ //Standart mode (default)
		ccrValue = RCC_GetPCLK1() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		temp |= (ccrValue & 0xFFF); //Mask bits so we don't overwrite
	} else { //Fast mode
		temp |= (1 << 15); //Set fast mode
		temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccrValue = RCC_GetPCLK1() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else {
			ccrValue = RCC_GetPCLK1() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		temp |= (ccrValue & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR |= temp;
	temp = 0;
}


/***************************************************************************
 * @fn					- I2C_DeInit
 *
 * @brief				- Resets registers of a given I2C
 *
 * @param[in]			- Register addresses of a given I2C
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}


/***************************************************************************
 * @fn					- I2C_GenerateStartCondition
 *
 * @brief				- Helper function to initiate I2C communications
 *
 * @param[in]			- Register addresses of a given I2C
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/***************************************************************************
 * @fn					- I2C_MasterSendData
 *
 * @brief				- Sending data over a given I2C peripheral
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 * @param[in]			- Buffer that holds the byte to be sent
 * @param[in]			- Total byte count that will be sent
 * @param[in]			- Address of the device that master will convey it's message to
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_MasterSendData (I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr){
	//1. Generate start condition
	I2C_GenerateStartCondition (pI2CHandle->pI2Cx);

	//2.Confirm that start generation is completed by checking the SB flag in the SR1
	//Until SB is cleared SCL will be stretched
	//while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)){};

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1

	//5. Clear the ADDR flag according to its software sequence
	//Until ADDR is cleared SCL will be stretched

	//6. Send data until Len is 0

	//7.When Len becomes 0 wait for TXE = 1 and BTF = 1 before generating stop condition
	//TXE = 1 and BTF = 1 means SR and DR are empty and next transmission should begin when BTF = 1
	//SCL will be stretched

	//8. Generate stop condition
	//Master does not need to wait for completion of stop condition
	//Generating stop automatically clears BTF
	//
}

/***************************************************************************
 * @fn					- I2C_PeripheralControl
 *
 * @brief				- Enables or disables a given I2C peripheral
 *
 * @param[in]			- Register addresses of a given I2C
 * @param[in]			- Enable or disable command
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_PeripheralControl (I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/***************************************************************************
 * @fn					- I2C_IRQInterruptConfig
 *
 * @brief				- Enables or disables the relevant interrupt
 *
 * @param[in]			- IRQ number for the interrupt
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnOrDi){
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
 * @fn					- I2C_IRQPriorityConfig
 *
 * @brief				- Change priority of a given interrupt
 *
 * @param[in]			- Interrupt that the priority of will be changed
 * @param[in]			- New priority of the interrupt
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_IRQPriorityConfig (uint8_t IRQNumber, uint8_t IRQPriority){
	//Find correct IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}