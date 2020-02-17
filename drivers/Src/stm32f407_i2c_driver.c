/*
 * stm32f407_i2c_driver.c
 *
 *  Created on: Jan 7, 2020
 *      Author: Oguz CAN
 */

#include "stm32f407_i2c_driver.h"

/**
 * Helper functions
 */
//TO DO: Create new files for helper functions
static void I2C_GenerateStart(I2C_RegDef_t *pI2Cx);
static void I2C_AddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_AddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ClearADDR(I2C_Handle_t * pI2CHandle);
static void I2C_MasterHandleRxNEInterrupt(I2C_Handle_t *pI2CHandle );
static void I2C_MasterHandleTxEInterrupt(I2C_Handle_t *pI2CHandle );


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


/***************************************************************************
 * @fn					- AckControl
 *
 * @brief				- Sets or clears ack bit
 *
 * @param[in]			- I2Cx base address
 * @param[in]			- "ENABLE" or "DISABLE" command for ack control
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_AckControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}


/***************************************************************************
 * @fn					- I2C_GetFlagStatus
 *
 * @brief				- Fetches I2C_SR1 register flags
 *
 * @param[in]			- Register addresses of a given I2C
 * @param[in]			- Requested flag
 *
 * @return				- Flag status
 *
 * @Note				- none
 */
bool I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
	if (pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
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
 * @brief				- Configures I2C peripheral register given the
 * 						configuration
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
	uint32_t PCLK1_value = RCC_GetPCLK1(); //Peripheral clock in Hz

	//Enable the clock for I2Cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//Set FREQ bits
	temp = PCLK1_value / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (temp & 0x3F); //Mask bits so we don't overwrite
	temp = 0;

	//Set 7-bit slave address (In slave mode)
	temp = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	temp |= (1 << 14);	//This bit of the OAR1 register supposed to be maintained by software as 1 (see RM0090 reference manual)
	pI2CHandle->pI2Cx->OAR1 |= temp;
	temp = 0;

	//Calculate and set clock control
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){ //Standard mode (default)
		ccrValue = PCLK1_value / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		temp |= (ccrValue & 0xFFF); //Mask bits so we don't overwrite
	} else { //Fast mode
		temp |= (1 << 15); //Set fast mode
		temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccrValue = PCLK1_value / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else {
			ccrValue = PCLK1_value / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		temp |= (ccrValue & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR |= temp;
	temp = 0;

	//TRISE configuration
	//Maximum rise time-=>These bits must be programmed with the maximum SCL rise time given in the I2C bus
	//specification, incremented by 1 (see RM0090 reference manual)
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){ //Standard mode (default)
		temp = (PCLK1_value / 1000000U) + 1;
	} else { //Fast mode
		temp = ((PCLK1_value * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = 0; //Reset value of TRISE is not 0 (0x0002)
	pI2CHandle->pI2Cx->TRISE |= (temp & 0x3F);
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
		I2C_AckControl(pI2Cx, ENABLE); //Enable ack, this can not be set before PE = 1
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/***************************************************************************
 * @fn					- I2C_GenerateStart
 *
 * @brief				- Helper function to initiate I2C communications
 *
 * @param[in]			- Register addresses of a given I2C
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_GenerateStart(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}


/***************************************************************************
 * @fn					- I2C_AddressPhaseWrite
 *
 * @brief				- Helper function to initiate address phase of I2C
 * 						communications when in transmission
 *
 * @param[in]			- Register addresses of a given I2C
 * @param[in]			- Address of the slave 
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_AddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr){
	slaveAddr = slaveAddr << 1; //This byte contains slave address and r/w bit
	slaveAddr &= ~(1);			//Set r/w bit to 'write' aka. clear this bit
	pI2Cx->DR = slaveAddr;
}


/***************************************************************************
 * @fn					- I2C_AddressPhaseRead
 *
 * @brief				- Helper function to initiate address phase of I2C
 * 						communications when in reception
 *
 * @param[in]			- Register addresses of a given I2C
 * @param[in]			- Address of the slave
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_AddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr){
	slaveAddr = slaveAddr << 1; //This byte contains slave address and r/w bit
	slaveAddr |= 1;				//Set r/w bit to 'read' aka. set this bit
	pI2Cx->DR = slaveAddr;
}



/***************************************************************************
 * @fn					- I2C_ClearADDR
 *
 * @brief				- Helper function to clear ADDR flag
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 *
 * @return				- none
 *
 * @Note				- ADDR flag is cleared when SR1 is read followed
 * 						by SR2 is read operations or by hardware when PE
 * 						flag is 0 (see reference manual: RM0090)
 */
static void I2C_ClearADDR(I2C_Handle_t *pI2CHandle){
	uint32_t foo;

	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX && pI2CHandle->RxSize == 1) {
		//Disable ack
		I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);

		//Clear ADDR
		foo = pI2CHandle->pI2Cx->SR1;
		foo = pI2CHandle->pI2Cx->SR2;
		(void) foo; //Cast it to void to avoid compiler warning

	} else {
		//Clear ADDR
		foo = pI2CHandle->pI2Cx->SR1;
		foo = pI2CHandle->pI2Cx->SR2;
		(void) foo; //Cast it to void to avoid compiler warning
		}
	}
}

/***************************************************************************
 * @fn					- I2C_GenerateStop
 *
 * @brief				- Helper function to produce stop condition
 *
 * @param[in]			- Register addresses of a given I2C
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_GenerateStop(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}


/***************************************************************************
 * @fn					- I2C_MasterSendData
 *
 * @brief				- Sending data over a given I2C peripheral
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 * @param[in]			- Buffer that holds the data to be sent
 * @param[in]			- Total byte count that will be sent
 * @param[in]			- Address of the device that master will convey it's message to
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_MasterSendData (I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, bool Sr){
	//1. Generate start condition
	I2C_GenerateStart (pI2CHandle->pI2Cx);

	//2.Confirm that start generation is completed by checking the SB flag in the SR1
	//Until SB is cleared SCL will be stretched
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)){};

	//3. Send the address of the slave with r/w bit set to w(0) (total of 8 bits)
	I2C_AddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)){};

	//5. Clear the ADDR flag according to its software sequence
	//Until ADDR is cleared SCL will be stretched
	I2C_ClearADDR(pI2CHandle);

	//6. Send data until Len is 0
	for (uint32_t i = Len; i > 0; i--) {
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE)) {} //Check if transmission data register is empty
		pI2CHandle->pI2Cx->DR = *pTxBuffer; //Write to data register
		pTxBuffer++; //Increment data buffer
	}

	//7.When Len becomes 0 wait for TXE = 1 and BTF = 1 before generating stop condition
	//TXE = 1 and BTF = 1 means SR and DR are empty and next transmission should begin when BTF = 1
	//SCL will be stretched
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE)){}; //Check if data register is empty
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)){}; //Check if last byte transmission is completed
	
	//8. Generate stop condition
	//Master does not need to wait for completion of stop condition
	//Generating stop automatically clears BTF
	if (Sr == I2C_DISABLE_SR) {
		I2C_GenerateStop(pI2CHandle->pI2Cx);
	}
}


/***************************************************************************
 * @fn					- I2C_MasterReceiveData
 *
 * @brief				- Receiving data over a given I2C peripheral, polling
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 * @param[in]			- Buffer that will record the received message
 * @param[in]			- Total byte count that will be received
 * @param[in]			- Address of the device that master will receive it's message from
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_MasterReceiveData (I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, bool Sr){
	//1.Generate start condition
	I2C_GenerateStart (pI2CHandle->pI2Cx);

	//2.Confirm start generation is completed by checking I2C_SR1 SB
	//Until SB is cleared SCL will be stretched
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)){};

	//3.Send the address of the slave with r/w bit set to r(1) (total of 8 bits)
	I2C_AddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4.Wait until address phase is completed by checking the I2C_SR1 ADDR
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)){};

	//Read only 1 byte from slave
	if (Len == 1){
		//1.Disable Ack (signals end of reception)
		I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);

		//2.Clear ADDR flag
		I2C_ClearADDR(pI2CHandle);

		//3.Wait until RxNE = 1 so data register is not empty (wait for reception to be complete)
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE)){};

		//4.Generate stop condition
		if (Sr == I2C_DISABLE_SR) {
			I2C_GenerateStop(pI2CHandle->pI2Cx);
		}

		//5.Read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	} else {
		//1.Clear ADDR flag
		I2C_ClearADDR(pI2CHandle);

		//2.Start reception
		for (uint32_t i = Len; i > 0; i--) {
			//Wait until RxNE is 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE)){};

			if (i == 2) { //When there is 2 bytes remaining close I2C reception
				//Disable ack before receiving last byte
				I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);

				//Generate stop condition
				if (Sr == I2C_DISABLE_SR) {
					I2C_GenerateStop(pI2CHandle->pI2Cx);
				}
			}

			//3.Read the data from data register
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//4.Increment buffer address
			pRxBuffer++;
		}
	}

	//Enable Ack
	if (pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE) {
		I2C_AckControl(pI2CHandle->pI2Cx, ENABLE);
	}
}


/***************************************************************************
 * @fn					- I2C_SlaveSendData
 *
 * @brief				- Sends data over a given I2C peripheral one byte at a time
 *
 * @param[in]			- I2C structure for register addresses
 * @param[in]			- Data byte to be sent
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_SlaveSendData (I2C_RegDef_t *pI2Cx, uint8_t data){
	pI2Cx->DR = data;
}


/***************************************************************************
 * @fn					- I2C_SlaveSendData
 *
 * @brief				- Sends data over a given I2C peripheral one byte at a time
 *
 * @param[in]			- I2C structure for register addresses
 *
 * @return				- Received data
 *
 * @Note				- none
 */
uint8_t I2C_SlaveReceiveData (I2C_RegDef_t *pI2Cx){
	return (uint8_t) pI2Cx->DR;
}


/***************************************************************************
 * @fn					- I2C_MasterSendDataIT
 *
 * @brief				- Saves buffer address and length information and
 * 						  enables I2C interrupt
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 * @param[in]			- Buffer that will record the received message
 * @param[in]			- Total byte count that will be received
 * @param[in]			- Address of the device that master will receive it's message from
 *
 * @return				- State that shows if the peripheral is busy in transmission
 *
 * @Note				- This function will block until Len is 0
 */
uint8_t I2C_MasterSendDataIT (I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, bool Sr){
	uint8_t	busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate start Condition
		I2C_GenerateStart (pI2CHandle->pI2Cx); //SB bit will be set and cause an interrupt when enabled

		//Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}


/***************************************************************************
 * @fn					- I2C_MasterReceiveDataIT
 *
 * @brief				- Saves buffer address and length information and
 * 						  enables I2C interrupt
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 * @param[in]			- Buffer that will record the received message
 * @param[in]			- Total byte count that will be received
 * @param[in]			- Address of the device that master will receive it's message from
 *
 * @return				- State that shows if the peripheral is busy in transmission
 *
 * @Note				- This function will block until Len is 0
 */
uint8_t I2C_MasterReceiveDataIT (I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, bool Sr){
	uint8_t	busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate start Condition
		I2C_GenerateStart (pI2CHandle->pI2Cx); //SB bit will be set and cause an interrupt when enabled

		//Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
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


/***************************************************************************
 * @fn					- I2C_MasterHandleRXNEInterrupt
 *
 * @brief				- Handles interrupt based reception
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_MasterHandleRxNEInterrupt(I2C_Handle_t *pI2CHandle) {
	//Reception
	if (pI2CHandle->RxSize == 1) {
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1) {
		if (pI2CHandle->RxLen == 2) {
			//Clear ACK bit
			I2C_AckControl (pI2CHandle->pI2Cx, DISABLE);
		}
		//Read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxLen == 0) {
		//close the I2C data reception and notify the application

		//Generate the stop condition
		if (pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStop(pI2CHandle->pI2Cx);

		//Close the I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		//Notify application that reception is completed
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}


/***************************************************************************
 * @fn					- I2C_MasterHandleTxEInterrupt
 *
 * @brief				- Handles interrupt based transmission
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_MasterHandleTxEInterrupt(I2C_Handle_t *pI2CHandle) {
	if (pI2CHandle->TxLen > 0) {
		//Load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;
	}
}


/***************************************************************************
 * @fn					- I2C_EV_IRQHandling
 *
 * @brief				- I2C event interrupt handler for master and slave modes
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {
	uint32_t temp1, temp2, temp3;
	//TO DO: Generalize I2C_GetFlagStatus()
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

/*************************************************************************************************/
	//1.SB event
	//SB flag only set in master mode
	if (temp1 && temp3) {
		//Start address phase
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) { //Set r/w bit for transmission
			I2C_AddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) { //Set r/w bit for reception
			I2C_AddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

/*************************************************************************************************/
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2.ADDR event
	//Master mode -> Address is sent
	//Slave mode -> Address received matches own address
	if (temp1 && temp3) {
		//Clear ADDR
		I2C_ClearADDR(pI2CHandle);
	}

/*************************************************************************************************/
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3.BTF(Byte Transfer Finished) event
	if (temp1 && temp3) {
		//Check Tx/Rx state
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			//Make sure that TxE is also set
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE)) {
				//BTF, TXE = 1
				if (pI2CHandle->TxLen == 0) {
					//Generate the STOP condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStop(pI2CHandle->pI2Cx);

					//Reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//Notify application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}

		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {

		}
	}

/*************************************************************************************************/
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4.STOPF event
	//Stop detection flag is applicable only in slave mode
	if (temp1 && temp3) {
		//Clear the STOPF -> read SR1 then write to CR1

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

/*************************************************************************************************/
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);
	//5.TXE event
	if (temp1 && temp2 && temp3) {
		//Check device mode
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			//Master mode TxE flag is set
			//We have to transmit
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				I2C_MasterHandleTxEInterrupt(pI2CHandle);
			}
		} else { //Slave mode
			if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) { //Confirm slave is in transmitter mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

/*************************************************************************************************/
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
	//6. Handle For interrupt generated by RxNE event
	if (temp1 && temp2 && temp3) {
		//Check device mode .
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) { //Master mode RxNE flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) { //In reception?
				I2C_MasterHandleRxNEInterrupt(pI2CHandle);
			}
		} else { //Slave mode RxNE flag is set
			if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))) { //Confirm slave is in receiver mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV); //Notify application
			}
		}
	}
}


/***************************************************************************
 * @fn					- I2C_ER_IRQHandling
 *
 * @brief				- I2C error interrupt handler
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {

	uint32_t temp1, temp2;

	//Status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

/*************************************************************************************************/
//Bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if (temp1 && temp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

/*************************************************************************************************/
//Arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
	if (temp1 && temp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

/*************************************************************************************************/
//ACK failure  error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if (temp1 && temp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/*************************************************************************************************/
//Overrun/underrun error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if (temp1 && temp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

/*************************************************************************************************/
//Time out error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if (temp1 && temp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}


/***************************************************************************
 * @fn					- I2C_SlaveCallbackEventControl
 *
 * @brief				- Control slave interrupt flags
 *
 * @param[in]			- I2C structure for register addresses
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_SlaveCallbackEventControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE){
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	} else {
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}


/***************************************************************************
 * @fn					- I2C_CloseSendData
 *
 * @brief				- Disable interrupt and reset member elements of
 * 						the handle structure
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
	//Disable ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable ITEVTEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

	if (pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE) {
		I2C_AckControl(pI2CHandle->pI2Cx, ENABLE);
	}
}

/***************************************************************************
 * @fn					- I2C_CloseReceiveData
 *
 * @brief				- Disable interrupt and reset member elements of
 * 						the handle structure
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
	//Disable ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable ITEVTEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if (pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE) {
		I2C_AckControl(pI2CHandle->pI2Cx, ENABLE);
	}
}

