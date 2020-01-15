/*
 * stm32f407_SPI.driver.c
 *
 *  Created on: Dec 23, 2019
 *      Author: Oguz CAN
 */
#include "stm32f407_spi_driver.h"

/***************************************************************************
 * @fn					- SPI_PeriClockControl
 *
 * @brief				- Enables or disables clock for a given SPI
 *
 * @param[in]			- SPIx base address
 * @param[in]			- "ENABLE" or "DISABLE" command for the SPIx clock
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if (EnOrDi) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		} else if (pSPIx == SPI5) {
			SPI5_PCLK_EN();
		} else if (pSPIx == SPI6) {
			SPI6_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
		} else if (pSPIx == SPI5) {
			SPI5_PCLK_DI();
		} else if (pSPIx == SPI6) {
			SPI6_PCLK_DI();
		}
	}
}

/***************************************************************************
 * @fn					- SPI_Init
 *
 * @brief				- Configures SPI peripheral
 *
 * @param[in]			- SPI structure that holds configuration and SPI address
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	//Enable SPI clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempreg = 0; //Temporary register to hold configured values

	//1- Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2- Configure bus configuration
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//BIDI mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY){
		//BIDI mode should be cleared RXONLY should be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3- Configure SPI Clock speed (baud)
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//4- Configure DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5- Configure CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6- Configure CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//7- Configure SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}


/***************************************************************************
 * @fn					- SPI_DeInit
 *
 * @brief				- Resets registers of a given SPI
 *
 * @param[in]			- Register addresses of a given SPI
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	} else if (pSPIx == SPI5) {
		SPI5_REG_RESET();
	} else if (pSPIx == SPI6) {
		SPI6_REG_RESET();
	}
}

/***************************************************************************
 * @fn					- SPI_PeripheralControl
 *
 * @brief				- Enables or disables a given SPI peripheral
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Enable or disable command
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_PeripheralControl (SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/***************************************************************************
 * @fn					- SPI_GetFlagStatus
 *
 * @brief				- Fetches SPI_SR register flags
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Requested flag
 *
 * @return				- Flag status
 *
 * @Note				- none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/***************************************************************************
 * @fn					- SPI_SSIConfig
 *
 * @brief				- Enables or disables internal slave select when
 * 						  software slave management is enabled
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Enable or disable command
 *
 * @return				- Flag status
 *
 * @Note				- none
 */
void SPI_SSIConfig (SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/***************************************************************************
 * @fn					- SPI_SSOEConfig
 *
 * @brief				- Slave select output control function
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Enable or disable command
 *
 *
 * @return				- none
 *
 * @Note				- This function will block until Len is 0
 */
void SPI_SSOEConfig (SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/***************************************************************************
 * @fn					- SPI_SendData
 *
 * @brief				- Sends the desired data through selected SPI interface
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Buffer for transmission data
 * @param[in]			- Data length
 *
 * @return				- none
 *
 * @Note				- This function will block until Len is 0
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while (Len > 0){
		/*1- Wait until TXE is set (Tx buffer is empty)*/
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TxE) == FLAG_RESET);
		/*2- Check DFF bit*/
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			//16-bit DFF
			//3- Load data in to the Data Register
			pSPIx->DR = *((uint16_t *)pTxBuffer);	//Write to data register
			Len -= 2;							//Decrease length of the data to be sent by 2 bytes
			(uint16_t *)pTxBuffer++;				//Increase Tx buffer address by 2 bytes for the next package
		} else {
			//8-bit DFF
			//3- Load data in to the Data Register
			pSPIx->DR = *pTxBuffer;					//Write to data register
			Len --;									//Decrease length of the data to to be sent by 1 byte
			pTxBuffer++;							//Increase Tx buffer address by 1 byte for the next package
		}
	}
}


/***************************************************************************
 * @fn					- SPI_ReceiveData
 *
 * @brief				- Receives data from specified SPI peripheral
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Buffer for reception data
 * @param[in]			- Expected data length for reception
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
while (Len > 0){
	/*1- Wait until RXNE is set (Rx buffer is not empty)*/
	while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_RxNE) == (uint8_t) FLAG_RESET);
	/*2- Check DFF bit*/
	if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		//16-bit DFF
		//3- Read from Data Register
		*((uint16_t *)pRxBuffer) = pSPIx->DR;	//Save register data to Rx buffer
		Len -= 2;							//Decrease length of the data to be received by 2 bytes
		(uint16_t *)pRxBuffer++;				//Increase Rx buffer address by 2 bytes for the next package
	} else {
		//8-bit DFF
		//3- Read from Data Register
		*pRxBuffer = pSPIx->DR;					//Save register data to Rx buffer
		Len --;									//Decrease length of the data to to be sent by 1 byte
		pRxBuffer++;							//Increase Rx buffer address by 1 byte for the next package
		}
	}
}


/***************************************************************************
 * @fn					- SPI_SendDataIT
 *
 * @brief				- Saves buffer address and length information and
 * 						  enables SPI interrupt using TXEIE
 *
 * @param[in]			- Structure that holds configuration and register information
 * @param[in]			- Buffer for transmission data
 * @param[in]			- Data length
 *
 * @return				- State that shows if the peripheral is busy in transmission
 *
 * @Note				- This function will block until Len is 0
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->TxState;

	if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		//1. Save Tx buffer address and length information in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code can take over same peripheral
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmission will be handled by the ISR code
	}

	return state;
}


/***************************************************************************
 * @fn					- SPI_ReceiveDataIT
 *
 * @brief				-
 *
 * @param[in]			- Structure that holds configuration and register information
 * @param[in]			- Buffer for reception data
 * @param[in]			- Expected data length
 *
 * @return				- State that shows if the peripheral is busy in reception
 *
 * @Note				- none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {
	uint8_t state = pSPIHandle->RxState;

	if (pSPIHandle->RxState != SPI_BUSY_IN_RX) {
		//1. Save Rx buffer address and length information in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code can take over same peripheral
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Data transmission will be handled by the ISR code
	}

	return state;
}


/***************************************************************************
 * @fn					- SPI_IRQInterruptConfig
 *
 * @brief				- Enables or disables the relevant interrupt
 *
 * @param[in]			- IRQ number for the interrupt
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_IRQInterruptConfig		(uint8_t IRQNumber, uint8_t EnOrDi){
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
 * @fn					- SPI_TXE_Interrupt_Handle
 *
 * @brief				- SPI transmission interrupt handler
 *
 * @param[in]			- Structure that holds configuration and register information
 *
 * @return				- none
 *
 * @Note				- none
 */
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pHandle){

	//Check DFF bit
	if (pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		//16-bit DFF
		//Load data in to the Data Register
		pHandle->pSPIx->DR = *((uint16_t*) pHandle->pTxBuffer);	//Write to data register
		pHandle->TxLen -= 2;				//Decrease length of the data to be sent by 2 bytes
		(uint16_t*) pHandle->pTxBuffer++;	//Increase Tx buffer address by 2 bytes for the next package
	} else {
		//8-bit DFF
		//Load data in to the Data Register
		pHandle->pSPIx->DR = *pHandle->pTxBuffer;	//Write to data register
		pHandle->TxLen--;		//Decrease length of the data to to be sent by 1 byte
		pHandle->pTxBuffer++;	//Increase Tx buffer address by 1 byte for the next package
	}

	if (!pHandle->TxLen){ //Transmission is finished, close SPI communications, inform application

		pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE); 	//Clear interrupt pending bit
		pHandle->pTxBuffer = NULL;						//Clear transmission buffer
		pHandle->TxLen = 0;								//Set transmission length to zero
		pHandle->TxState = SPI_READY;					//Set SPI state as ready

		SPI_ApplicationEventCallback (pHandle, SPI_EVENT_TX_CMPLT);
	}
}


/***************************************************************************
 * @fn					- SPI_RXNE_Interrupt_Handle
 *
 * @brief				- SPI reception interrupt handler
 *
 * @param[in]			- Structure that holds configuration and register information
 *
 * @return				- none
 *
 * @Note				- none
 */
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pHandle){
	//Check DFF bit
	if (pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		//16-bit DFF
		//Read from Data Register
		*((uint16_t*) pHandle->pRxBuffer) = (uint16_t)pHandle->pSPIx->DR;	//Save register data to Rx buffer
		pHandle->RxLen -= 2;		//Decrease length of the data to be received by 2 bytes
		pHandle->pRxBuffer += 2;	//Increase Rx buffer address by 2 bytes for the next package
	} else {
		//8-bit DFF
		//Read from Data Register
		*(pHandle->pRxBuffer) = (uint8_t) pHandle->pSPIx->DR;	//Save register data to Rx buffer
		pHandle->RxLen--;		//Decrease length of the data to to be sent by 1 byte
		pHandle->pRxBuffer++;	//Increase Rx buffer address by 1 byte for the next package
	}

	if (!pHandle->RxLen) { //Reception is finished, close SPI communications, inform application

		pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE); 	//Clear interrupt pending bit
		pHandle->pRxBuffer = NULL;						//Clear reception buffer
		pHandle->RxLen = 0;								//Set reception length to zero
		pHandle->RxState = SPI_READY;					//Set SPI state as ready

		SPI_ApplicationEventCallback (pHandle, SPI_EVENT_RX_CMPLT);
	}
}


/***************************************************************************
 * @fn					- SPI_OVRerr_Interrupt_Handle
 *
 * @brief				- SPI overrun error interrupt handler
 *
 * @param[in]			- Structure that holds configuration and register information
 *
 * @return				- none
 *
 * @Note				- none
 */
static void SPI_OVRerr_Interrupt_Handle(SPI_Handle_t *pHandle){
	//Clear overrun flag (refer to reference manual for OVR error and how to clear it's flag)
	uint8_t temp;
	if (pHandle->TxState != SPI_BUSY_IN_TX){
		temp = pHandle->pSPIx->DR;
		temp =pHandle->pSPIx->SR;
	}
	(void) temp; //To clear compiler warning for unused variable
	//Inform application
	SPI_ApplicationEventCallback (pHandle, SPI_EVENT_OVR_ERR);
}


/***************************************************************************
 * @fn					- SPI_IRQHandling
 *
 * @brief				-
 *
 * @param[in]			- Structure that holds configuration and register information
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_IRQHandling (SPI_Handle_t *pHandle) { //TO DO: implement MODF, CRCERR and FRE error handling
	uint8_t temp1, temp2;

	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if (temp1 && temp2) { //Check for transmission interrupt
		//Handle transmission
		SPI_TXE_Interrupt_Handle(pHandle);
	}


	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2) { //Check for transmission interrupt
		//Handle transmission
		SPI_RXNE_Interrupt_Handle(pHandle);
	}


	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2) { //Check for overrun error
		//Error Handling
		SPI_OVRerr_Interrupt_Handle(pHandle);
	}
}


/***************************************************************************
 * @fn					- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint8_t IRQPriority){
	//Find correct IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


/***************************************************************************
 * @fn					- SPI_ApplicationEventCallback
 *
 * @brief				- Returns events from interrupt to the application
 *
 * @param[in]			- Structure that holds configuration and register information
 * @param[in]			- Event that will be returned to the application
 *
 * @return				- none
 *
 * @Note				- This function can be re-defined in application for more specific operations
 */
__attribute__((weak)) void SPI_ApplicationEventCallback (SPI_Handle_t *pHandle, uint8_t AppEvent){

}
