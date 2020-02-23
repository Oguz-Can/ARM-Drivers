/*
 * stm32f407_usart_driver.c
 *
 *  Created on: 18 Feb 2020
 *      Author: Oguz
 */

#include "stm32f407_usart_driver.h"

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
static uint32_t RCC_GetPLLOutputClock() {
	uint32_t PLLclock = 0;
	return PLLclock;
}

/***************************************************************************
 * @fn					- RCC_GetPCLK1
 *
 * @brief				- Helper function to calculate peripheral clock 1
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
	uint32_t pclk1 = 0;			//APB1 bus speed

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

	/*Get APB bus speed*/
	uint8_t APBpresc = (RCC->CFGR >> 10) & 0x7;

	if (APBpresc < 4) {
		pclk1 = ahbspeed;
	} else if (APBpresc == 4) {
		pclk1 = ahbspeed / 2;
	} else if (APBpresc == 5) {
		pclk1 = ahbspeed / 4;
	} else if (APBpresc == 6) {
		pclk1 = ahbspeed / 8;
	} else if (APBpresc == 7) {
		pclk1 = ahbspeed / 16;
	}
	return pclk1;
}


/***************************************************************************
 * @fn					- RCC_GetPCLK2
 *
 * @brief				- Helper function to calculate peripheral clock 2
 *
 * @param[in]			- none
 *
 * @return				- PCLK2
 *
 * @Note				- none
 */
static uint32_t RCC_GetPCLK2() {
	uint32_t systemclck = 0;	//Systick
	uint32_t ahbspeed = 0;		//AHB bus speed
	uint32_t pclk2 = 0;			//APB2 bus speed

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

	/*Get APB bus speed*/
	uint8_t APBpresc = (RCC->CFGR >> 13) & 0x7;

	if (APBpresc < 4) {
		pclk2 = ahbspeed;
	} else if (APBpresc == 4) {
		pclk2 = ahbspeed / 2;
	} else if (APBpresc == 5) {
		pclk2 = ahbspeed / 4;
	} else if (APBpresc == 6) {
		pclk2 = ahbspeed / 8;
	} else if (APBpresc == 7) {
		pclk2 = ahbspeed / 16;
	}
	return pclk2;
}


/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - Helper function to calculate register
 * 						configuration for desired baud rate
 *
 * @param[in]         - USART register addresses
 * @param[in]         - Desired baud rate
 *
 * @return            -
 *
 * @Note              -
 */
static void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate) {
	//TO DO: OVER8 = 1, DIV_Fraction3 is not considered, must overflow to mantissa and should be kept clear
	
	uint32_t PCLKx;
	uint32_t usartdiv;

	uint32_t mantissa, fraction;

	uint32_t tempreg = 0;

	//Get the value of APB bus clock
	if (pUSARTx == USART1 || pUSARTx == USART6) {
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2();
	} else {
		PCLKx = RCC_GetPCLK1();
	}

	//Check for OVER8 configuration bit
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		//OVER8 = 1, over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	} else {
		//OVER8 = 0, over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	//Calculate the Mantissa part
	mantissa = usartdiv / 100;


	//Extract the fraction part
	fraction = (usartdiv - (mantissa * 100));

	//Calculate the final fractional
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		//OVER8 = 1 , over sampling by 8
		fraction = (((fraction * 8) + 50) / 100) & ((uint8_t) 0x07);
		if(fraction > 7){
			fraction = 0;
			mantissa++;
		}
	} else {
		//Over sampling by 16
		fraction = (((fraction * 16) + 50) / 100) & ((uint8_t) 0x0F);
	}

	//Place the Mantissa part in appropriate bit position, refer to USART_BRR
	tempreg |= mantissa << 4;

	//Place the fractional part in appropriate bit position, refer to USART_BRR
	tempreg |= fraction;

	//Copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
	tempreg = 0;
}

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - Configures USART registers given the configuration
 *
 * @param[in]         - USART structure that holds configuration and USART address
 *
 * @return            - none
 *
 * @Note              - none
 */
void USART_Init(USART_Handle_t *pUSARTHandle) {
	uint32_t tempreg = 0;

	/******************************** Configuration of CR1******************************************/
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE); //Enable the clock for given USART peripheral


	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) {
		tempreg |= (1 << USART_CR1_RE); //Receiver bit field

	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX) {
		tempreg |= (1 << USART_CR1_TE); //Transmitter bit field

	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX) {
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE)); //Both transmitter and receiver bit field
	}

	tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M); //Configure word length

	//Configuration of parity control bit fields
	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN) {
		tempreg |= (1 << USART_CR1_PCE); //Enable parity control
		//By default EVEN parity will be selected once parity control is enabled

	} else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD) {
		tempreg |= (1 << USART_CR1_PCE); //Enable parity control
		tempreg |= (1 << USART_CR1_PS); //Set parity control to odd parity
	}
	pUSARTHandle->pUSARTx->CR1 = tempreg; //Write configuration to CR1
	tempreg = 0;

	/******************************** Configuration of CR2******************************************/
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP; //Configure number of stop bits

	pUSARTHandle->pUSARTx->CR2 = tempreg; //Write configuration to CR2
	tempreg = 0;

	/******************************** Configuration of CR3******************************************/
	//Configuration of USART hardware flow control
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
		tempreg |= (1 << USART_CR3_CTSE); //CTS flow control

	} else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
		tempreg |= (1 << USART_CR3_RTSE); //RTS flow control

	} else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
		//CTS and RTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;
	tempreg = 0;

	/******************************** Configuration of BRR******************************************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);
}

/***************************************************************************
 * @fn					- USART_DeInit
 *
 * @brief				- Resets registers of a given USART
 *
 * @param[in]			- Register addresses of a given USART
 *
 * @return				- none
 *
 * @Note				- none
 */
void USART_DeInit (USART_RegDef_t *pUSARTx) {
//TO DO
}

/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - Enables or disables a given USART/UART peripheral
 *
 * @param[in]         - USART address
 * @param[in]         - Enable or disable command
 *
 * @return            - none
 *
 * @Note              - none
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	} else {
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - Enables the clock of a given USART/UART peripheral
 *
 * @param[in]         - USART address
 * @param[in]         - Enable or disable command
 *
 * @return            - none
 *
 * @Note              - none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		if (pUSARTx == USART1) {
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_EN();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_EN();
		} else if (pUSARTx == UART4) {
			UART4_PCLK_EN();
		} else if (pUSARTx == UART5) {
			UART5_PCLK_EN();
		} else if (pUSARTx == USART6) {
			USART6_PCLK_EN();
		}
	} else {
		if (pUSARTx == USART1) {
			USART1_PCLK_DI();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_DI();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_DI();
		} else if (pUSARTx == UART4) {
			UART4_PCLK_DI();
		} else if (pUSARTx == UART5) {
			UART5_PCLK_DI();
		} else if (pUSARTx == USART6) {
			USART6_PCLK_DI();
		}
	}
}

/***************************************************************************
 * @fn					- USART_GetFlagStatus
 *
 * @brief				- Fetches USART_SR register flags
 *
 * @param[in]			- Register addresses of a given USART
 * @param[in]			- Requested flag
 *
 * @return				- Flag status
 *
 * @Note				- none
 */
bool USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName) {
	if (pUSARTx->SR & FlagName) {
		return true;
	}
	return false;
}

/***************************************************************************
 * @fn					- USART_SendData
 *
 * @brief				- Sends data over a given USART peripheral
 *
 * @param[in]			- USART structure that holds configuration and USART address
 * @param[in]			- Buffer that holds the data to be sent
 *
 * @return				- none
 *
 * @Note				- none
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len) {

	uint16_t *pdata;

	//Loop over until all bytes are transferred
	for (uint32_t i = 0; i < Len; i++) {
		//Wait until TXE flag is set
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE)) {}

		//Check the USART_WordLength item for 9-bit or 8-bit in a frame
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			//if 9-bit load the DR with 2 bytes while masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);

			//Check parity control
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				//No parity, 9-bits of user data will be sent
				pTxBuffer += 2;
			} else {
				//Parity bit is used, 8-bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		} else {
			//8-bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t) 0xFF);

			//Increment the buffer address
			pTxBuffer++;
		}
	}

	//Wait until transmission is completed
	while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC)){}
}

/***************************************************************************
 * @fn					- USART_ReceiveData
 *
 * @brief				- Receives data over a given USART peripheral
 *
 * @param[in]			- USART structure that holds configuration and USART address
 * @param[in]			- Buffer that will record the received message
 *
 * @return				- none
 *
 * @Note				- none
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {
	//Loop over until all bytes are transferred
	for (uint32_t i = 0; i < Len; i++) {
		//Wait until RXNE flag is set in the SR
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE)){}

		//Check the USART_WordLength item for 9-bit or 8-bit in a frame
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			//9-bit frame

			//Check parity control
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				//No parity
				//Read first 9 bits
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01FF);

				pRxBuffer += 2;

			} else {
				//Parity enabled, 8-bit user data 1-bit parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);
				pRxBuffer++;
			}
		} else {
			//8-bit frame

			//Check parity control
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				//No parity

				//read 8 bits from DR
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);
			}

			else {
				//Parity enabled, 7-bit user data 1-bit parity
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0x7F);
			}
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn					- USART_SendDataIT
 *
 * @brief				- Enables and initializes interrupt based transmission
 *
 * @param[in]			- USART structure that holds configuration and USART address
 * @param[in]			- Buffer that holds the data to be sent
 * @param[in]			- Data length
 *
 * @return				- none
 *
 * @Note				- none
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len) {

	uint8_t txstate = pUSARTHandle->TxBusyState;

	if (txstate != USART_BUSY_IN_TX) {
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Enable TXE interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//Enable TC interrupt
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}
	return txstate;
}

/*********************************************************************
 * @fn					- USART_ReceiveDataIT
 *
 * @brief				- Enables and initializes interrupt base reception
 *
 * @param[in]			- USART structure that holds configuration and USART address
 * @param[in]			- Buffer that will record the received messagee
 * @param[in]			- Data length
 *
 * @return				- none
 *
 * @Note				- none
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if (rxstate != USART_BUSY_IN_RX) {
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void) pUSARTHandle->pUSARTx->DR;

		//Enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}

/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - Helper function to clear USART flags
 *
 * @param[in]         - USART register addresses
 * @param[in]         - Flag name
 *
 * @return            - none
 *
 * @Note              - Applicable to USART_CTS_FLAG, USART_LBD_FLAG,
 *						USART_TC_FLAG and USART_TC_FLAG flags
 *
 */

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName) {
	pUSARTx->SR &= ~(StatusFlagName);
}

/***************************************************************************
 * @fn					- USART_IRQInterruptConfig
 *
 * @brief				- Enables or disables the relevant interrupt
 *
 * @param[in]			- IRQ number for the interrupt
 *
 * @return				- none
 *
 * @Note				- none
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
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

/*********************************************************************
 * @fn					- USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	//Find correct IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn					- USART_IRQHandler
 *
 * @brief				- Handles interrupt based communication
 *
 * @param[in]			- USART structure that holds configuration and USART address
 *
 * @return				- none
 *
 * @Note				- none
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle) {

	uint32_t temp1, temp2;
	//uint32_t temp3;

	uint16_t *pdata;

	/*************************Check for TC flag ********************************************/
	//Check TC bit
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);

	//Check TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if (temp1 && temp2) { //TC & TCEIE
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {
			if (!pUSARTHandle->TxLen) { //Check if we have data to send
				//Clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);
				//Reset state
				pUSARTHandle->TxBusyState = USART_READY;
				//Reset Buffer address
				pUSARTHandle->pTxBuffer = NULL;
				//Reset data length
				pUSARTHandle->TxLen = 0;
				//Return the state to application
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	/*************************Check for TXE flag ********************************************/
	//Check TXE bit
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);

	//Check TXEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if (temp1 && temp2) { //TXE & TXEIE
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {
			if (pUSARTHandle->TxLen > 0) { //Send data if have any
				if (pUSARTHandle->USART_Config.USART_WordLength	== USART_WORDLEN_9BITS) { //Check word length
					//9-bit frame load the DR with 2 bytes with masking
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);
					//Check if parity is enabled
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
						//No parity, 9-bits of data
						pUSARTHandle->pTxBuffer += 2;
						pUSARTHandle->TxLen -= 2;
					} else {
						//Parity enabled 8-bits of data
						//9th bit will be replaced with parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 1;
					}
				} else {
					//8-bit frame
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t) 0xFF);

					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen -= 1;
				}
			}
			if (pUSARTHandle->TxLen == 0) { //No more data to send
				//Clear TXEIE bit
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	/*************************Check for RXNE flag ********************************************/
	//Check RXNE flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);

	//Check RXNEIE flag
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if (temp1 && temp2) { //RXNE & RXNEIE
		if (pUSARTHandle->RxBusyState == USART_BUSY_IN_RX) {
			if (pUSARTHandle->RxLen > 0) {

				if (pUSARTHandle->USART_Config.USART_WordLength	== USART_WORDLEN_9BITS) { //Check word length
					//9-bit frame
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) { //Check if parity enabled
						//No parity
						//Read first 9-bits
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01FF);

						pUSARTHandle->pRxBuffer += 2;
						pUSARTHandle->RxLen -= 2;
					} else {
						//Parity enabled, 8-bit data + parity bit
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 1;
					}
				} else {
					//8-bit frame
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) { //Check if parity is enabled
						//No parity
						//Read 8-bits
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR	& (uint8_t) 0xFF);
					}
					else {
						//Parity enabled, 7-bit data + parity bit
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR	& (uint8_t) 0x7F);
					}
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen -= 1;
				}
			}
			if (!pUSARTHandle->RxLen) {
				//Finish reception
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	/*************************Check for CTS flag ********************************************/
	//CTS feature is not applicable for UART4 and UART5
	//Check CTS
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);

	//Check CTSE
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

	//Check CTSIE bit (not available for UART4 & UART5)
	//temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if (temp1 && temp2) { //CTS & CTSE
		//Clear CTS flag
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		//Inform user application
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

	/*************************Check for IDLE detection flag ********************************************/
	//Check IDLE bit
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	//Check IDLEIE
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if (temp1 && temp2) {
		//Clear IDLE flag, refer to RM
		temp1 = pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		//Inform user application
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	/*************************Check for Overrun detection flag ********************************************/
	//Check ORE
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Check RXNEIE
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if (temp1 && temp2) {
		//Dont need to clear the ORE flag here, give an api for the application to clear the ORE flag
		//Inform application
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

	/*************************Check for Error Flag ********************************************/
	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);

	if (temp2) {
		temp1 = pUSARTHandle->pUSARTx->SR;
		if (temp1 & (1 << USART_SR_FE)) {
			/*
			 This bit is set by hardware when a de-synchronization, excessive noise or a break character
			 is detected. Cleared by software (read USART_SR register followed by a read to the USART_DR
			 register).
			 */
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}

		if (temp1 & (1 << USART_SR_NE)) {
			/*
			 This bit is set by hardware when noise is detected on a received frame. Cleared by software
			 (read USART_SR register followed by a read to the USART_DR register).
			 */
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
		}

		if (temp1 & (1 << USART_SR_ORE)) {
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_ApplicationEventCallback
 *
 * @brief             - Informs application about USART interrupts
 *
 * @param[in]         - USART structure that holds configuration and USART address
 * @param[in]         - Status to be reported
 *
 * @return            - none
 *
 * @Note              - Should be implemented in application
 */
__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event) {

}

