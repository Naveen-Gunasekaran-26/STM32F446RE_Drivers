/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 18-Jun-2022
 *      Author: gn260
 */


#include "stm32f446xx.h"


/*
 * The interrupt handle function prototypes are defined in the source file and not in the header file
 * because these are not meant to be used by the user application
 */
static void SPI_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_interrupt_handle(SPI_Handle_t *pSPIHandle);


// SPI Peripheral clock control
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

		if(pSPIx == SPI1)
			SPI1_PCLCK_EN();
		else if(pSPIx == SPI2)
			SPI2_PCLCK_EN();
		else if(pSPIx == SPI3)
			SPI3_PCLCK_EN();
		else if(pSPIx == SPI4)
			SPI4_PCLCK_EN();
	}
	else{

		if(pSPIx == SPI1)
			SPI1_PCLCK_DI();
		else if(pSPIx == SPI2)
			SPI2_PCLCK_DI();
		else if(pSPIx == SPI3)
			SPI3_PCLCK_DI();
		else if(pSPIx == SPI4)
			SPI4_PCLCK_DI();
	}
}


// SPI peripheral initialization and de-initialization
void SPI_Init(SPI_Handle_t *pSPIHandle){

	// Enable peripheral clock for the SPI
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 1) Configure the device mode
	pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2) Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUP){

		// BIDI mode should be cleared
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUP){

		// BIDI mode should be set
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMP_RXONLY){

		// BIDI mode should be cleared
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDI_MODE);

		// RXONLY bit should be set
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_RXONLY);

	}

	// 3) Configure the SPI serial clock speed (Baud rate)
	pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_SCLKSpeed << SPI_CR1_BR;

	// 4) Configure the DFF
	pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5) Configure the CPOL
	pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6) Configure the CPHA
	pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

}


void SPI_DeInit(SPI_RegDef_t *pSPIx){

	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
	else if(pSPIx == SPI3)
		SPI3_REG_RESET();
	else if(pSPIx == SPI4)
		SPI4_REG_RESET();
}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){

	if(pSPIx->SR & FlagName)
		return FLAG_SET;
	return FLAG_UNSET;
}

// Data send and receive (Blocking)
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){

		// 1) Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_TXE_FLAG) == FLAG_UNSET);

		// 2) Check the DFF bit in CR1 register
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){

			// 16-bit DFF
			// 1) Load the data into the DR
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len-=2;		// Since 16 bytes of data is sent
			(uint16_t *)pTxBuffer++;
		}
		else{

			// 8-bit DFF
			// 1) Load the data into the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

	while(Len > 0){

		// 1) Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE_FLAG) == FLAG_UNSET);

		// 2) Check the DFF bit in CR1 register
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){

			// 16-bit DFF
			// 1) Load the data from DR into RxBuffer
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			Len-=2;		// Since 16 bytes of data is sent
			(uint16_t *)pRxBuffer++;
		}
		else{

			// 8-bit DFF
			// 1) Load the data from DR into RxBuffer
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}


// Data send and receive interrupt based (Non-blocking)
uint8_t SPI_SendDataInterruptBased(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t currentTxState = pSPIHandle->TxState;
	if(currentTxState != SPI_BUSY_IN_TX){

		// 1) Save the Tx buffer address and length in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// 2) Mark the SPI state as busy during transmission so that
		//    no other node can take over same SPI peripheral until the transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3) Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// 4) Data transmission will be handled by the ISR code (will implement later)
	}

	return currentTxState;
}


uint8_t SPI_ReceiveDataInterruptBased(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){

	uint8_t currentRxState = pSPIHandle->RxState;
	if(currentRxState != SPI_BUSY_IN_RX){

		// 1) Save the Rx buffer address and length in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// 2) Mark the SPI state as busy during reception so that
		//    no other node can take over same SPI peripheral until the reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3) Enable the TXEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4) Data reception will be handled by the ISR code (will implement later)
	}

	return currentRxState;
}


// IRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

		if(IRQNumber < 32){

			// Set ISER0 Register in Processor
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 32 && IRQNumber < 64){

			// Set ISER1 Register in Processor
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){

			// Set ISER2 Register in Processor
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}

	}
	else{

		if(IRQNumber < 32){

			// Set ICER0 Register in Processor
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 32 && IRQNumber < 64){

			// Set ICER1 Register in Processor
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){

			// Set ICER2 Register in Processor
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t IprxRegNum 			= IRQNumber / 4;
	uint8_t IprxRegSectionNum 	= IRQNumber % 4;

	*(NVIC_IPR_BASEADDR + IprxRegNum) |= IRQPriority << ((8 * IprxRegSectionNum) + NUM_OF_NON_IMPLMNTD_LOW_ORD_BITS_IN_PRIOR_REG);
}


void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

	// Check for TXE
	// TXEBitState && TXEIEBitState
	if( (pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE)) && (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE)) ){

		// Handle TXE
		SPI_TXE_interrupt_handle(pSPIHandle);
	}

	// Check for RXNE
	// RXNEBitState && RXNEIEBitState
	if( (pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE)) && (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE)) ){

		// Handle TXE
		SPI_RXNE_interrupt_handle(pSPIHandle);
	}

	// Check for OVR flag
	// OVRBitState && ERRIEBitState
	if( (pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR)) && (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE)) ){

		// Handle TXE
		SPI_OVR_interrupt_handle(pSPIHandle);
	}
}


// Other SPI Peripheral APIs
// SPI PeripheralControl to Enable/Disable the peripheral
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE)
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);

	pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}


// SPI SSI Config to Enable/Disable the SSI
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE)
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);

	pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
}


// SPI SSOE Config to Enable/Disable the SSOE bit (SlaveSelectOutputEnable)
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE)
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);

	pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){

	// Simply reading DR and SR in-order to clear the flag
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;

	// This typecasting is done since temp is not used anywhere and
	// to prevent any compiler optimization
	(void)temp;
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


static void SPI_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle){


	// Check the DFF bit in CR1 register
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){

		// 16-bit DFF
		// 1) Load the data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen-=2;		// Since 16 bytes of data is sent
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}
	else{

		// 8-bit DFF
		// 1) Load the data into the DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(pSPIHandle->TxLen == 0){

		// If TxLen = 0, close the SPI transmission and
		// inform the application that transmission is over
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
	}
}


static void SPI_RXNE_interrupt_handle(SPI_Handle_t *pSPIHandle){

	// Check the DFF bit in CR1 register
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){

		// 16-bit DFF
		// 1) Load the data from DR into RxBuffer
		*((uint16_t *)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen-=2;		// Since 16 bytes of data is sent
		(uint16_t *)pSPIHandle->pRxBuffer++;
	}
	else{

		// 8-bit DFF
		// 1) Load the data from DR into RxBuffer
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(pSPIHandle->RxLen == 0){

		// If RxLen = 0, close the SPI reception and
		// inform the application that reception is over
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}

}


static void SPI_OVR_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;

	// 1) Clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){

		// Simply reading DR and SR in-order to clear the flag
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	// This typecasting is done since temp is not used anywhere and
	// to prevent any compiler optimization
	(void)temp;

	// 2) Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERROR);

}


// This is a weak implementation of the SPI_ApplicationEventCallback in order to prevent compilation errors.
// The application may override this weak implementation.
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t ApplicationEvent){


	// The gcc attribute '__attribute__((weak))' specifies the compiler that
	// this function is a weak implementation

}
