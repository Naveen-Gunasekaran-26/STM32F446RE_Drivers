/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Jun 8, 2022
 *      Author: gn260
 */

#include "stm32f446xx_gpio_driver.h"


// API supported by this driver

// Peripheral Clock control
/***********************************************************
 * @fn			-	GPIO_PeriClockControl
 *
 * @brief		-	Enables or disables the peripheral clock for the given GPIO port
 *
 * @param[in]	-	Base address of the GPIO peripheral
 * @param[in]	-	ENABLE or DISABLE macro
 * @param[in]	-
 *
 * @return		-	none
 *
 * @Note		-	none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

		if(pGPIOx == GPIOA)
			GPIOA_PCLCK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLCK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLCK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLCK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLCK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLCK_EN();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLCK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLCK_EN();
	}
	else{

		if(pGPIOx == GPIOA)
			GPIOA_PCLCK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLCK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLCK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLCK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLCK_DI();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLCK_DI();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLCK_DI();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLCK_DI();
	}
}


// Init and Deinit
/***********************************************************
 * @fn			-
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){


	// Enable peripheral clock for the GPIO
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//	1.Configure GPIO pin mode

	//	The <=3 check here means modes not involving interrupts
	//  If >3 means modes involving interrupts
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= 3){

		//	Non-interrupt modes
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing bits if it has been set previously
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Setting bits
	}
	else{

		// Interrupt modes
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IP_FET){

			// 1.Configure the FTSR FallingTriggeredSelectionRegister
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing RTSR if it has been set previously
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IP_RET){

			// 1.Configure the RTSR RisingTriggeredSelectionRegister
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing FTSR if it has been set previously
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IP_FRET){

			// 1.Configure both FTSR & RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// 2.Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t Syscfg_Exticr_Reg_Num = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t Syscfg_Exticr_Reg_Sec_Num = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t Gpio_Port_Code = GPIO_BASEADDR_TO_PORTCODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLCK_EN();
		SYSCFG->EXTICR[Syscfg_Exticr_Reg_Num] |= Gpio_Port_Code << (4 * Syscfg_Exticr_Reg_Sec_Num);


		// 3.Enable EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

//	2.Configure GPIO pin speed
	pGPIOHandle->pGPIOx->SPEEDR &= ~(0x3 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing bits
	pGPIOHandle->pGPIOx->SPEEDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Setting bits


//	3.Configure PULLUP/PULLDOWN
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing bits
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Setting bits


//	4.Configure Output type
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT){

		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing bits
		pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Setting bits
	}

//	5.Configure alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		uint32_t AltFunRegLowOrHigh, AltFunRegBitsForPin;

		AltFunRegLowOrHigh 	= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		AltFunRegBitsForPin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[AltFunRegLowOrHigh] &= ~(0xF << (4 * AltFunRegBitsForPin)); //Clearing bits
		pGPIOHandle->pGPIOx->AFR[AltFunRegLowOrHigh] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * AltFunRegBitsForPin)); //Setting bits
	}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if(pGPIOx == GPIOF)
		GPIOF_REG_RESET();
	else if(pGPIOx == GPIOG)
		GPIOG_REG_RESET();
	else if(pGPIOx == GPIOH)
		GPIOH_REG_RESET();
}


// Reading and Writing

/***********************************************************
 * @fn			-
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	return (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
}




/***********************************************************
 * @fn			-
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	return (uint16_t) (pGPIOx->IDR);
}





/***********************************************************
 * @fn			-
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET){

		// Write 1 to the ODR at the bitfield corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{

		// Write 0 to the ODR at the bitfield corresponding to the pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}




/***********************************************************
 * @fn			-
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR = Value;
}



/***********************************************************
 * @fn			-
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);
}


// IRQ Configuration and ISR Handling
/***********************************************************
 * @fn			-
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){

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



/***********************************************************
 * @fn			-
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

	uint8_t IprxRegNum 			= IRQNumber / 4;
	uint8_t IprxRegSectionNum 	= IRQNumber % 4;

	*(NVIC_IPR_BASEADDR + IprxRegNum) |= IRQPriority << ((8 * IprxRegSectionNum) + NUM_OF_NON_IMPLMNTD_LOW_ORD_BITS_IN_PRIOR_REG);
}


/***********************************************************
 * @fn			-
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 */
void GPIO_IRQHandling(uint8_t PinNumber){

	// Clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)){

		EXTI->PR |= (1 << PinNumber);
	}
}
