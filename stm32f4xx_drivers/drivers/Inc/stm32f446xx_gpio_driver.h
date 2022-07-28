/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jun 8, 2022
 *      Author: gn260
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

typedef struct{

	uint8_t GPIO_PinNumber;			// Possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// Possible values from @GPIO_POSSIBLE_PIN_MODES
	uint8_t GPIO_PinSpeed;			// Possible values from @GPIO_POSSIBLE_PIN_SPEEDS
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig_t;


// Handle structure for a GPIO pin
typedef struct{

	GPIO_RegDef_t *pGPIOx; //This holds the base address of the GPIO Port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; //This holds the GPIO pin configuration settings
}GPIO_Handle_t;


// @GPIO_PIN_NUMBERS
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15


// @GPIO_POSSIBLE_PIN_MODES
// GPIO pin possible modes
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IP_FET		4
#define GPIO_MODE_IP_RET		5
#define GPIO_MODE_IP_FRET		6

// GPIO pin possible output types
#define GPIO_OPTYPE_PP			0		// Push-Pull
#define GPIO_OPTYPE_OD			1		// Open Drain

// @GPIO_POSSIBLE_PIN_SPEEDS
// GPIO pin possible output speeds
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MED			1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VHIGH		3

// GPIO pin pullup and pulldown configuration
#define GPIO_PIN_NO_PUPD			0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD					2


// API supported by this driver
// For more information about the API check the function definitions

// Peripheral Clock control
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);


// Init and Deinit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


// Reading and Writing
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


// IRQ Configuration and ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
