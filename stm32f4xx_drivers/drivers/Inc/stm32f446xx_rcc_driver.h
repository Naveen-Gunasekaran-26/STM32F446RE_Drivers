/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: 17-Jul-2022
 *      Author: gn260
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_


#include "stm32f446xx.h"

uint32_t RCC_GetPLLOutputClock(void);

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);




#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
