/*
 * 003Button_Interrupt.c
 *
 *  Created on: Jun 13, 2022
 *      Author: gn260
 */


#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"


#define BTN_PRESSED DISABLE


void delay(void){

	for(uint32_t i=0; i<500000; i++);
}


int main(void){

	GPIO_Handle_t GPIO_Led, GPIO_Button;

	GPIO_Led.pGPIOx = GPIOA;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MED;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIO_Led.pGPIOx, ENABLE);
	GPIO_Init(&GPIO_Led);

	GPIO_Button.pGPIOx = GPIOC;
	GPIO_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IP_FET;
	GPIO_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MED;
//	GPIO_Button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GPIO_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIO_Button.pGPIOx, ENABLE);
	GPIO_Init(&GPIO_Button);


	// IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI15_10, NVIC_IRQ_PRIORITY_NUM_15);
	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI15_10, ENABLE);

	while(1);

	return 0;
}


void EXTI15_10_IRQHandler(void){

	GPIO_IRQHandling(GPIO_PIN_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
}
