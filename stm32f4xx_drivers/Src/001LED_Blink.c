/*
 * 001LED_Toggle.c
 *
 *  Created on: Jun 11, 2022
 *      Author: gn260
 */


#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

void delay(void){

	for(uint32_t i=0; i<500000; i++);
}

int main(void){

	GPIO_Handle_t GPIO_Led;

	GPIO_Led.pGPIOx = GPIOA;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MED;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIO_Led.pGPIOx, ENABLE);
	GPIO_Init(&GPIO_Led);

	while(1){
		GPIO_ToggleOutputPin(GPIO_Led.pGPIOx, GPIO_PIN_5);
		delay();
	}


	return 0;
}
