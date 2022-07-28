/*
 * 005SPI_TXOnly_Arduino.c
 *
 *  Created on: 26-Jun-2022
 *      Author: gn260
 */



#include "stm32f446xx.h"


/* Goto your device datasheet stm32f446re.pdf -> Alternate function mapping for GPIO pins that can be
 * used for SPI peripheral
 *
 * AF5
 * PB12 - SPI2_NSS
 * PB13 - SPI2_SCK
 * PB14 - SPI2_MISO
 * PB15 - SPI2_MOSI
 *
 */


void SPI2_GPIOInit(void){

	GPIO_Handle_t SPI_Pins;
	SPI_Pins.pGPIOx = GPIOB;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MED;

	// SPI2_SCK - PB13
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPI_Pins);

	// SPI2_MOSI - PB15
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI_Pins);

	// SPI2_MISO - PB14
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPI_Pins);

	// SPI2_NSS - PB12
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPI_Pins);

}


void SPI2_Init(void){

	SPI_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUP;
	SPI2_Handle.SPIConfig.SPI_DeviceMode = SPI_DEV_MODE_MASTER;
	SPI2_Handle.SPIConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_PRESCALER_8;
	SPI2_Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Disabled SMM for Hardware slave management

	SPI_Init(&SPI2_Handle);
}

void GPIO_ButtonInit(void){

	GPIO_Handle_t GPIO_Button;

	GPIO_Button.pGPIOx = GPIOC;
	GPIO_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MED;
//	GPIO_Button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	GPIO_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_Init(&GPIO_Button);
}


void delay(void){

	for(uint32_t i=0; i<500000; i++);
}


int main(void){

	SPI2_GPIOInit();
	SPI2_Init();
	GPIO_ButtonInit();

	/*
	 * Making SSOE 1 does NSS output enable
	 * The NSS pin managed automatically by the hardware
	 * i.e When SPE=1, NSS will be pulled low
	 * When SPE=0, NSS will be pulled high
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){

		while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));
		delay();

		SPI_PeripheralControl(SPI2, ENABLE);


		char data[] = "Hello world!";
		uint8_t dataLength = sizeof(data)/sizeof(data[0]);

		// Arduino sketch expects 1 byte of length information followed by datastream
		// So first send length information
		SPI_SendData(SPI2, &dataLength, sizeof(dataLength));

		// Sending datastream
		SPI_SendData(SPI2, (uint8_t *)data, dataLength);
		// SPI_SendData(SPI2, (uint8_t *)data, strlen(data));

		while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY)); // Wait till SPI comes out of busy state
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
