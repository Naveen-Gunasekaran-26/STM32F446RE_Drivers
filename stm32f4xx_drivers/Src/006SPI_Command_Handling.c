/*
 * 006SPI_Command_Handling.c
 *
 *  Created on: 26-Jun-2022
 *      Author: gn260
 */


#include <stdio.h>
#include <string.h>

#include "stm32f446xx.h"


// Included for using the semi-hosting feature with printf
extern void initialise_monitor_handles();



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


// Command codes for SPI communication b/w STM32 and Arduino
#define COMMAND_LED_CTRL			0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ			0x52
#define COMMAND_PRINT				0x53
#define COMMAND_ID_READ				0x54

#define LED_ON						1
#define LED_OFF						0
#define LED_PINNUM_ON_ARDUINO		9

// Arduino analog pins
#define ANALOG_PIN0					0
#define ANALOG_PIN1					1
#define ANALOG_PIN2					2
#define ANALOG_PIN3					3
#define ANALOG_PIN4					4



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


uint8_t SPI_VerifyResponse(uint8_t AckByte){

	if(AckByte == 0xF5)
		return 1;
	return 0;
}


int main(void){

	// Included for using the semi-hosting feature with printf
	initialise_monitor_handles();
	printf("Application started running.\n");

	SPI2_GPIOInit();
	SPI2_Init();
	GPIO_ButtonInit();

	printf("SPI initialized.\n");

	/*
	 * Making SSOE 1 does NSS output enable
	 * The NSS pin managed automatically by the hardware
	 * i.e When SPE=1, NSS will be pulled low
	 * When SPE=0, NSS will be pulled high
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){

		uint8_t AckByte, Args[2], DummyReadByte;


		// 1) COMMAND_LED_CTRL		<pin num>	<value>
		while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));
		printf("COMMAND_LED_CTRL Buttonpress 1\n");
		delay();

		SPI_PeripheralControl(SPI2, ENABLE);

		SPI_SendData(SPI2, (uint8_t *)COMMAND_LED_CTRL, sizeof(uint8_t));

		/*
		 * In SPI communuication, when master or slave sends 1-byte,
		 * it also receives 1 byte in return. The transmission of 1-byte
		 * for sending the command COMMAND_LED_CTRL above will result in
		 * 1 garbage byte collection in Rx buffer of the master and
		 * RXNE flag is set. So, we should a dummy read inorder to clear the RXNE flag
		 */
		SPI_ReceiveData(SPI2, &DummyReadByte, sizeof(DummyReadByte));


		/*
		 * After sending the data, the slave(Arduino) should send back ACK
		 * But in SPI communication, a master always has to initiate the communication
		 * A slave cannot initiate the communication
		 * So, inorder to get response from the slave, we have to send a dummy byte from the master
		 */

		// Sending 1 byte of dummy data since we are using 8bit DFF; Use 2bytes for 16bit DFF
		SPI_SendData(SPI2, (uint8_t *)0xff, sizeof(uint8_t));
		SPI_ReceiveData(SPI2, &AckByte, sizeof(uint8_t));

		if(SPI_VerifyResponse(AckByte)){

			// If received ACK, then send the arguments for the command to the slave
			Args[0] = LED_PINNUM_ON_ARDUINO;
			Args[1] = LED_ON;
			SPI_SendData(SPI2, Args, sizeof(Args)/sizeof(Args[0]));
			printf("COMMAND_LED_CTRL executed\n");
		}

		// 2) COMMAND_SENSOR_READ
		while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));
		printf("COMMAND_SENSOR_READ Buttonpress 2\n");
		delay();

		SPI_SendData(SPI2, (uint8_t *)COMMAND_SENSOR_READ, sizeof(uint8_t));
		SPI_ReceiveData(SPI2, &DummyReadByte, sizeof(DummyReadByte));
		SPI_SendData(SPI2, (uint8_t *)0xff, sizeof(uint8_t));
		SPI_ReceiveData(SPI2, &AckByte, sizeof(uint8_t));

		if(SPI_VerifyResponse(AckByte)){

			// If received ACK, then send the arguments for the command to the slave
			Args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, Args, sizeof(Args[0]));
			SPI_ReceiveData(SPI2, &DummyReadByte, sizeof(DummyReadByte));

			/*
			 * Slave actually takes sometime to read the analog value (Slave has to do ADC conversion
			 * on that pin). So, master should wait for sometime before sending the dummy byte to
			 * fetch the result.
			 */
			delay(); // Delay inserted for SPI to wait for ADC conversion by slave
			SPI_SendData(SPI2, (uint8_t *)0xff, sizeof(uint8_t));

			uint8_t AnalogPinReading;
			SPI_ReceiveData(SPI2, &AnalogPinReading, sizeof(uint8_t));
			printf("COMMAND_SENSOR_READ %d\n", AnalogPinReading);
		}



		// 3) COMMAND_LED_READ
		while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));
		printf("COMMAND_LED_READ Buttonpress 3\n");
		delay();
		SPI_SendData(SPI2, (uint8_t *)COMMAND_LED_READ, sizeof(uint8_t));
		SPI_ReceiveData(SPI2, &DummyReadByte, sizeof(DummyReadByte));
		SPI_SendData(SPI2, (uint8_t *)0xff, sizeof(uint8_t));
		SPI_ReceiveData(SPI2, &AckByte, sizeof(AckByte));

		if(SPI_VerifyResponse(AckByte)){

			Args[0] = LED_PINNUM_ON_ARDUINO;

			SPI_SendData(SPI2, Args, sizeof(Args[0]));
			SPI_ReceiveData(SPI2, &DummyReadByte, sizeof(DummyReadByte));
			delay();
			SPI_SendData(SPI2, (uint8_t *)0xff, sizeof(uint8_t));

			uint8_t LedStateReading;
			SPI_ReceiveData(SPI2, &LedStateReading, sizeof(uint8_t));
			printf("COMMAND_LED_READ %d\n", LedStateReading);
		}

		// 4) COMMAND_PRINT
		while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));
		printf("COMMAND_PRINT Buttonpress 4\n");
		delay();
		SPI_SendData(SPI2, (uint8_t *)COMMAND_PRINT, sizeof(uint8_t));
		SPI_ReceiveData(SPI2, &DummyReadByte, sizeof(DummyReadByte));
		SPI_SendData(SPI2, (uint8_t *)0xff, sizeof(uint8_t));
		SPI_ReceiveData(SPI2, &AckByte, sizeof(AckByte));

		if(SPI_VerifyResponse(AckByte)){

			uint8_t message[] = "Hello! How are you?";
			Args[0] = strlen((char *)message);

			SPI_SendData(SPI2, Args, sizeof(Args[0]));
			SPI_ReceiveData(SPI2, &DummyReadByte, sizeof(DummyReadByte));
			delay();

			for(uint8_t i=0; i<Args[0]; i++){

				SPI_SendData(SPI2, &message[i], sizeof(uint8_t));
				SPI_ReceiveData(SPI2, &DummyReadByte, sizeof(DummyReadByte));
			}

			printf("COMMAND_PRINT Executed\n");
		}


		// 5) COMMAND_ID_READ
		while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));
		printf("COMMAND_ID_READ Buttonpress 5\n");
		delay();
		SPI_SendData(SPI2, (uint8_t *)COMMAND_ID_READ, sizeof(uint8_t));
		SPI_ReceiveData(SPI2, &DummyReadByte, sizeof(DummyReadByte));
		SPI_SendData(SPI2, (uint8_t *)0xff, sizeof(uint8_t));
		SPI_ReceiveData(SPI2, &AckByte, sizeof(AckByte));

		if(SPI_VerifyResponse(AckByte)){

			uint8_t ID[11];

			for(uint8_t i=0; i<10; i++){

				SPI_SendData(SPI2, (uint8_t *)0xff, sizeof(uint8_t));
				SPI_ReceiveData(SPI2, &ID[i], sizeof(uint8_t));
			}
			ID[10] = '\0';

			printf("COMMAND_ID %s\n", ID);
		}


		while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY)); // Wait till SPI comes out of busy state
		SPI_PeripheralControl(SPI2, DISABLE);
		printf("SPI communication closed.\n");
	}

	return 0;
}
