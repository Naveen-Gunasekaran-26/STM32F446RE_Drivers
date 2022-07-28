/*
 * 010I2C_Master_TX_Testing.c
 *
 *  Created on: 15-Jul-2022
 *      Author: gn260
 */

#include "stm32f446xx.h"
#include <string.h>

/*
 * PB6 -> SCL
 * PB9 (or) PB7 -> SDA
 */

// Data buffer to be sent to the slave
/*
 * Note: On the slave side, the Arduino sketch is written using the Arduino Wire library.
 * The wire library has limitations such that only 32 bytes can be transmitted or received in a
 * single I2C transaction.
 * If you have more than 32 bytes, make sure you split the bytes into multiple transactions
 */
uint8_t someData[] = "We are testing I2C Master TX\n";
#define MASTER_ADDRESS		0x61
#define SLAVE_ADDRESS 		0x68


I2C_Handle_t I2C1Handle;


void I2C1_GPIOInit(void){

	GPIO_Handle_t I2C1_Pins;

	I2C1_Pins.pGPIOx = GPIOB;
	I2C1_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C1_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_OD;
	I2C1_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2C1_Pins.GPIO_PinConfig.GPIO_PinAltFuncMode = 4;
	I2C1_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// SCL
	I2C1_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2C1_Pins);

	// SDA
	I2C1_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2C1_Pins);

}


void I2C1_Init(void){

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MASTER_ADDRESS;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
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

	// GPIO Button configuration and initialization
	GPIO_ButtonInit();

	// I2C1 pin configuration and initialization
	I2C1_GPIOInit();

	// I2C1 Peripheral configuration and initialization
	I2C1_Init();

	// Enable the I2C1 peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	while(1){

		// Wait till button press
		while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13));

		// Delay to avoid button de-bouncing issue
		delay();

		// Send data to the slave
		I2C_MasterSendData(&I2C1Handle, someData, strlen((char *)someData), SLAVE_ADDRESS);
	}
}
