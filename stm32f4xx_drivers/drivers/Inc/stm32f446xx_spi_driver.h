/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 18-Jun-2022
 *      Author: gn260
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_



#include "stm32f446xx.h"


// Configuration structure for SPIx peripheral
typedef struct{

	uint8_t SPI_DeviceMode;			// Refer @SPI_DeviceMode Macros section
	uint8_t SPI_BusConfig;			// Refer @SPI_BusConfig Macros section
	uint8_t SPI_SCLKSpeed;			// Refer @SPI_SCLKSpeed Macros section
	uint8_t SPI_DFF;				// Refer @SPI_DFF Macros section
	uint8_t SPI_CPOL;				// Refer @SPI_CPOL Macros section
	uint8_t SPI_CPHA;				// Refer @SPI_CPHA Macros section
	uint8_t SPI_SSM;				// Refer @SPI_SSM Macros section
}SPI_Config_t;


// Handle structure for SPIx peripheral
typedef struct{

	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;

}SPI_Handle_t;



// SPI Peripheral clock control
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


// SPI peripheral initialization and de-initialization
void SPI_Init(SPI_Handle_t *pSPIx);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


// Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataInterruptBased(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataInterruptBased(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


// IRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


// Other SPI Peripheral APIs
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pHandle);

// Application callback
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t ApplicationEvent);


// @SPI_DeviceMode possible values
#define SPI_DEV_MODE_SLAVE		0
#define SPI_DEV_MODE_MASTER		1


// @SPI_BusConfig possible values
#define SPI_BUS_CONFIG_FULL_DUP			0
#define SPI_BUS_CONFIG_HALF_DUP			1
#define SPI_BUS_CONFIG_SIMP_RXONLY		2


// @SPI_SCLKSpeed possible values
#define SPI_SCLK_SPEED_PRESCALER_2		0
#define SPI_SCLK_SPEED_PRESCALER_4		1
#define SPI_SCLK_SPEED_PRESCALER_8		2
#define SPI_SCLK_SPEED_PRESCALER_16		3
#define SPI_SCLK_SPEED_PRESCALER_32		4
#define SPI_SCLK_SPEED_PRESCALER_64		5
#define SPI_SCLK_SPEED_PRESCALER_128	6
#define SPI_SCLK_SPEED_PRESCALER_256	7


// @SPI_DFF possible values
#define SPI_DFF_8BIT		0
#define SPI_DFF_16BIT		1


// @SPI_CPOL possible values
#define SPI_CPOL_LOW		0
#define SPI_CPOL_HIGH		1

// @SPI_CPHA possible values
#define SPI_CPHA_LOW		0
#define SPI_CPHA_HIGH		1

// @SPI_SSM possible values
#define SPI_SSM_DI			0
#define SPI_SSM_EN			1


// @SPI_SR_FLAGS possible values
#define SPI_SR_RXNE_FLAG				(1 << SPI_SR_RXNE)
#define SPI_SR_TXE_FLAG					(1 << SPI_SR_TXE)
#define SPI_SR_CHSIDE_FLAG				(1 << SPI_SR_CHSIDE)
#define SPI_SR_UDR_FLAG					(1 << SPI_SR_UDR)
#define SPI_SR_CRC_ERR_FLAG				(1 << SPI_SR_CRC_ERR)
#define SPI_SR_MODF_FLAG				(1 << SPI_SR_MODF)
#define SPI_SR_OVR_FLAG					(1 << SPI_SR_OVR)
#define SPI_SR_BSY_FLAG					(1 << SPI_SR_BSY)
#define SPI_SR_FRE_FLAG					(1 << SPI_SR_FRE)


// Possible SPI application states
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

// Possible SPI application events
#define SPI_EVENT_TX_COMPLETE			1
#define SPI_EVENT_RX_COMPLETE			2
#define SPI_EVENT_OVR_ERROR				3
#define SPI_EVENT_CRC_ERROR				4


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
