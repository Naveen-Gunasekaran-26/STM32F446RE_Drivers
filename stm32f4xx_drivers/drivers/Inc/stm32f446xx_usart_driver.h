/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: 17-Jul-2022
 *      Author: gn260
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_


#include "stm32f446xx.h"

// Configuration structure for USARTx peripheral
typedef struct{

	uint8_t USART_Mode;
	uint32_t USART_Baudrate;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;

}USART_Config_t;


// Handle structure for USARTx peripheral
typedef struct{

	USART_RegDef_t *pUSARTx;
	USART_Config_t	USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;

}USART_Handle_t;


// USART Peripheral clock control
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);


// USART peripheral initialization and de-initialization
void USART_Init(USART_Handle_t *pUSARTx);
void USART_DeInit(USART_RegDef_t *pUSARTx);


// Data send and receive
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataInterruptBased(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataInterruptBased(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);


// IRQ Configuration and ISR Handling
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);


// Other USART Peripheral APIs
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

// Application callback
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApplicationEvent);


// USART possible mode of operations
#define USART_MODE_TX_ONLY			0
#define USART_MODE_RX_ONLY			1
#define USART_MODE_TX_RX			2

// USART possible baud-rates
#define USART_STD_BAUD_1200   		1200
#define USART_STD_BAUD_2400   		2400
#define USART_STD_BAUD_9600   		9600
#define USART_STD_BAUD_19200  		19200
#define USART_STD_BAUD_38400  		38400
#define USART_STD_BAUD_57600  		57600
#define USART_STD_BAUD_115200 		115200
#define USART_STD_BAUD_230400 		230400
#define USART_STD_BAUD_460800 		460800
#define USART_STD_BAUD_921600 		921600
#define USART_STD_BAUD_2M	    	2000000
#define USART_STD_BAUD_3M	    	3000000


// USART possible parity control
#define USART_PARITY_DISABLE				0
#define USART_PARITY_ENABLE_EVEN			1
#define USART_PARITY_ENABLE_ODD				2


// USART possible word length
#define USART_WORDLEN_8BITS			0
#define USART_WORDLEN_9BITS			1


// USART number of possible stop bits
#define USART_STOPBITS_1          	0
#define USART_STOPBITS_0_5        	1
#define USART_STOPBITS_2          	2
#define USART_STOPBITS_1_5        	3


// USART possible hardware flow control options
#define USART_HW_FLOW_CTRL_NONE					0
#define USART_HW_FLOW_CTRL_CTS					1
#define USART_HW_FLOW_CTRL_RTS					2
#define USART_HW_FLOW_CTRL_CTS_RTS				3


// USART SR register flags
#define USART_FLAG_PE		 	( 1 << USART_SR_PE )
#define USART_FLAG_FE		 	( 1 << USART_SR_FE )
#define USART_FLAG_NF		 	( 1 << USART_SR_NF )
#define USART_FLAG_ORE	 		( 1 << USART_SR_ORE	)
#define USART_FLAG_IDLE			( 1 << USART_SR_IDLE )
#define USART_FLAG_RXNE			( 1 << USART_SR_RXNE )
#define USART_FLAG_TC		 	( 1 << USART_SR_TC )
#define USART_FLAG_TXE	 		( 1 << USART_SR_TXE )
#define USART_FLAG_LBD	 		( 1 << USART_SR_LBD )
#define USART_FLAG_CTS	 		( 1 << USART_SR_CTS )


// USART Application states
#define USART_READY 			0
#define USART_BUSY_IN_TX 		1
#define USART_BUSY_IN_RX 		2

// USART Events and Errors
#define USART_EVENT_TX_CMPLT   		0
#define USART_EVENT_RX_CMPLT   		1
#define USART_EVENT_IDLE      		2
#define USART_EVENT_CTS       		3
#define USART_EVENT_PE        		4
#define USART_ERR_FE     			5
#define USART_ERR_NE    	 		6
#define USART_ERR_ORE    			7



#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
