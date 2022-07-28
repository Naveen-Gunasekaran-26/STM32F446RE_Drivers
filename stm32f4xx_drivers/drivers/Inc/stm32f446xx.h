/*
 * stm32f446xx.h
 *
 *  Created on: May 29, 2022
 *      Author: gn260
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#include <stddef.h>


// Processor specific details
// ARM Cortex M4 Processor NVIC ISERx register addresses
#define NVIC_ISER0			((volatile uint32_t *)0xE000E100U)
#define NVIC_ISER1			((volatile uint32_t *)0xE000E104U)
#define NVIC_ISER2			((volatile uint32_t *)0xE000E108U)
#define NVIC_ISER3			((volatile uint32_t *)0xE000E10CU)
#define NVIC_ISER4			((volatile uint32_t *)0xE000E110U)
#define NVIC_ISER5			((volatile uint32_t *)0xE000E114U)
#define NVIC_ISER6			((volatile uint32_t *)0xE000E118U)
#define NVIC_ISER7			((volatile uint32_t *)0xE000E11CU)


// ARM Cortex M4 Processor NVIC ICERx register addresses
#define NVIC_ICER0			((volatile uint32_t *)0xE000E180U)
#define NVIC_ICER1			((volatile uint32_t *)0xE000E184U)
#define NVIC_ICER2			((volatile uint32_t *)0xE000E188U)
#define NVIC_ICER3			((volatile uint32_t *)0xE000E18CU)
#define NVIC_ICER4			((volatile uint32_t *)0xE000E190U)
#define NVIC_ICER5			((volatile uint32_t *)0xE000E194U)
#define NVIC_ICER6			((volatile uint32_t *)0xE000E198U)
#define NVIC_ICER7			((volatile uint32_t *)0xE000E19CU)


// ARM Cortex M4 Processor Priority register addresses
#define NVIC_IPR_BASEADDR 			((volatile uint32_t *)0xE000E400U)


// ARM Cortex M4 Processor - Number of non-implemented lower order bits in priority register
#define NUM_OF_NON_IMPLMNTD_LOW_ORD_BITS_IN_PRIOR_REG 		4


// ARM Cortex M4 Processor - IRQ priority number macros
#define NVIC_IRQ_PRIORITY_NUM_0			0
#define NVIC_IRQ_PRIORITY_NUM_1			1
#define NVIC_IRQ_PRIORITY_NUM_2			2
#define NVIC_IRQ_PRIORITY_NUM_3			3
#define NVIC_IRQ_PRIORITY_NUM_4			4
#define NVIC_IRQ_PRIORITY_NUM_5			5
#define NVIC_IRQ_PRIORITY_NUM_6			6
#define NVIC_IRQ_PRIORITY_NUM_7			7
#define NVIC_IRQ_PRIORITY_NUM_8			8
#define NVIC_IRQ_PRIORITY_NUM_9			9
#define NVIC_IRQ_PRIORITY_NUM_10		10
#define NVIC_IRQ_PRIORITY_NUM_11		11
#define NVIC_IRQ_PRIORITY_NUM_12		12
#define NVIC_IRQ_PRIORITY_NUM_13		13
#define NVIC_IRQ_PRIORITY_NUM_14		14
#define NVIC_IRQ_PRIORITY_NUM_15		15


//Base addresses of Flash, SRAM, ROM memory
#define FLASH_BASEADDR 		0x08000000U
#define SRAM1_BASEADDR 		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define SRAM_BASEADDR		SRAM1_BASEADDR
#define ROM_BASEADDR		0x1FFF0000U		/*System memory base address*/


//Base addresses of Peripheral buses
#define PERIPH_BASEADDR		0x40000000U
#define APB1_BASEADDR		PERIPH_BASEADDR
#define APB2_BASEADDR		0x40010000U
#define AHB1_BASEADDR		0x40020000U
#define AHB2_BASEADDR		0x50000000U
#define AHB3_BASEADDR		0xA0000000U


//Base addresses of each peripherals hanging on AHB1 bus
#define GPIOA_BASEADDR      	(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR      	(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR      	(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR      	(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR      	(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR      	(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR      	(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1_BASEADDR + 0x1C00)

#define USB_OTG_HS_BASEADDR      (AHB1_BASEADDR + 0x0000)
#define DMA2_BASEADDR            (AHB1_BASEADDR + 0x6400)
#define DMA1_BASEADDR            (AHB1_BASEADDR + 0x6000)
#define BKPSRAM_BASEADDR         (AHB1_BASEADDR + 0x4000)
#define FLASH_INTF_BASEADDR		 (AHB1_BASEADDR + 0x3C00)
#define RCC_BASEADDR             (AHB1_BASEADDR + 0x3800)
#define CRC_BASEADDR             (AHB1_BASEADDR + 0x3000)


// Base addresses of peripherals hanging on AHB2 bus
#define USB_OTG_FS_BASEADDR		(AHB2_BASEADDR + 0x0000)
#define DCMI_BASEADDR			(AHB2_BASEADDR + 0x50000)


// Base addresses of peripherals hanging on AHB2 bus
#define FMC_CTRL_REG 				(AHB3_BASEADDR + 0x0000)
#define QUADSPI_REG  				(AHB3_BASEADDR + 0x1000)


// Base addresses of peripherals hanging on APB1 bus
#define DAC_BASEADDR                     (APB1_BASEADDR + 0x7400)
#define PWR_BASEADDR                     (APB1_BASEADDR + 0x7000)
#define HDMI_OR_CEC_BASEADDR             (APB1_BASEADDR + 0x6C00)
#define CAN2_BASEADDR                    (APB1_BASEADDR + 0x6800)
#define CAN1_BASEADDR                    (APB1_BASEADDR + 0x6400)
#define I2C3_BASEADDR                    (APB1_BASEADDR + 0x5C00)
#define I2C2_BASEADDR                    (APB1_BASEADDR + 0x5800)
#define I2C1_BASEADDR                    (APB1_BASEADDR + 0x5400)
#define UART5_BASEADDR                   (APB1_BASEADDR + 0x5000)
#define UART4_BASEADDR                   (APB1_BASEADDR + 0x4C00)
#define USART3_BASEADDR                  (APB1_BASEADDR + 0x4800)
#define USART2_BASEADDR                  (APB1_BASEADDR + 0x4400)
#define SPDIF_OR_RX_BASEADDR             (APB1_BASEADDR + 0x4000)
#define SPI3_OR_I2S3_BASEADDR            (APB1_BASEADDR + 0x3C00)
#define SPI2_OR_I2S2_BASEADDR            (APB1_BASEADDR + 0x3800)
#define IWDG_BASEADDR                    (APB1_BASEADDR + 0x3000)
#define WWDG_BASEADDR                    (APB1_BASEADDR + 0x2C00)
#define RTC_AND_BKP_BASEADDR             (APB1_BASEADDR + 0x2800)
#define TIM14_BASEADDR                   (APB1_BASEADDR + 0x2000)
#define TIM13_BASEADDR                   (APB1_BASEADDR + 0x1C00)
#define TIM12_BASEADDR                   (APB1_BASEADDR + 0x1800)
#define TIM7_BASEADDR                    (APB1_BASEADDR + 0x1400)
#define TIM6_BASEADDR                    (APB1_BASEADDR + 0x1000)
#define TIM5_BASEADDR                    (APB1_BASEADDR + 0x0C00)
#define TIM4_BASEADDR                    (APB1_BASEADDR + 0x0800)
#define TIM3_BASEADDR                    (APB1_BASEADDR + 0x0400)
#define TIM2_BASEADDR                    (APB1_BASEADDR + 0x0000)


// Base addresses of peripherals hanging on APB2 bus
#define SAI2_BASEADDR                     (APB2_BASEADDR + 0x5C00)
#define SAI1_BASEADDR                     (APB2_BASEADDR + 0x5800)
#define TIM11_BASEADDR                    (APB2_BASEADDR + 0x4800)
#define TIM10_BASEADDR                    (APB2_BASEADDR + 0x4400)
#define TIM9_BASEADDR                     (APB2_BASEADDR + 0x4000)
#define EXTI_BASEADDR                     (APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR                   (APB2_BASEADDR + 0x3800)
#define SPI4_BASEADDR                     (APB2_BASEADDR + 0x3400)
#define SPI1_BASEADDR                     (APB2_BASEADDR + 0x3000)
#define SDMMC_BASEADDR                    (APB2_BASEADDR + 0x2C00)
#define ADC1_OR_ADC2_OR_ADC3_BASEADDR     (APB2_BASEADDR + 0x2000)
#define USART6_BASEADDR                   (APB2_BASEADDR + 0x1400)
#define USART1_BASEADDR                   (APB2_BASEADDR + 0x1000)
#define TIM8_BASEADDR                     (APB2_BASEADDR + 0x0400)
#define TIM1_BASEADDR                     (APB2_BASEADDR + 0x0000)



// Structure definition for GPIO registers
// Refer GPIO memory mapping in RM
typedef struct{
	volatile uint32_t MODER;						// GPIO port mode register
	volatile uint32_t OTYPER;						// GPIO port output type register
	volatile uint32_t SPEEDR;						// GPIO port output speed register
	volatile uint32_t PUPDR;						// GPIO port pull-up/pull-down register
	volatile uint32_t IDR;							// GPIO port input data register
	volatile uint32_t ODR;							// GPIO port output data register
	volatile uint32_t BSRR;							// GPIO port bit set/reset register
	volatile uint32_t LCKR;							// GPIO port configuration lock register
	volatile uint32_t AFR[2];						// AFR[0] -> GPIO alternate function low register; AFR[1] -> alternate function high register
}GPIO_RegDef_t;



// Structure definition for RCC registers
// Refer RCC memory mapping in RM
typedef struct{
	volatile uint32_t CR;
	volatile uint32_t PLL_CFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1_RSTR;
	volatile uint32_t AHB2_RSTR;
	volatile uint32_t AHB3_RSTR;
	uint32_t RESERVED_0;
	volatile uint32_t APB1_RSTR;
	volatile uint32_t APB2_RSTR;
	uint32_t RESERVED_1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED_2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED_3[2];
	volatile uint32_t AHB1_LPENR;
	volatile uint32_t AHB2_LPENR;
	volatile uint32_t AHB3_LPENR;
	uint32_t RESERVED_4;
	volatile uint32_t APB1_LPENR;
	volatile uint32_t APB2_LPENR;
	uint32_t RESERVED_5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED_6[2];
	volatile uint32_t SS_CGR;
	volatile uint32_t PLLI2_SCFGR;
	volatile uint32_t PLL_SAI_CFGR;
	volatile uint32_t DCK_CFGR;
	volatile uint32_t CK_GATENR;
	volatile uint32_t DCK_CFGR2;

}RCC_RegDef_t;



// Structure definition for EXTI registers
// Refer EXTI memory mapping in RM
typedef struct{

	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

}EXTI_RegDef_t;



// Structure definition for SYSCFG register
// Refer SYSCFG memory mapping in RM
typedef struct{

	//	SYSCFG_BASEADDR
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED_0[2];
	volatile uint32_t CMPCR;
	uint32_t RESERVED_1[2];
	volatile uint32_t CFGR;

}SYSCFG_RegDef_t;



// Structure definition for SPI Peripheral registers
// Refer SPI memory mapping in RM
typedef struct{

	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;

}SPI_RegDef_t;

typedef struct{

	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;

}I2C_RegDef_t;


typedef struct{

	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;

}USART_RegDef_t;



// EXTI - EXTI_RegDef_t* pointer macro pointing to EXTI_BASEADDR
#define EXTI			  	((EXTI_RegDef_t*)EXTI_BASEADDR)

// SYSCFG - SYSCFG_RegDef_t* pointer macro pointing to SYSCFG_BASEADDR
#define SYSCFG			  	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

// RCC - RCC_RegDef_t* pointer macro pointing to RCC_BASEADDR
#define RCC			  		((RCC_RegDef_t*)RCC_BASEADDR)


// Peripheral base addresses macros type-casted to (xxxx_RegDef_t*)
// GPIO_RegDef_t
#define GPIOA         ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB         ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC         ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD         ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE         ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF         ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG         ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH         ((GPIO_RegDef_t*)GPIOH_BASEADDR)


// Peripheral base addresses macros type-casted to (xxxx_RegDef_t*)
// SPI_RegDef_t
#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_OR_I2S2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_OR_I2S3_BASEADDR)
#define SPI4			((SPI_RegDef_t*)SPI4_BASEADDR)


// Peripheral base addresses macros type-casted to (xxxx_RegDef_t*)
// I2C_RegDef_t
#define I2C1			((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t *)I2C3_BASEADDR)


// Peripheral base addresses macros type-casted to (xxxx_RegDef_t*)
// USART_RegDef_t
#define USART1			((USART_RegDef_t *)USART1_BASEADDR)
#define USART2			((USART_RegDef_t *)USART2_BASEADDR)
#define USART3			((USART_RegDef_t *)USART3_BASEADDR)
#define UART4			((USART_RegDef_t *)UART4_BASEADDR)
#define UART5			((USART_RegDef_t *)UART5_BASEADDR)
#define USART6			((USART_RegDef_t *)USART6_BASEADDR)


// Clock enable macros for GPIOx peripherals
#define GPIOA_PCLCK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLCK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLCK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLCK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLCK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLCK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLCK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLCK_EN()	(RCC->AHB1ENR |= (1 << 7))


// Clock enable macros for I2Cx peripherals
#define I2C1_PCLCK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLCK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLCK_EN()		(RCC->APB1ENR |= (1 << 23))


// Clock enable macros for SPIx peripherals
#define SPI1_PCLCK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLCK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLCK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLCK_EN()		(RCC->APB2ENR |= (1 << 13))


// Clock enable macros for USARTx peripherals
#define USART1_PCLCK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLCK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLCK_EN()		(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLCK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLCK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLCK_EN()		(RCC->APB2ENR |= (1 << 5))


// Clock enable macros for SYSCFG peripherals
#define SYSCFG_PCLCK_EN()		(RCC->APB2ENR |= (1 << 14))


// GPIOx peripheral register reset macros
// Since it is a reset register we should set once and then clear the registers immediately
// Else the registers will be reset continuously
#define GPIOA_REG_RESET()	do {(RCC->AHB1_RSTR |= (1 << 0)); (RCC->AHB1_RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()	do {(RCC->AHB1_RSTR |= (1 << 1)); (RCC->AHB1_RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()	do {(RCC->AHB1_RSTR |= (1 << 2)); (RCC->AHB1_RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()	do {(RCC->AHB1_RSTR |= (1 << 3)); (RCC->AHB1_RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()	do {(RCC->AHB1_RSTR |= (1 << 4)); (RCC->AHB1_RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()	do {(RCC->AHB1_RSTR |= (1 << 5)); (RCC->AHB1_RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()	do {(RCC->AHB1_RSTR |= (1 << 6)); (RCC->AHB1_RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()	do {(RCC->AHB1_RSTR |= (1 << 7)); (RCC->AHB1_RSTR &= ~(1 << 7));} while(0)



// SPIx peripheral register reset macros
// Since it is a reset register we should set once and then clear the registers immediately
// Else the registers will be reset continuously
#define SPI1_REG_RESET()	do {RCC->APB2_RSTR |= (1 << 12); RCC->APB2_RSTR &= ~(1 << 12);} while(0)
#define SPI2_REG_RESET()	do {RCC->APB1_RSTR |= (1 << 14); RCC->APB1_RSTR &= ~(1 << 14);} while(0)
#define SPI3_REG_RESET()	do {RCC->APB1_RSTR |= (1 << 15); RCC->APB1_RSTR &= ~(1 << 15);} while(0)
#define SPI4_REG_RESET()	do {RCC->APB2_RSTR |= (1 << 13); RCC->APB2_RSTR &= ~(1 << 13);} while(0)


// I2Cx peripheral register reset macros
// Since it is a reset register we should set once and then clear the registers immediately
// Else the registers will be reset continuously
#define I2C1_REG_RESET()	do {RCC->APB1_RSTR |= (1 << 21); RCC->APB1_RSTR &= ~(1 << 21);} while(0)
#define I2C2_REG_RESET()	do {RCC->APB1_RSTR |= (1 << 22); RCC->APB1_RSTR &= ~(1 << 22);} while(0)
#define I2C3_REG_RESET()	do {RCC->APB1_RSTR |= (1 << 23); RCC->APB1_RSTR &= ~(1 << 23);} while(0)


// USARTx peripheral register reset macros
// Since it is a reset register we should set once and then clear the registers immediately
// Else the registers will be reset continuously
#define USART1_REG_RESET()	do {RCC->APB2_RSTR |= (1 << 4);  RCC->APB2_RSTR &= ~(1 << 4);}  while(0)
#define UART2_REG_RESET()	do {RCC->APB1_RSTR |= (1 << 17); RCC->APB1_RSTR &= ~(1 << 17);} while(0)
#define UART3_REG_RESET()	do {RCC->APB1_RSTR |= (1 << 18); RCC->APB1_RSTR &= ~(1 << 18);} while(0)
#define UART4_REG_RESET()	do {RCC->APB1_RSTR |= (1 << 19); RCC->APB1_RSTR &= ~(1 << 19);} while(0)
#define UART5_REG_RESET()	do {RCC->APB1_RSTR |= (1 << 20); RCC->APB1_RSTR &= ~(1 << 20);} while(0)
#define USART6_REG_RESET()	do {RCC->APB2_RSTR |= (1 << 5);  RCC->APB2_RSTR &= ~(1 << 5);}  while(0)



// Clock disable macros for GPIOx peripherals
#define GPIOA_PCLCK_DI()	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLCK_DI()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLCK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLCK_DI()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLCK_DI()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLCK_DI()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLCK_DI()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLCK_DI()	(RCC->AHB1ENR &= ~(1 << 7))


// Clock disable macros for I2Cx peripherals
#define I2C1_PCLCK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLCK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLCK_DI()		(RCC->APB1ENR &= ~(1 << 23))


// Clock disable macros for SPIx peripherals
#define SPI1_PCLCK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLCK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLCK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLCK_DI()		(RCC->APB2ENR &= ~(1 << 13))


// Clock disable macros for USARTx peripherals
#define USART1_PCLCK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLCK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLCK_DI()		(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLCK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLCK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLCK_DI()		(RCC->APB2ENR &= ~(1 << 5))


// Clock disable macros for SYSCFG peripherals
#define SYSCFG_PCLCK_DI()		(RCC->APB2ENR &= ~(1 << 14))


// Returns GPIO port code for the given GPIOx port baseaddress
#define GPIO_BASEADDR_TO_PORTCODE(x)	   ((x == GPIOA) ? 0 :\
											(x == GPIOB) ? 1 :\
											(x == GPIOC) ? 2 :\
											(x == GPIOD) ? 3 :\
											(x == GPIOE) ? 4 :\
											(x == GPIOF) ? 5 :\
											(x == GPIOG) ? 6 :\
											(x == GPIOH) ? 7 : 0)


// Interrupt Request Numbers (IRQ) of STM32F446 MCU
#define IRQ_NUM_EXTI0			6
#define IRQ_NUM_EXTI1			7
#define IRQ_NUM_EXTI2			8
#define IRQ_NUM_EXTI3			9
#define IRQ_NUM_EXTI4			10
#define IRQ_NUM_EXTI9_5			23
#define IRQ_NUM_EXTI15_10		40
#define IRQ_NUM_EXTI16			41  //RTC Alarms A & B through EXTI
#define IRQ_NUM_EXTI17			42  //USB OTG_FS WKUP through EXTI
#define IRQ_NUM_EXTI18			76  //USB OTG_HS_WKUP through EXTI
#define IRQ_NUM_SPI1			35
#define IRQ_NUM_SPI2			36
#define IRQ_NUM_SPI3			51
#define IRQ_NUM_SPI4			84
#define IRQ_NUM_I2C1_EV			31
#define IRQ_NUM_I2C1_ER			32
#define IRQ_NUM_I2C2_EV			33
#define IRQ_NUM_I2C2_ER			34
#define IRQ_NUM_I2C3_EV			72
#define IRQ_NUM_I2C3_ER			73
#define IRQ_NUM_USART1			37
#define IRQ_NUM_USART2			38
#define IRQ_NUM_USART3			39
#define IRQ_NUM_UART4			52
#define IRQ_NUM_UART5			53
#define IRQ_NUM_USART6			71


// MCU generic macros
#define ENABLE 				1
#define DISABLE 			0
#define SET					1
#define RESET				0
#define GPIO_PIN_SET		1
#define GPIO_PIN_RESET		0
#define FLAG_SET			1
#define FLAG_UNSET			0


// Bit position macro definitions for SPI peripheral
// Bit position macros for SPI_CR1 Register
#define SPI_CR1_CPHA					0
#define SPI_CR1_CPOL					1
#define SPI_CR1_MSTR					2
#define SPI_CR1_BR						3
#define SPI_CR1_SPE						6
#define SPI_CR1_LSB_FIRST				7
#define SPI_CR1_SSI						8
#define SPI_CR1_SSM						9
#define SPI_CR1_RXONLY					10
#define SPI_CR1_DFF						11
#define SPI_CR1_CRC_NEXT				12
#define SPI_CR1_CRC_EN					13
#define SPI_CR1_BIDI_OE					14
#define SPI_CR1_BIDI_MODE				15

// Bit position macros for SPI_CR2 Register
#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

// Bit position macros for SPI_SR Register
#define SPI_SR_RXNE							0
#define SPI_SR_TXE							1
#define SPI_SR_CHSIDE						2
#define SPI_SR_UDR							3
#define SPI_SR_CRC_ERR						4
#define SPI_SR_MODF							5
#define SPI_SR_OVR							6
#define SPI_SR_BSY							7
#define SPI_SR_FRE							8



// Bit position macro definitions for I2C peripheral
// Bit position macros for I2C_CR1 Register
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRECH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15


// Bit position macros for I2C_CR2 Register
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12


// Bit position macros for I2C_DR Register
#define I2C_DR						0


// Bit position macros for I2C_SR1 Register
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15


// Bit position macros for I2C_SR2 Register
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8


// Bit position macros for I2C_CCR Register
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_F_S			15



// Bit position macros for USART_SR Register
#define USART_SR_PE				0
#define USART_SR_FE				1
#define USART_SR_NF				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC				6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9


// Bit position macros for USART_BRR Register
#define USART_BRR_DIV_FRACTION			0
#define USART_BRR_DIV_MANTISSA			4

// Bit position macros for USART_CR1 Register
#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15


// Bit position macros for USART_CR2 Register
#define USART_CR2_ADD			0
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14


// Bit position macros for USART_CR3 Register
#define USART_CR3_EIE				0
#define USART_CR3_IREN				1
#define USART_CR3_IRLP				2
#define USART_CR3_HDSEL				3
#define USART_CR3_NACK				4
#define USART_CR3_SCEN				5
#define USART_CR3_DMAR				6
#define USART_CR3_DMAT				7
#define USART_CR3_RTSE				8
#define USART_CR3_CTSE				9
#define USART_CR3_CTSIE				10
#define USART_CR3_ONEBIT			11


// Bit position macros for USART_GTPR Register
#define USART_GTPR_PSC			0
#define USART_GTPR_GT			8




#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"

#endif /* INC_STM32F446XX_H_ */
