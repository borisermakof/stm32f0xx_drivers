/*
 * stm32f051xx.h
 *
 *  Created on: Jan 4, 2020
 *      Author: ITL
 */

#include <stdint.h>

#ifndef INC_STM32F051XX_H_
#define INC_STM32F051XX_H_

#define __vo volatile

/******************************************************************************
 * ARM Cortex M0 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER						((__vo uint32_t*) 0xE000E100)

/******************************************************************************
 * ARM Cortex M0 Processor NVIC ICERx register addresses
 */
#define NVIC_ICER						((__vo uint32_t*) 0xE000E180)

/*
 * ARM Cortex M0 Processir NVIC IPR register addresses
 */
#define NVIC_IPR_BASE_ADDR				((__vo uint32_t*) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED			4

#define NVIC_IRQ_PR0 					0
#define NVIC_IRQ_PR1 					1
#define NVIC_IRQ_PR2 					2
#define NVIC_IRQ_PR3 					3
#define NVIC_IRQ_PR4 					4
#define NVIC_IRQ_PR5 					5
#define NVIC_IRQ_PR6 					6
#define NVIC_IRQ_PR7 					7
#define NVIC_IRQ_PR8 					8
#define NVIC_IRQ_PR9 					9
#define NVIC_IRQ_PR10 					10
#define NVIC_IRQ_PR11 					11
#define NVIC_IRQ_PR12 					12
#define NVIC_IRQ_PR13 					13
#define NVIC_IRQ_PR14 					14
#define NVIC_IRQ_PR15 					15


/*
 * MEMORY ADDRESSES
 */

#define FLASH_BASE_ADDR					0x08000000U 	// 64KB
#define SRAM1_BASE_ADDR					0x20000000U		// 8KB
#define SRAM 							SRAM1_BASE_ADDR
#define ROM_BASE_ADDR					0x1FFFEC00U		// 3KB
#define ROM 							ROM_BASE_ADDR

/*
 * BUS ADRESSES
 */

#define PERIPH_BASE_ADDR				0x40000000U
#define APB_BASE_ADDR					PERIPH_BASE_ADDR
#define AHB1_BASE_ADDR					0x40020000U
#define AHB2_BASE_ADDR					0x48000000U

/*
 * Addresses of peripherals of APB bus
 */

#define SPI1_BASE_ADDR					(APB_BASE_ADDR + 0x00013000U)
#define SPI2_BASE_ADDR					(APB_BASE_ADDR + 0x3800)

#define I2C1_BASE_ADDR					(APB_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR					(APB_BASE_ADDR + 0x5800)

// #define I2S1_BASE_ADDR					(APB_BASE_ADDR + 0x00013000U)

#define	USART1_BASE_ADDR				(APB_BASE_ADDR + 0x00013800U)
#define	USART2_BASE_ADDR				(APB_BASE_ADDR + 0x4400)
#define	USART3_BASE_ADDR				(APB_BASE_ADDR + 0x4800)
#define	USART4_BASE_ADDR				(APB_BASE_ADDR + 0x4C00)
#define	USART5_BASE_ADDR				(APB_BASE_ADDR + 0x5000)
#define	USART6_BASE_ADDR				(APB_BASE_ADDR + 0x00011400U)
#define	USART7_BASE_ADDR				(APB_BASE_ADDR + 0x00011800U)
#define	USART8_BASE_ADDR				(APB_BASE_ADDR + 0x00011C00U)

#define EXTI_BASE_ADDR					(APB_BASE_ADDR + 0x00010400U)

#define SYSCFG_BASE_ADDR				(APB_BASE_ADDR + 0x00010000U)

/*
 * Addresses of peripherals of AHB2 bus
 */

#define GPIOA_BASE_ADDR					(AHB2_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR					(AHB2_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR					(AHB2_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR					(AHB2_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR					(AHB2_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR					(AHB2_BASE_ADDR + 0x1400)

#define RCC_BASE_ADDR					(AHB1_BASE_ADDR + 0x1000)

/*
 * Peripheral structure definition for GPIO
 */
typedef struct {
	__vo uint32_t MODER;		// GPIO port mode register					Offset 0x00
	__vo uint32_t OTYPER;		// GPIO port output type register			Offset 0x04
	__vo uint32_t OSPEEDR;		// GPIO port output speed register  		Offset 0x08
	__vo uint32_t PUPDR;		// GPIO port pull-up/pull-down register 	Offset 0x0C
	__vo uint32_t IDR;			// GPIO port input data register 			Offset 0x10
	__vo uint32_t ODR;			// GPIO port output data register			Offset 0x14
	__vo uint32_t BSRR;			// GPIO port bit set/reset register			Offset 0x18
	__vo uint32_t LCKR;			// GPIO port configuration lock register 	Offset 0x1C
	__vo uint32_t AFR[2];		// GPIO alternate function low/high registerOffset 0x20-0x24
	__vo uint32_t BRR;			// GPIO port bit reset register 			Offset 0x28
} GPIO_RegDef_t;

/*
 * Peripheral structure definition for RCC
 */
typedef struct {
	__vo uint32_t CR;			// Offset 0x00
	__vo uint32_t CFGR;			// Offset 0x04
	__vo uint32_t CIR;			// Offset 0x08
	__vo uint32_t APB2RSTR;		// Offset 0x0C
	__vo uint32_t APB1RSTR;		// Offset 0x10
	__vo uint32_t AHBENR;		// AHB peripheral clock enable register		Offset 0x14
	__vo uint32_t APB2ENR;		// APB peripheral clock enable register 2   Offset 0x18
	__vo uint32_t APB1ENR;		// APB peripheral clock enable register 1	Offset 0x1C
	__vo uint32_t BDCR;			// RTC domain control register				Offset 0x20
	__vo uint32_t CSR;			// Offset 0x24
	__vo uint32_t AHBRSTR;		// Offset 0x28
	__vo uint32_t CFGR2;		// Offset 0x2C
	__vo uint32_t CFGR3;		// Offset 0x30
	__vo uint32_t CR2;			// Offset 0x34
} RCC_RegDef_t;

/*
 * Peripheral structure definition for EXTI. Extended interrupts and events controller
 */
typedef struct {
	__vo uint32_t IMR; 			// Interrupt mask register 					Offset 0x00
	__vo uint32_t EMR;			// Event mask register 						Offset 0x04
	__vo uint32_t RTSR;			// Rising trigger selection register		Offset 0x08
	__vo uint32_t FTSR;			// Falling trigger selection register 		Offset 0x0C
	__vo uint32_t SWIER;		// Software interrupt event register		Offset 0x10
	__vo uint32_t PR;			// Pending register							Offset 0x14
} EXTI_RegDef_t;

/*
 * Peripheral structure definition for EXTI. Extended interrupts and events controller
 */

/*
 * Peripheral structure definitions for SYSCFG. System configuration controller
 */
typedef struct {
	__vo uint32_t CFGR1;		// Configuration register 1 			Offset 0x00
	uint32_t RESERVERD1;
	__vo uint32_t EXTICR[4];		// External interrupt configuration register 1-4 Offset 0x08-0x14
	__vo uint32_t CFGR2;		// Configuration register 2				Offset 0x18
} SYSCFG_RegDef_t;

#define GPIOA							((GPIO_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB							((GPIO_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC							((GPIO_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD							((GPIO_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE							((GPIO_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOF							((GPIO_RegDef_t*) GPIOF_BASE_ADDR)

#define RCC								((RCC_RegDef_t*) RCC_BASE_ADDR)

#define EXTI							((EXTI_RegDef_t*) EXTI_BASE_ADDR)

#define SYSCFG							((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()					(RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN()					(RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()					(RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN()					(RCC->AHBENR |= (1 << 20))
#define GPIOE_PCLK_EN()					(RCC->AHBENR |= (1 << 21))
#define GPIOF_PCLK_EN()					(RCC->AHBENR |= (1 << 22))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()					(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()					(RCC->APB1ENR |= (1 << 22))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()					(RCC->APB2ENR |= (1 << 12)
#define SPI2_PCLK_EN()					(RCC->APB1ENR |= (1 << 14))

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()				(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()				(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()				(RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()				(RCC->APB2ENR |= (1 << 5))
#define USART7_PCLK_EN()				(RCC->APB2ENR |= (1 << 6))
#define USART8_PCLK_EN()				(RCC->APB2ENR |= (1 << 7))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()				(RCC->APB2ENR |= (1 << 0))

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()					(RCC->AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI()					(RCC->AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI()					(RCC->AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI()					(RCC->AHBENR &= ~(1 << 20))
#define GPIOE_PCLK_DI()					(RCC->AHBENR &= ~(1 << 21))
#define GPIOF_PCLK_DI()					(RCC->AHBENR &= ~(1 << 22))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 22))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 12)
#define SPI2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 14))

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 5))
#define USART7_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 6))
#define USART8_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 7))

/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 0))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()				do { (RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17)); } while (0)
#define GPIOB_REG_RESET()				do { (RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18)); } while (0)
#define GPIOC_REG_RESET()				do { (RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19)); } while (0)
#define GPIOD_REG_RESET()				do { (RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20)); } while (0)
#define GPIOE_REG_RESET()				do { (RCC->AHBRSTR |= (1 << 21)); (RCC->AHBRSTR &= ~(1 << 21)); } while (0)
#define GPIOF_REG_RESET()				do { (RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22)); } while (0)

#define GPIO_BASE_ADDR_TO_PORT_CODE(x)  ( (x == GPIOA) ? 0 :\
										  (x == GPIOB) ? 1 :\
										  (x == GPIOC) ? 2 :\
										  (x == GPIOD) ? 3 :\
										  (x == GPIOE) ? 4 :\
										  (x == GPIOF) ? 5 : 0 )


/*
 * Some generic macros
 */

#define ENABLE 							1
#define DISABLE 						0
#define SET								ENABLE
#define RESET							DISABLE
#define GPIO_PIN_SET					SET
#define GPIO_PIN_RESET					RESET


#endif /* INC_STM32F051XX_H_ */
