/*
 * stm32f051xx_gpio_driver.h
 *
 *  Created on: Jan 4, 2020
 *      Author: ITL
 */

#ifndef INC_STM32F051XX_GPIO_DRIVER_H_
#define INC_STM32F051XX_GPIO_DRIVER_H_

#include "stm32f051xx.h"

/*
 * Configuration structure for a GPIO pin
 */
typedef struct {
	uint8_t GPIO_PinNumber;						/*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;						/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;						/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;				/*!< possible values from @GPIO_PIN_PU_PD >*/
	uint8_t GPIO_PinOPType;						/*!< possible values from @GPIO_PIN_OP_TYPE >*/
	uint8_t GPIO_PinAltFunMode;					/*!< possible values from @GPIO_PIN_MODES >*/
} GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;						// Holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;			// Holds GPIO pin configuration settings
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_0							0
#define GPIO_PIN_1							1
#define GPIO_PIN_2							2
#define GPIO_PIN_3							3
#define GPIO_PIN_4							4
#define GPIO_PIN_5							5
#define GPIO_PIN_6							6
#define GPIO_PIN_7							7
#define GPIO_PIN_8							8
#define GPIO_PIN_9							9
#define GPIO_PIN_10							10
#define GPIO_PIN_11							11
#define GPIO_PIN_12							12
#define GPIO_PIN_13							13
#define GPIO_PIN_14							14
#define GPIO_PIN_15							15

/*
 * @GPIO_PIN_MODES
 * GPIO possible modes
 */
#define GPIO_MODE_IN						0
#define	GPIO_MODE_OUT						1
#define GPIO_MODE_ALTFN						2
#define GPIO_MODE_ANALOG					3
#define GPIO_MODE_IT_FT						4	// Interrupt falling edge
#define GPIO_MODE_IT_RT						5	// Interrupt rising edge
#define GPIO_MODE_IT_RFT					6	// falling/rising

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO output pin types
 */
#define GPIO_OP_TYPE_PP						0 	// push-pull
#define GPIO_OP_TYPE_OD						1	// open drain

/*
 * @@GPIO_PIN_SPEED
 * GPIO output pin speed
 */
#define GPIO_SPEED_LOW						0
#define GPIO_SPEED_MEDIUM					1
#define GPIO_SPEED_HIGH						3

/*
 * @GPIO_PIN_PU_PD
 * GPIO pin pull up and pull down modes
 */
#define GPIO_NO_PUPD						0	// no config
#define	GPIO_PIN_PU							1	// pull up
#define	GPIO_PIN_PD							2	// pull down

/*
 * IRQ EXTI position numbers
 */
#define	IQR_EXTI0_1 						5
#define IRQ_EXTI2_3							6
#define IRQ_EXTI4_15						7

/**************************************************************************
 * APIs supported by this driver
 **************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Initialization/Deinitialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read/write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, int8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, int16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptCongif(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F051XX_GPIO_DRIVER_H_ */
