/*
 * stm32f051xx_spi_driver.h
 *
 *  Created on: Jan 6, 2020
 *      Author: ITL
 */

#ifndef INC_STM32F051XX_SPI_DRIVER_H_
#define INC_STM32F051XX_SPI_DRIVER_H_

#include "stm32f051xx.h"

/*
 *  Configuration structure for SPIx peripherals
 */
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t	SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t	SPI_DDF;
	uint8_t	SPI_CPOL;
	uint8_t	SPI_CPHA;
	uint8_t	SPI_SSM;
} SPI_Config_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER 					1
#define SPI_DEVICE_MODE_SLAVE					0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD						1
#define SPI_BUS_CONFIG_HD						2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY			3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_VID2						0
#define SPI_SCLK_SPEED_VID4						1
#define SPI_SCLK_SPEED_VID8						2
#define SPI_SCLK_SPEED_VID16					3
#define SPI_SCLK_SPEED_VID32					4
#define SPI_SCLK_SPEED_VID64					5
#define SPI_SCLK_SPEED_VID128					6
#define SPI_SCLK_SPEED_VID256					7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS							0
#define SPI_DFF_16BITS							1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH							1
#define SPI_CPOL_LOW							0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH							1
#define SPI_CPHA_LOW							0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN								1
#define SPI_SSM_DI								0

/*
 * SPI related flag status flags definitions
 */
#define SPI_TXE_FLAG							(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG							(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG							(1 << SPI_SR_BSY)

/*
 * Handle structure for SPIx peripheral
 */
typedef struct {
	SPI_RegDef_t 	*pSPIx; 				/*!< Holds the base address of SPIx(x:0,1) peripherals>*/
	SPI_Config_t	SPIConfig;
} SPI_Handle_t;

/***************************************************************************
 * API
 ***************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Initialization/Deinitialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptCongif(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other peripheral APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

#endif /* INC_STM32F051XX_SPI_DRIVER_H_ */
