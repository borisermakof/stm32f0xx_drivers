/*
 * stm32f051xx_spi_driver.c
 *
 *  Created on: Jan 6, 2020
 *      Author: ITL
 */

#include "stm32f051xx_spi_driver.h"
#include "stm32f051xx.h"

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}
	}
	else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		}
	}
}

/*
 * Initialization/Deinitialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Configure the SPI_CR1 registers
	uint32_t tempReg = 0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 1. Configure the device mode
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	// 2. Configure bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		// RxOnly
		tempReg |= (1 << SPI_CR1_RX_ONLY);
	}
	// 3. Clock speed
	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
	// 4. DDF(Data size)
	tempReg |= pSPIHandle->SPIConfig.SPI_DDF << SPI_CR1_CRCL;
	// 5. Clock polarity
	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	// 6. Clock phase
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	// 7. SSM
	tempReg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempReg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	}
}

uint8_t SPI_GetSRFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	return pSPIx->SR & FlagName;
}

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	if (Len == 0) {
		return;
	}

	while(Len > 0) {
		// 1. Wait until TXE is set
		while(SPI_GetSRFlagStatus(pSPIx, SPI_TXE_FLAG) == RESET);

		// 2. Check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_CRCL)) {
			// 16 bit
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		} else {
			// 8 bit
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptCongif(uint8_t IRQNumber, uint8_t EnOrDi)
{

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}
