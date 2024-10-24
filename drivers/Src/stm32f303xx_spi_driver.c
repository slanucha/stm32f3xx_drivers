/*
 * stm32f303xx_gpio_driver.c
 *
 *  Created on: Oct 23, 2024
 *      Author: slan
 */

#include "stm32f303xx_spi_driver.h"

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
  if (EnOrDi == ENABLE)
  {
    switch ((uint32_t)pSPIx)
    {
    case SPI1_BASEADDR:
      SPI1_PCLOCK_EN();
      break;
    case SPI2_BASEADDR:
      SPI2_PCLOCK_EN();
      break;
    case SPI3_BASEADDR:
      SPI3_PCLOCK_EN();
      break;
    case SPI4_BASEADDR:
      SPI4_PCLOCK_EN();
      break;
    default:
      break;
    }
  } else
  {
    switch ((uint32_t)pSPIx)
    {
    case SPI1_BASEADDR:
      SPI1_PCLOCK_DI();
      break;
    case SPI2_BASEADDR:
      SPI2_PCLOCK_DI();
      break;
    case SPI3_BASEADDR:
      SPI3_PCLOCK_DI();
      break;
    case SPI4_BASEADDR:
      SPI4_PCLOCK_DI();
      break;
    default:
      break;
    }
  }
}

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
  SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

  uint32_t tempreg = 0;

  tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

  switch(pSPIHandle->SPI_Config.SPI_BusConfig)
  {
  case SPI_BUS_CONFIG_FD:
    tempreg &= ~(0x1 << SPI_CR1_BDIMODE);
    break;
  case SPI_BUS_CONFIG_HD:
    tempreg |= (0x1 << SPI_CR1_BDIMODE);
    break;
  case SPI_BUS_CONFIG_SIMPLEX_RXONLY:
    tempreg &= ~(0x1 << SPI_CR1_BDIMODE);
    tempreg |= (0x1 << SPI_CR1_RXONLY);
    break;
  default:
    break;
  }

  tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;
  tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;
  tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;
  tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

  pSPIHandle->pSPIx->CR1 = tempreg;

  tempreg = 0;
  tempreg |= pSPIHandle->SPI_Config.SPI_DS << SPI_CR2_DS;
  pSPIHandle->pSPIx->CR2 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
  switch ((uint32_t)pSPIx) {
  case SPI1_BASEADDR:
    SPI1_REG_RESET();
    break;
  case SPI2_BASEADDR:
    SPI2_REG_RESET();
    break;
  case SPI3_BASEADDR:
    SPI3_REG_RESET();
    break;
  case SPI4_BASEADDR:
    SPI4_REG_RESET();
    break;
  default:
    break;
  }
}

/*
 * Data Read and Write
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
  uint8_t data_size = (pSPIx->CR2 & (0xF << SPI_CR2_DS)) >> SPI_CR2_DS;

  while (len > 0)
  {
    while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

    if (data_size == SPI_DS_16BIT)
    {
      // 16-bit
      pSPIx->DR = *((uint16_t*)pTxBuffer);
      len--;
      if (len > 0)
      {
        len--;
        pTxBuffer += 2;
      }
    } else
    {
      // 8-bit
      pSPIx->DR = *pTxBuffer;
      len--;
      pTxBuffer++;
    }
  }
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
  uint8_t data_size = (pSPIx->CR2 & (0xF << SPI_CR2_DS)) >> SPI_CR2_DS;

  while (len > 0)
  {
    while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_SET);

    if (data_size == SPI_DS_16BIT)
    {
      // 16 bit
      *((uint16_t*)pRxBuffer)= pSPIx->DR;
      len -= 2;
      pRxBuffer += 2;

    } else
    {
      // 8 bit
      *pRxBuffer = (uint8_t)pSPIx->DR;
      len--;
      pRxBuffer++;
    }
  }
}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}

/*
 * Other Peripheral Control APIs
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
  if (pSPIx->SR & FlagName)
  {
    return FLAG_SET;
  }
  return FLAG_RESET;
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
  if (EnOrDi == ENABLE)
  {
    pSPIx->CR1 |= (0x1 << SPI_CR1_SPE);
  } else
  {
    pSPIx->CR1 &= ~(0x1 << SPI_CR1_SPE);
  }
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
  if (EnOrDi == ENABLE)
  {
    pSPIx->CR1 |= (0x1 << SPI_CR1_SSI);
  } else
  {
    pSPIx->CR1 &= ~(0x1 << SPI_CR1_SSI);
  }
}
