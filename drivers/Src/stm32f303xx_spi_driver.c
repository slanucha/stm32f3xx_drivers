/*
 * stm32f303xx_gpio_driver.c
 *
 *  Created on: Oct 23, 2024
 *      Author: slan
 */

#include "stm32f303xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
  uint8_t state = pSPIHandle->TxState;

  if (state != SPI_BUSY_IN_TX)
  {
    pSPIHandle->pTxBuffer = pTxBuffer;
    pSPIHandle->TxLen = len;
    pSPIHandle->TxState = SPI_BUSY_IN_TX;

    pSPIHandle->pSPIx->CR2 |= (0x1 << SPI_CR2_TXEIE);
  }

  return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
  uint8_t state = pSPIHandle->TxState;

  if (state != SPI_BUSY_IN_RX)
  {
    pSPIHandle->pRxBuffer = pRxBuffer;
    pSPIHandle->RxLen = len;
    pSPIHandle->RxState = SPI_BUSY_IN_RX;

    pSPIHandle->pSPIx->CR2 |= (0x1 << SPI_CR2_RXNEIE);
  }

  return state;
}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
  if (EnOrDi == ENABLE)
  {
    if (IRQNumber <= 31)
    {
      *NVIC_ISER0 |= (0x1 << IRQNumber);
    } else if (IRQNumber > 31 && IRQNumber < 64)
    {
      *NVIC_ISER1 |= (0x1 << (IRQNumber % 32));
    } else if (IRQNumber >= 64 &&  IRQNumber < 96)
    {
      *NVIC_ISER2 |= (0x1 << (IRQNumber % 32));
    }
  } else
  {
    if (IRQNumber <= 31)
    {
      *NVIC_ICER0 |= (0x1 << IRQNumber);
    } else if (IRQNumber > 31 && IRQNumber < 64)
    {
      *NVIC_ICER1 |= (0x1 << (IRQNumber % 32));
    } else if (IRQNumber >= 64 &&  IRQNumber < 96)
    {
      *NVIC_ICER2 |= (0x1 << (IRQNumber % 32));
    }
  }
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
  uint8_t iprx = IRQNumber / 4;
  uint8_t iprx_section = IRQNumber % 4;

  uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
  *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
  uint8_t temp1, temp2;

  // Check for TXE
  temp1 = pSPIHandle->pSPIx->SR & (0x1 << SPI_SR_TXE);
  temp2 = pSPIHandle->pSPIx->CR2 & (0x1 << SPI_CR2_TXEIE);
  if (temp1 && temp2)
  {
    spi_txe_interrupt_handle(pSPIHandle);
  }

  // Check for RXNE
  temp1 = pSPIHandle->pSPIx->SR & (0x1 << SPI_SR_RXNE);
  temp2 = pSPIHandle->pSPIx->CR2 & (0x1 << SPI_CR2_RXNEIE);
  if (temp1 && temp2)
  {
    spi_rxne_interrupt_handle(pSPIHandle);
  }

  // Check for OVR
  temp1 = pSPIHandle->pSPIx->SR & (0x1 << SPI_SR_OVR);
  temp2 = pSPIHandle->pSPIx->CR2 & (0x1 << SPI_CR2_ERRIE);
  if (temp1 && temp2)
  {
    spi_ovr_err_interrupt_handle(pSPIHandle);
  }
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

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
  if (EnOrDi == ENABLE)
  {
    pSPIx->CR2 |= (0x1 << SPI_CR2_SSOE);
  } else
  {
    pSPIx->CR2 &= ~(0x1 << SPI_CR2_SSOE);
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

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
  uint8_t temp;
  temp = pSPIx->DR;
  temp = pSPIx->SR;
  (void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
  pSPIHandle->pSPIx->CR2 &= ~(0x1 << SPI_CR2_TXEIE);
  pSPIHandle->pTxBuffer = NULL;
  pSPIHandle->TxLen = 0;
  pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
  pSPIHandle->pSPIx->CR2 &= ~(0x1 << SPI_CR2_RXNEIE);
  pSPIHandle->pRxBuffer = NULL;
  pSPIHandle->RxLen = 0;
  pSPIHandle->RxState = SPI_READY;
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
  uint8_t data_size = (pSPIHandle->pSPIx->CR2 & (0xF << SPI_CR2_DS)) >> SPI_CR2_DS;

  if (data_size == SPI_DS_16BIT)
  {
    // 16-bit
    pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
    pSPIHandle->TxLen--;
    if (pSPIHandle->TxLen > 0)
    {
      pSPIHandle->TxLen--;
      pSPIHandle->pTxBuffer += 2;
    }
  } else
  {
    // 8-bit
    pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
    pSPIHandle->TxLen--;
    pSPIHandle->pTxBuffer++;
  }

  if (!pSPIHandle->TxLen)
  {
    // TxLen is zero, close the SPI transmission
    SPI_CloseTransmission(pSPIHandle);
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
  }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
  uint8_t data_size = (pSPIHandle->pSPIx->CR2 & (0xF << SPI_CR2_DS)) >> SPI_CR2_DS;

  if (data_size == SPI_DS_16BIT)
  {
    // 16 bit
    *((uint16_t*)pSPIHandle->pRxBuffer)= pSPIHandle->pSPIx->DR;
    pSPIHandle->RxLen--;
    if (pSPIHandle->RxLen > 0)
    {
      pSPIHandle->RxLen--;
      pSPIHandle->pRxBuffer += 2;
    }
  } else
  {
    // 8 bit
    *pSPIHandle->pRxBuffer = (uint8_t)pSPIHandle->pSPIx->DR;
    pSPIHandle->RxLen--;
    pSPIHandle->pRxBuffer++;
  }

  if (!pSPIHandle->RxLen)
  {
    // RxLen is zero, close the SPI transmission
    SPI_CloseReception(pSPIHandle);
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
  }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

  if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
  {
    SPI_ClearOVRFlag(pSPIHandle->pSPIx);
  }

  SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t SPIEvent)
{
  // This is a weak implementation. User may override this function.
}
