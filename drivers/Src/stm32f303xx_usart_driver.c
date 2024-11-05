/*
 * stm32f303xx_usart_driver.c
 *
 *  Created on: Nov 4, 2024
 *      Author: slan
 */

#include "stm32f303xx_usart_driver.h"

static void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baudRate);
static void USART_TXE_Interrupt_Handle(USART_Handle_t *pUSARTHandle);

/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
  if (EnOrDi == ENABLE)
  {
    switch ((uint32_t)pUSARTx)
    {
    case USART1_BASEADDR:
      USART1_PCLOCK_EN();
      break;
    case USART2_BASEADDR:
      USART2_PCLOCK_EN();
      break;
    case USART3_BASEADDR:
      USART3_PCLOCK_EN();
      break;
    case UART4_BASEADDR:
      UART4_PCLOCK_EN();
      break;
    case UART5_BASEADDR:
      UART5_PCLOCK_EN();
      break;
    default:
      break;
    }
  } else
  {
    switch ((uint32_t)pUSARTx)
    {
    case USART1_BASEADDR:
      USART1_PCLOCK_DI();
      break;
    case USART2_BASEADDR:
      USART2_PCLOCK_DI();
      break;
    case USART3_BASEADDR:
      USART3_PCLOCK_DI();
      break;
    case UART4_BASEADDR:
      UART4_PCLOCK_DI();
      break;
    case UART5_BASEADDR:
      UART5_PCLOCK_DI();
      break;
    default:
      break;
    }
  }
}

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
  USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

  uint32_t tempreg = 0;

  // Transmitter/Receiver
  if (pUSARTHandle->USART_Config.USART_Mode  == USART_MODE_ONLY_RX)
  {
    tempreg |= (0x1 << USART_CR1_RE);
  } else if (pUSARTHandle->USART_Config.USART_Mode  == USART_MODE_ONLY_TX)
  {
    tempreg |= (0x1 << USART_CR1_TE);
  } else if (pUSARTHandle->USART_Config.USART_Mode  == USART_MODE_TXRX)
  {
    tempreg |= ((0x1 << USART_CR1_TE) | (0x1 << USART_CR1_RE));
  }

  // Word length
  tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M0);

  // Parity
  if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
  {
    // Enable parity control
    tempreg |= (0x1 << USART_CR1_PCE);
  } else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
  {
    // Enable parity control
    tempreg |= (0x1 << USART_CR1_PCE);
    // Enable odd parity
    tempreg |= (0x1 << USART_CR1_PS);
  }

  pUSARTHandle->pUSARTx->CR1 = tempreg;
  tempreg = 0;

  // Number of stop bits
  tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

  pUSARTHandle->pUSARTx->CR2 = tempreg;
  tempreg = 0;

  if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
  {
    tempreg |= (0x1 << USART_CR3_CTSE);
  } else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
  {
    tempreg |= (0x1 << USART_CR3_RTSE);
  } else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
  {
    tempreg |= ((0x1 << USART_CR3_CTSE) | (0x1 << USART_CR3_RTSE));
  }

  pUSARTHandle->pUSARTx->CR3 = tempreg;

  USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
  switch ((uint32_t)pUSARTx) {
  case USART1_BASEADDR:
    USART1_REG_RESET();
    break;
  case USART2_BASEADDR:
    USART2_REG_RESET();
    break;
  case USART3_BASEADDR:
    USART3_REG_RESET();
    break;
  case UART4_BASEADDR:
    UART4_REG_RESET();
    break;
  case UART5_BASEADDR:
    UART5_REG_RESET();
    break;
  default:
    break;
  }
}

/*
 * Data Read and Write
 */
void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t len)
{
  for (uint32_t i = 0; i < len; ++i)
  {
    while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE));

    pUSARTx->TDR = *pTxBuffer;
    pTxBuffer++;
  }

  while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_TC));
}

void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t len)
{
  for (uint32_t i = 0; i < len; ++i)
  {
    while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_RXNE));

    *pRxBuffer = pUSARTx->RDR;
    pRxBuffer++;
  }
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
  uint8_t txstate = pUSARTHandle->TxState;

  if (txstate != USART_BUSY_IN_TX)
  {
    pUSARTHandle->TxLen = len;
    pUSARTHandle->pTxBuffer = pTxBuffer;
    pUSARTHandle->TxState = USART_BUSY_IN_TX;

    // Enable interrupt for TXE
    pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_TXEIE);

    // Enable interrupt for TC
    pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_TCIE);
  }

  return txstate;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
  uint8_t rxstate = pUSARTHandle->RxState;

  if (rxstate != USART_BUSY_IN_RX)
  {
    pUSARTHandle->RxLen = len;
    pUSARTHandle->pRxBuffer = pRxBuffer;
    pUSARTHandle->RxState = USART_BUSY_IN_RX;

    // Enable interrupt for RXNE
    pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_RXNEIE);
  }

  return rxstate;
}

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
      *NVIC_ICER0 &= ~(0x1 << IRQNumber);
    } else if (IRQNumber > 31 && IRQNumber < 64)
    {
      *NVIC_ICER1 &= ~(0x1 << (IRQNumber % 32));
    } else if (IRQNumber >= 64 &&  IRQNumber < 96)
    {
      *NVIC_ICER2 &= ~(0x1 << (IRQNumber % 32));
    }
  }
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
  uint8_t iprx = IRQNumber / 4;
  uint8_t iprx_section = IRQNumber % 4;

  uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
  *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
  uint8_t temp1, temp2;

  // Check for TXE
  temp1 = pUSARTHandle->pUSARTx->ISR & (0x1 << USART_ISR_TXE);
  temp2 = pUSARTHandle->pUSARTx->CR1 & (0x1 << USART_CR1_TXEIE);
  if (temp1 && temp2)
  {
    USART_TXE_Interrupt_Handle(pUSARTHandle);
  }
}

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
  if (EnOrDi == ENABLE)
  {
    pUSARTx->CR1 |= (0x1 << USART_CR1_UE);
  } else
  {
    pUSARTx->CR1 &= ~(0x1 << USART_CR1_UE);
  }
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
  if (pUSARTx->ISR & FlagName)
  {
    return FLAG_SET;
  }
  return FLAG_RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
  pUSARTx->ICR |= (0x1 << StatusFlagName);
}

/*
 * Application callback
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent)
{
}

static void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baudRate)
{
  uint32_t PCLKx;
  uint32_t usartdiv;
  uint32_t M_part, F_part;
  uint32_t tempreg = 0;

  if (pUSARTx == USART1)
  {
    PCLKx = RCC_GetPCLK2Value();
  } else
  {
    PCLKx = RCC_GetPCLK1Value();
  }

  if (pUSARTx->CR1 & (0x1 << USART_CR1_OVER8))
  {
    usartdiv = ((25 * PCLKx) / (2 * baudRate));
  } else
  {
    usartdiv = ((25 * PCLKx) / (4 * baudRate));
  }

  M_part = usartdiv / 100;
  F_part = usartdiv - (M_part * 100);

  tempreg |= (M_part << 4);
  tempreg |= (F_part << 0);
  pUSARTx->BRR = tempreg;
}

static void USART_TXE_Interrupt_Handle(USART_Handle_t *pUSARTHandle)
{
  if (pUSARTHandle->TxLen > 0)
  {
    pUSARTHandle->pUSARTx->TDR = *pUSARTHandle->pTxBuffer;
    pUSARTHandle->TxLen--;
    pUSARTHandle->pTxBuffer++;
  } else
  {
    pUSARTHandle->pTxBuffer = NULL;
    pUSARTHandle->TxState = USART_READY;
  }
}
