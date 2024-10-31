/*
 * stm32f303xx_i2c_driver.c
 *
 *  Created on: Oct 29, 2024
 *      Author: slan
 */

#include "stm32f303xx_spi_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint32_t len, uint8_t ReadOrWrite);
/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
  if (EnOrDi == ENABLE)
  {
    switch ((uint32_t)pI2Cx)
    {
    case I2C1_BASEADDR:
      I2C1_PCLOCK_EN();
      break;
    case I2C2_BASEADDR:
      I2C2_PCLOCK_EN();
      break;
    case I2C3_BASEADDR:
      I2C3_PCLOCK_EN();
      break;
    default:
      break;
    }
  } else
  {
    switch ((uint32_t)pI2Cx)
    {
    case I2C1_BASEADDR:
      I2C1_PCLOCK_DI();
      break;
    case I2C2_BASEADDR:
      I2C2_PCLOCK_DI();
      break;
    case I2C3_BASEADDR:
      I2C3_PCLOCK_DI();
      break;
    default:
      break;
    }
  }
}

/*
 * Init and De-init
 * Timing setting are adjusted for default HSI clock source of 8MHz
 * and will not work if I2C clock source is changed.
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
  I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

  uint32_t tempreg = 0;

  // Timing settings
  uint8_t presc, scll, sclh, sdadel, scldel;
  switch(pI2CHandle->I2C_Config.I2C_SCLSpeed)
  {
  case I2C_SCL_SPEED_SM1:
    presc = 0x1;
    scll = 0xC7;
    sclh = 0xC3;
    sdadel = 0x2;
    scldel = 0x4;
    break;
  case I2C_SCL_SPEED_SM2:
    presc = 0x1;
    scll = 0x13;
    sclh = 0xF;
    sdadel = 0x2;
    scldel = 0x4;
    break;
  case I2C_SCL_SPEED_FM:
    presc = 0x0;
    scll = 0x9;
    sclh = 0x3;
    sdadel = 0x1;
    scldel = 0x3;
    break;
  case I2C_SCL_SPEED_FM_PLUS:
    presc = 0x0;
    scll = 0x6;
    sclh = 0x3;
    sdadel = 0x0;
    scldel = 0x1;
    break;
  default:
    break;
  }

  tempreg |= (presc << I2C_TIMINGR_PRESC);
  tempreg |= (scll << I2C_TIMINGR_SCLL);
  tempreg |= (sclh << I2C_TIMINGR_SCLH);
  tempreg |= (sdadel << I2C_TIMINGR_SDADEL);
  tempreg |= (scldel << I2C_TIMINGR_SCLDEL);

  pI2CHandle->pI2Cx->TIMINGR = tempreg;

  // Device address
  tempreg = 0;
  tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
  pI2CHandle->pI2Cx->OAR1 = tempreg;
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
  switch ((uint32_t)pI2Cx) {
  case I2C1_BASEADDR:
    I2C1_REG_RESET();
    break;
  case I2C2_BASEADDR:
    I2C2_REG_RESET();
    break;
  case I2C3_BASEADDR:
    I2C3_REG_RESET();
    break;
  default:
    break;
  }
}

/*
 * Data Read and Write
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr)
{
  // Confirm bus is idle
  while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BUSY));

  // Configure address and data length
  I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddr, len, I2C_WRITE);

  // Generate start condition
  I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

  // Wait until bus becomes busy
  while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BUSY));

  // Write bytes until the transfer is complete
  while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TC))
  {
    if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXIS))
    {
      pI2CHandle->pI2Cx->TXDR = *pTxBuffer;
      pTxBuffer++;
    }
  }

  // Generate stop condition
  I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr)
{
  // Confirm bus is idle
  while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BUSY));

  // Configure address and data length
  I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddr, len, I2C_READ);

  // Generate start condition
  I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

  // Wait until bus becomes busy
  while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BUSY));

  // Read bytes until all data is collected
  uint32_t dataReceived = 0;
  while (dataReceived < len)
  {
    if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)) {
      *pRxBuffer = pI2CHandle->pI2Cx->RXDR;
      pRxBuffer++;
      dataReceived++;
    }
  }

  // Generate stop condition
  I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
  pI2Cx->CR2 |= (0x1 << I2C_CR2_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
  pI2Cx->CR2 |= (0x1 << I2C_CR2_STOP);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint32_t, len, uint8_t ReadOrWrite)
{
  // Configure address
  slaveAddr = slaveAddr << 1;
  pI2Cx->CR2 |= (slaveAddr << I2C_CR2_SADD);
  pI2Cx->CR2 |= (ReadOrWrite << I2C_CR2_RD_WRN);

  // Set the data length
  pI2Cx->CR2 |= (len << I2C_CR2_NBYTES);
}

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
  uint8_t iprx = IRQNumber / 4;
  uint8_t iprx_section = IRQNumber % 4;

  uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
  *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
 * Other Peripheral Control APIs
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
  if (pI2Cx->ISR & FlagName)
  {
    return FLAG_SET;
  }
  return FLAG_RESET;
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
  if (EnOrDi == ENABLE)
  {
    pI2Cx->CR1 |= (0x1 << I2C_CR1_PE);
  } else
  {
    pI2Cx->CR1 &= ~(0x1 << I2C_CR1_PE);
  }
}

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t I2CEvent);
