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
//void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
//{
//  if (EnOrDi == ENABLE) {
//    switch ((uint32_t)pGPIOx) {
//    case GPIOA_BASEADDR:
//      GPIOA_PCLOCK_EN();
//      break;
//    case GPIOB_BASEADDR:
//      GPIOB_PCLOCK_EN();
//      break;
//    case GPIOC_BASEADDR:
//      GPIOC_PCLOCK_EN();
//      break;
//    case GPIOD_BASEADDR:
//      GPIOD_PCLOCK_EN();
//      break;
//    case GPIOE_BASEADDR:
//      GPIOE_PCLOCK_EN();
//      break;
//    case GPIOF_BASEADDR:
//      GPIOF_PCLOCK_EN();
//      break;
//    case GPIOG_BASEADDR:
//      GPIOG_PCLOCK_EN();
//      break;
//    case GPIOH_BASEADDR:
//      GPIOH_PCLOCK_EN();
//      break;
//    default:
//      break;
//    }
//  } else {
//    switch ((uint32_t)pGPIOx) {
//    case GPIOA_BASEADDR:
//      GPIOA_PCLOCK_DI();
//      break;
//    case GPIOB_BASEADDR:
//      GPIOB_PCLOCK_DI();
//      break;
//    case GPIOC_BASEADDR:
//      GPIOC_PCLOCK_DI();
//      break;
//    case GPIOD_BASEADDR:
//      GPIOD_PCLOCK_DI();
//      break;
//    case GPIOE_BASEADDR:
//      GPIOE_PCLOCK_DI();
//      break;
//    case GPIOF_BASEADDR:
//      GPIOF_PCLOCK_DI();
//      break;
//    case GPIOG_BASEADDR:
//      GPIOG_PCLOCK_DI();
//      break;
//    case GPIOH_BASEADDR:
//      GPIOH_PCLOCK_DI();
//      break;
//    default:
//      break;
//    }
//  }
//}
//
///*
// * Init and De-init
// */
//void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
//{
//  uint32_t temp = 0;
//
//  if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
//    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//    pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//    pGPIOHandle->pGPIOx->MODER |= temp;
//  } else
//  {
//    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
//    {
//      EXTI->FTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//      EXTI->RTSR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//    } else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
//    {
//      EXTI->RTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//      EXTI->FTSR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//    } else
//    {
//      EXTI->FTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//      EXTI->RTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//    }
//
//    uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
//    uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
//    uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
//    SYSCFG_PCLOCK_EN();
//    SYSCFG->EXTICR[temp1] &= ~(0xF << 4 * temp2);
//    SYSCFG->EXTICR[temp1] |= (portcode << 4 * temp2);
//
//    EXTI->IMR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//  }
//
//  temp = 0;
//
//  temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//  pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//  pGPIOHandle->pGPIOx->OSPEEDR |= temp;
//
//  temp = 0;
//
//  temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//  pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//  pGPIOHandle->pGPIOx->PUPDR |= temp;
//
//  temp = 0;
//
//  temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
//  pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//  pGPIOHandle->pGPIOx->OTYPER |= temp;
//
//  if (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN) {
//    uint8_t temp1, temp2;
//    temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
//    temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
//    pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
//    pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
//  }
//}
//
//void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
//{
//  switch ((uint32_t)pGPIOx) {
//  case GPIOA_BASEADDR:
//    GPIOA_REG_RESET();
//    break;
//  case GPIOB_BASEADDR:
//    GPIOB_REG_RESET();
//    break;
//  case GPIOC_BASEADDR:
//    GPIOC_REG_RESET();
//    break;
//  case GPIOD_BASEADDR:
//    GPIOD_REG_RESET();
//    break;
//  case GPIOE_BASEADDR:
//    GPIOE_REG_RESET();
//    break;
//  case GPIOF_BASEADDR:
//    GPIOF_REG_RESET();
//    break;
//  case GPIOG_BASEADDR:
//    GPIOG_REG_RESET();
//    break;
//  case GPIOH_BASEADDR:
//    GPIOH_REG_RESET();
//    break;
//  default:
//    break;
//  }
//}
//
///*
// * Data Read and Write
// */
//uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
//{
//  uint8_t value;
//  value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
//  return value;
//}
//
//uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
//{
//  uint16_t value;
//  value = (uint16_t)pGPIOx->IDR;
//  return value;
//}
//
//void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
//{
//  if (Value == GPIO_PIN_SET)
//  {
//     pGPIOx->ODR |= (1 << PinNumber);
//  } else
//  {
//    pGPIOx->ODR &= ~(1 << PinNumber);
//  }
//}
//
//void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value)
//{
//  pGPIOx->ODR = Value;
//}
//
//void GPIO_ToggleToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
//{
//  pGPIOx->ODR ^= (1 << PinNumber);
//}
//
///*
// * IRQ Configuration and ISR handling
// */
//void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
//{
//  if (EnOrDi == ENABLE)
//  {
//    if (IRQNumber <= 31)
//    {
//      *NVIC_ISER0 |= (0x1 << IRQNumber);
//    } else if (IRQNumber > 31 && IRQNumber < 64)
//    {
//      *NVIC_ISER1 |= (0x1 << (IRQNumber % 32));
//    } else if (IRQNumber >= 64 &&  IRQNumber < 96)
//    {
//      *NVIC_ISER2 |= (0x1 << (IRQNumber % 32));
//    }
//  } else
//  {
//    if (IRQNumber <= 31)
//    {
//      *NVIC_ICER0 |= (0x1 << IRQNumber);
//    } else if (IRQNumber > 31 && IRQNumber < 64)
//    {
//      *NVIC_ICER1 |= (0x1 << (IRQNumber % 32));
//    } else if (IRQNumber >= 64 &&  IRQNumber < 96)
//    {
//      *NVIC_ICER2 |= (0x1 << (IRQNumber % 32));
//    }
//  }
//}
//
//void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
//{
//  uint8_t iprx = IRQNumber / 4;
//  uint8_t iprx_section = IRQNumber % 4;
//
//  uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
//  *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
////  *(NVIC_PR_BASEADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
//}
//
//void GPIO_IRQHandling(uint8_t PinNumber)
//{
//  if (EXTI->PR & (0x1 << PinNumber))
//  {
//    EXTI->PR |= (0x1 << PinNumber);
//  }
//}
