/*
 * stm32f303xx_rcc_driver.c
 *
 *  Created on: Nov 4, 2024
 *      Author: slan
 */

#include "stm32f303xx_rcc_driver.h"

static const uint16_t HCLK_DIV[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
static const uint8_t APB_DIV[4] = { 2, 4, 8, 16 };

uint32_t RCC_GetPCLK1Value(void)
{
  uint32_t sysClk, PClk1;
  uint32_t hclkPre, apb1Pre;
  uint8_t temp;

  sysClk = 8000000; // Only HSI clock support is implemented

  temp = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF);
  hclkPre = (temp < 8) ? 1 : HCLK_DIV[temp - 8];

  temp = ((RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7);
  apb1Pre = (temp < 4) ? 1 : APB_DIV[temp - 4];

  PClk1 = sysClk / hclkPre / apb1Pre;

  return PClk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
  uint32_t sysClk, PClk2;
  uint32_t hclkPre, apb2Pre;
  uint8_t temp;

  sysClk = 8000000; // Only HSI clock support is implemented

  temp = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF);
  hclkPre = (temp < 8) ? 1 : HCLK_DIV[temp - 8];

  temp = ((RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7);
  apb2Pre = (temp < 4) ? 1 : APB_DIV[temp - 4];

  PClk2 = sysClk / hclkPre / apb2Pre;

  return PClk2;
}
