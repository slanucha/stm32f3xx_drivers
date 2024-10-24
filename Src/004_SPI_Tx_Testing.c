/*
 * 004_SPI_Tx_Testing.c
 *
 *  Created on: Oct 23, 2024
 *      Author: slan
 */

#include "stm32f303xx.h"
#include <string.h>

void SPI2_GPIO_Inits(void)
{
  GPIO_Handle_t SPIPins;

  SPIPins.pGPIOx = GPIOB;
  SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
  SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;

  GPIO_Init(&SPIPins);

  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
  GPIO_Init(&SPIPins);

  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
  GPIO_Init(&SPIPins);

  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
  GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
  SPI_Handle_t SPI2Handle;

  SPI2Handle.pSPIx = SPI2;
  SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
  SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
  SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
  SPI2Handle.SPI_Config.SPI_DS = SPI_DS_8BIT;
  SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
  SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
  SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;

  SPI_Init(&SPI2Handle);
}

int main(void)
{
  char user_data[] = "Hello World";

  SPI2_GPIO_Inits();
  SPI2_Inits();

  SPI_SSIConfig(SPI2, ENABLE);
  SPI_PeripheralControl(SPI2, ENABLE);

  SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

  while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
  SPI_PeripheralControl(SPI2, DISABLE);

  return 0;
}
