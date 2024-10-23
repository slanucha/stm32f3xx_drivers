/*
 * stm32f303xx_spi_driver.h
 *
 *  Created on: Oct 23, 2024
 *      Author: slan
 */

#ifndef INC_STM32F303XX_SPI_DRIVER_H_
#define INC_STM32F303XX_SPI_DRIVER_H_

#include "stm32f303xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct {
  uint8_t SPI_DeviceMode;
  uint8_t SPI_BusConfig;
  uint8_t SPI_SclkSpeed;
  uint8_t SPI_DFF;
  uint8_t SPI_CPOL;
  uint8_t SPI_CPHA;
  uint8_t SPI_SSM;
} SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct {
  SPI_RegDef_t *pSPIx;
  SPI_Config_t SPI_Config;
} SPI_Handle_t;

#endif /* INC_STM32F303XX_SPI_DRIVER_H_ */
