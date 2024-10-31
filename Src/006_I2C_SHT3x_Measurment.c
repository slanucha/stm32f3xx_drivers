/*
 * 005_I2C_Testing.c
 *
 *  Created on: Oct 30, 2024
 *      Author: slan
 */

#include "stm32f303xx.h"

#define SHT3X_ADDR              0x44

#define SHT3X_MSB_CS            0x2C
#define SHT3X_LSB_CS_H          0x06
#define SHT3X_LSB_CS_M          0x0D
#define SHT3X_LSB_CS_L          0x10

#define SHT3X_MSB               0x24
#define SHT3X_LSB_H             0x00
#define SHT3X_LSB_M             0x0B
#define SHT3X_LSB_L             0x16

I2C_Handle_t I2CHandle;
GPIO_Handle_t gpioBtn;

static void delay(void)
{
  for (uint32_t i = 0; i < 300000; ++i);
}

static void GPIO_Btn_Inits(void)
{
  gpioBtn.pGPIOx = GPIOC;
  gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
  gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
  gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

  GPIO_Init(&gpioBtn);
}

static void GPIO_I2C_Inits(void)
{
  GPIO_Handle_t SPIPins;

  SPIPins.pGPIOx = GPIOB;
  SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
  SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
  SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
  SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
  GPIO_Init(&SPIPins);

  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
  GPIO_Init(&SPIPins);
}

static void I2C_Inits(void)
{
  I2CHandle.pI2Cx = I2C1;
  I2CHandle.I2C_Config.I2C_DeviceAddress = 0;
  I2CHandle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM2;

  I2C_Init(&I2CHandle);
}

int main(void)
{
  GPIO_Btn_Inits();
  GPIO_I2C_Inits();
  I2C_Inits();

  I2C_PeripheralControl(I2CHandle.pI2Cx, ENABLE);

  uint8_t cmd[] = { SHT3X_MSB_CS, SHT3X_LSB_CS_M };
  uint8_t data[6];
  double temp_c = 0;

  for(;;)
  {
    if (GPIO_ReadFromInputPin(gpioBtn.pGPIOx, gpioBtn.GPIO_PinConfig.GPIO_PinNumber) == 0)
    {
      delay();
      I2C_MasterSendData(&I2CHandle, cmd, 2, SHT3X_ADDR);
      delay();
      I2C_MasterReceiveData(&I2CHandle, data, 6, SHT3X_ADDR);

      uint16_t tempData = 0;
      tempData |= data[0] << 8;
      tempData |= data[1];
      temp_c = tempData * 175.0 / 65535.0 - 45.0;
      break;
    }
  }
  (void)temp_c;

  I2C_PeripheralControl(I2CHandle.pI2Cx, DISABLE);
  I2C_DeInit(I2CHandle.pI2Cx);

  return 0;
}
