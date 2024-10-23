  /*
 * 001_LED_Toggle.c
 *
 *  Created on: Oct 21, 2024
 *      Author: slan
 */

#include "stm32f303xx.h"
#include "stm32f303xx_gpio_driver.h"

void delay(void)
{
  for (uint32_t i = 0; i < 750000; ++i);
}

int main(void)
{
  GPIO_Handle_t gpioLed;

  gpioLed.pGPIOx = GPIOA;
  gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
  gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

  GPIO_PeriClockControl(GPIOA, ENABLE);
  GPIO_Init(&gpioLed);

  for(;;)
  {
    GPIO_ToggleToOutputPin(gpioLed.pGPIOx, gpioLed.GPIO_PinConfig.GPIO_PinNumber);
    delay();
  }
  return 0;
}
