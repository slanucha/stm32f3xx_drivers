/*
 * 003_LED_Button_Interrupt.c
 *
 *  Created on: Oct 22, 2024
 *      Author: slan
 */

#include "stm32f303xx.h"
#include "stm32f303xx_gpio_driver.h"

void delay(void)
{
  for (uint32_t i = 0; i < 200000; ++i);
}

int main(void)
{
  GPIO_Handle_t gpioLed;
  GPIO_Handle_t gpioBtn;
  memset(&gpioLed, 0, sizeof(GPIO_Handle_t));
  memset(&gpioBtn, 0, sizeof(GPIO_Handle_t));

  gpioLed.pGPIOx = GPIOA;
  gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
  gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

  gpioBtn.pGPIOx = GPIOC;
  gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
  gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
  gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

  GPIO_PeriClockControl(GPIOA, ENABLE);
  GPIO_PeriClockControl(GPIOC, ENABLE);
  GPIO_Init(&gpioLed);
  GPIO_Init(&gpioBtn);

  GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
  GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

  for(;;);

  return 0;
}

void EXTI15_10_IRQHandler(void)
{
  delay();
  GPIO_IRQHandling(GPIO_PIN_13);
  GPIO_ToggleToOutputPin(GPIOA, GPIO_PIN_5);
}
