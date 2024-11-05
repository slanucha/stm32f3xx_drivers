/*
 * 008_USART_Testing.c
 *
 *  Created on: Nov 4, 2024
 *      Author: slan
 */

#include "stm32f303xx.h"
#include <string.h>

USART_Handle_t usart2_handle;

static void GPIO_Pins_Init(void)
{
  GPIO_Handle_t usart_pins;

  usart_pins.pGPIOx = GPIOA;
  usart_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  usart_pins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
  usart_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  usart_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;
  usart_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

  // USART CTS
  usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
  GPIO_Init(&usart_pins);

  // USART RTS
  usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
  GPIO_Init(&usart_pins);

  // USART TX
  usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
  GPIO_Init(&usart_pins);

  // USART RX
  usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_11;
  GPIO_Init(&usart_pins);

  // USART CK
  usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
  GPIO_Init(&usart_pins);
}

static void USART_Handle_Init(void)
{
  usart2_handle.pUSARTx = USART1;
  usart2_handle.TxState = USART_READY;
  usart2_handle.RxState = USART_READY;

  usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
  usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
  usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
  usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
  usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
  usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
}

int main(void)
{
  GPIO_Pins_Init();
  USART_Handle_Init();

  USART_Init(&usart2_handle);
  USART_PeripheralControl(usart2_handle.pUSARTx, ENABLE);

  char message[] = "Hello World!\r\n";
//  USART_SendData(usart2_handle.pUSARTx, (uint8_t*)message, strlen(message));

  USART_IRQInterruptConfig(IRQ_NO_USART1, ENABLE);
  while (!USART_SendDataIT(&usart2_handle, (uint8_t*)message, strlen(message)));

  while(1);
  return 0;
}

void USART1_EXTI25_IRQHandler(void)
{
  USART_IRQHandling(&usart2_handle);
}
