/*
 * stm32f303xx_usart_driver.h
 *
 *  Created on: Nov 4, 2024
 *      Author: slan
 */

#ifndef INC_STM32F303XX_USART_DRIVER_H_
#define INC_STM32F303XX_USART_DRIVER_H_

#include "stm32f303xx.h"

/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
  uint8_t USART_Mode;
  uint32_t USART_Baud;
  uint8_t USART_NoOfStopBits;
  uint8_t USART_WordLength;
  uint8_t USART_ParityControl;
  uint8_t USART_HWFlowControl;
} USART_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
  USART_RegDef_t *pUSARTx;
  USART_Config_t USART_Config;
  uint8_t *pTxBuffer;
  uint8_t *pRxBuffer;
  uint32_t TxLen;
  uint32_t RxLen;
  uint8_t TxState;
  uint8_t RxState;
} USART_Handle_t;

/*
 * @USART_Mode
 */
#define USART_MODE_ONLY_TX                0
#define USART_MODE_ONLY_RX                1
#define USART_MODE_TXRX                   2

/*
 * @USART_Baud
 */
#define USART_STD_BAUD_1200               1200
#define USART_STD_BAUD_2400               2400
#define USART_STD_BAUD_9600               9600
#define USART_STD_BAUD_19200              19200
#define USART_STD_BAUD_38400              38400
#define USART_STD_BAUD_57600              57600
#define USART_STD_BAUD_115200             115200
#define USART_STD_BAUD_230400             230400
#define USART_STD_BAUD_460800             460800
#define USART_STD_BAUD_921600             921600
#define USART_STD_BAUD_2M                 2M
#define USART_STD_BAUD_3M                 3M

/*
 * @USART_ParityControl
 */
#define USART_PARITY_EN_ODD               2
#define USART_PARITY_EN_EVEN              1
#define USART_PARITY_DISABLE              0

/*
 * @USART_WordLength
 */
#define USART_WORDLEN_8BITS               0
#define USART_WORDLEN_9BITS               1

/*
 * @USART_NoOfStopBits
 */
#define USART_STOPBITS_1                  0
#define USART_STOPBITS_0_5                1
#define USART_STOPBITS_2                  2
#define USART_STOPBITS_1_5                3

/*
 * @USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE            0
#define USART_HW_FLOW_CTRL_CTS             1
#define USART_HW_FLOW_CTRL_RTS             2
#define USART_HW_FLOW_CTRL_CTS_RTS         3

/*
 * USART related status flags definitions
 */
#define USART_FLAG_RXNE                  (0x1 << USART_ISR_RXNE)
#define USART_FLAG_TXE                  (0x1 << USART_ISR_TXE)
#define USART_FLAG_TC                   (0x1 << USART_ISR_TC)

#define USART_READY                     0
#define USART_BUSY_IN_TX                1
#define USART_BUSY_IN_RX                2

/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Data Read and Write
 */
void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t len);

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent);

#endif /* INC_STM32F303XX_USART_DRIVER_H_ */
