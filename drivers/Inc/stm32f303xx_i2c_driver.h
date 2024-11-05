/*
 * stm32f303xx_i2c_driver.h
 *
 *  Created on: Oct 29, 2024
 *      Author: slan
 */

#ifndef INC_STM32F303XX_I2C_DRIVER_H_
#define INC_STM32F303XX_I2C_DRIVER_H_

#include "stm32f303xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
  uint32_t I2C_SCLSpeed;
  uint8_t I2C_DeviceAddress;
  uint8_t I2C_ACKControl;
} I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
  I2C_RegDef_t *pI2Cx;
  I2C_Config_t I2C_Config;
  uint8_t *pTxBuffer;
  uint8_t *pRxBuffer;
  uint32_t TxLen;
  uint32_t RxLen;
  uint8_t TxRxState;
  uint8_t DevAddr;
} I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 * Supported timings in kHz for 8 MHz clock
 */
#define I2C_SCL_SPEED_SM1           10
#define I2C_SCL_SPEED_SM2           100
#define I2C_SCL_SPEED_FM            400
#define I2C_SCL_SPEED_FM_PLUS       500

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE                  1
#define I2C_ACK_DISABLE                 0

/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_BUSY                   (0x1 << I2C_ISR_BUSY)
#define I2C_FLAG_TXIS                   (0x1 << I2C_ISR_TXIS)
#define I2C_FLAG_TXE                    (0x1 << I2C_ISR_TXE)
#define I2C_FLAG_TCR                    (0x1 << I2C_ISR_TCR)
#define I2C_FLAG_TC                     (0x1 << I2C_ISR_TC)
#define I2C_FLAG_RXNE                   (0x1 << I2C_ISR_RXNE)

/*
 * SPI related status flags definitions
 */
#define I2C_DATA_READ                   0x1
#define I2C_DATA_WRITE                  0x0

#define I2C_READY                       0
#define I2C_BUSY_IN_TX                  1
#define I2C_BUSY_IN_RX                  2

/*
 * Possible SPI application events
 */
#define I2C_EVENT_TX_CMPLT              1
#define I2C_EVENT_RX_CMPLT              2
#define I2C_EVENT_STOP                  3

/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Read and Write
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t I2CEvent);

#endif /* INC_STM32F303XX_I2C_DRIVER_H_ */
