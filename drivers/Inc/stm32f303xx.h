/*
 * stm32f303xx.h
 *
 *  Created on: Oct 18, 2024
 *      Author: slan
 */

#ifndef INC_STM32F303XX_H_
#define INC_STM32F303XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0              ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1              ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2              ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3              ( (__vo uint32_t*)0xE000E10C )

#define NVIC_ICER0              ( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1              ( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2              ( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3              ( (__vo uint32_t*)0xE000E18C )

#define NVIC_PR_BASEADDR        ( (__vo uint32_t*)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED  4

/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR          0x08000000U // 512KB
#define SRAM_BASEADDR           0x20000000U // 64KB

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASE             0x40000000U
#define APB1PERIPH_BASE         PERIPH_BASE
#define APB2PERIPH_BASE         0x40010000U
#define AHB1PERIPH_BASE         0x40020000U
#define AHB2PERIPH_BASE         0x48000000U
#define AHB3PERIPH_BASE         0x50000000U
#define AHB4PERIPH_BASE         0x60000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define DMA1_BASEADDR           ((AHB1PERIPH_BASE) + (0x0000U))
#define DMA2_BASEADDR           ((AHB1PERIPH_BASE) + (0x0400U))
#define RCC_BASEADDR            ((AHB1PERIPH_BASE) + (0x1000U))
#define FLASH_INT_BASEADDR      ((AHB1PERIPH_BASE) + (0x2000U))
#define CRC_BASEADDR            ((AHB1PERIPH_BASE) + (0x3000U))
#define TSC_BASEADDR            ((AHB1PERIPH_BASE) + (0x4000U))

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */
#define GPIOA_BASEADDR          ((AHB2PERIPH_BASE) + (0x0000U))
#define GPIOB_BASEADDR          ((AHB2PERIPH_BASE) + (0x0400U))
#define GPIOC_BASEADDR          ((AHB2PERIPH_BASE) + (0x0800U))
#define GPIOD_BASEADDR          ((AHB2PERIPH_BASE) + (0x0C00U))
#define GPIOE_BASEADDR          ((AHB2PERIPH_BASE) + (0x1000U))
#define GPIOF_BASEADDR          ((AHB2PERIPH_BASE) + (0x1400U))
#define GPIOG_BASEADDR          ((AHB2PERIPH_BASE) + (0x1800U))
#define GPIOH_BASEADDR          ((AHB2PERIPH_BASE) + (0x1C00U))

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define TIM2_BASEADDR           ((APB1PERIPH_BASE) + (0x0000U))
#define TIM3_BASEADDR           ((APB1PERIPH_BASE) + (0x0400U))
#define TIM4_BASEADDR           ((APB1PERIPH_BASE) + (0x0800U))
#define TIM6_BASEADDR           ((APB1PERIPH_BASE) + (0x1000U))
#define TIM7_BASEADDR           ((APB1PERIPH_BASE) + (0x1400U))

#define I2C1_BASEADDR           ((APB1PERIPH_BASE) + (0x5400U))
#define I2C2_BASEADDR           ((APB1PERIPH_BASE) + (0x5800U))
#define I2C3_BASEADDR           ((APB1PERIPH_BASE) + (0x7800U))

#define SPI2_BASEADDR           ((APB1PERIPH_BASE) + (0x3800U))
#define SPI3_BASEADDR           ((APB1PERIPH_BASE) + (0x3C00U))

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define SYSCFG_BASEADDR         ((APB2PERIPH_BASE) + (0x0000U))
#define EXTI_BASEADDR           ((APB2PERIPH_BASE) + (0x0400U))
#define SPI1_BASEADDR           ((APB2PERIPH_BASE) + (0x3000U))
#define SPI4_BASEADDR           ((APB2PERIPH_BASE) + (0x3C00U))

/*
 * Peripheral Register Definition Structures
 */
typedef struct
{
  __vo uint32_t MODER;
  __vo uint32_t OTYPER;
  __vo uint32_t OSPEEDR;
  __vo uint32_t PUPDR;
  __vo uint32_t IDR;
  __vo uint32_t ODR;
  __vo uint32_t BSRR;
  __vo uint32_t LCKR;
  __vo uint32_t AFR[2];
  __vo uint32_t BR;
} GPIO_RegDef_t;

typedef struct
{
  __vo uint32_t CR;
  __vo uint32_t CFGR;
  __vo uint32_t CIR;
  __vo uint32_t APB2RSTR;
  __vo uint32_t APB1RSTR;
  __vo uint32_t AHBENR;
  __vo uint32_t APB2ENR;
  __vo uint32_t APB1ENR;
  __vo uint32_t BDCR;
  __vo uint32_t CSR;
  __vo uint32_t AHBRSTR;
  __vo uint32_t CFGR2;
  __vo uint32_t CFGR3;
} RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct {
  __vo uint32_t IMR;
  __vo uint32_t EMR;
  __vo uint32_t RTSR;
  __vo uint32_t FTSR;
  __vo uint32_t SWIER;
  __vo uint32_t PR;
} EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct {
  __vo uint32_t CFGR1;
  __vo uint32_t RCR;
  __vo uint32_t EXTICR[4];
  __vo uint32_t CFGR2;
} SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for SPIx
 */
typedef struct {
  __vo uint32_t CR1;
  __vo uint32_t CR2;
  __vo uint32_t SR;
  __vo uint32_t DR;
  __vo uint32_t CRCPR;
  __vo uint32_t RXCRCR;
  __vo uint32_t TXCRCR;
  __vo uint32_t I2SCFGR;
  __vo uint32_t I2SCFPR;
} SPI_RegDef_t;

/*
 * Peripheral definitions
 */
#define GPIOA                   ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                   ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                   ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                   ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                   ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                   ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                   ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH                   ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC                     ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI                    ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG                  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1                    ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                    ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                    ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4                    ((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLOCK_EN()       (RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLOCK_EN()       (RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLOCK_EN()       (RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLOCK_EN()       (RCC->AHBENR |= (1 << 20))
#define GPIOE_PCLOCK_EN()       (RCC->AHBENR |= (1 << 21))
#define GPIOF_PCLOCK_EN()       (RCC->AHBENR |= (1 << 22))
#define GPIOG_PCLOCK_EN()       (RCC->AHBENR |= (1 << 23))
#define GPIOH_PCLOCK_EN()       (RCC->AHBENR |= (1 << 16))

/*
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLOCK_EN()        (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLOCK_EN()        (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLOCK_EN()        (RCC->APB1ENR |= (1 << 30))

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLOCK_EN()        (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLOCK_EN()        (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLOCK_EN()        (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLOCK_EN()        (RCC->APB2ENR |= (1 << 15))

/*
 * Clock enable macros for USARTx peripherals
 */
#define USART1_PCLOCK_EN()      (RCC->APB2ENR |= (1 << 14))
#define USART2_PCLOCK_EN()      (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLOCK_EN()      (RCC->APB1ENR |= (1 << 18))
#define USART4_PCLOCK_EN()      (RCC->APB1ENR |= (1 << 19))
#define USART5_PCLOCK_EN()      (RCC->APB1ENR |= (1 << 20))

/*
 * Clock enable macro for SYSCFG peripheral
 */
#define SYSCFG_PCLOCK_EN()      (RCC->APB2ENR |= (1 << 0))

/*
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLOCK_DI()       (RCC->AHBENR &= ~(1 << 17))
#define GPIOB_PCLOCK_DI()       (RCC->AHBENR &= ~(1 << 18))
#define GPIOC_PCLOCK_DI()       (RCC->AHBENR &= ~(1 << 19))
#define GPIOD_PCLOCK_DI()       (RCC->AHBENR &= ~(1 << 20))
#define GPIOE_PCLOCK_DI()       (RCC->AHBENR &= ~(1 << 21))
#define GPIOF_PCLOCK_DI()       (RCC->AHBENR &= ~(1 << 22))
#define GPIOG_PCLOCK_DI()       (RCC->AHBENR &= ~(1 << 23))
#define GPIOH_PCLOCK_DI()       (RCC->AHBENR &= ~(1 << 16))

/*
 * Clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLOCK_DI()        (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLOCK_DI()        (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLOCK_DI()        (RCC->APB1ENR &= ~(1 << 30))

/*
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PCLOCK_DI()        (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLOCK_DI()        (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLOCK_DI()        (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLOCK_DI()        (RCC->APB2ENR &= ~(1 << 15))

/*
 * Clock disable macros for USARTx peripherals
 */
#define USART1_PCLOCK_DI()      (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLOCK_DI()      (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLOCK_DI()      (RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLOCK_DI()      (RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLOCK_DI()      (RCC->APB1ENR &= ~(1 << 20))

/*
 * Clock disable macro for SYSCFG peripheral
 */
#define SYSCFG_PCLOCK_DI()      (RCC->APB2ENR &= ~(1 << 0))

/*
 * Reset macros for GPIOx peripherals
 */
#define GPIOA_REG_RESET()       do { (RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17)); } while(0)
#define GPIOB_REG_RESET()       do { (RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18)); } while(0)
#define GPIOC_REG_RESET()       do { (RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19)); } while(0)
#define GPIOD_REG_RESET()       do { (RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20)); } while(0)
#define GPIOE_REG_RESET()       do { (RCC->AHBRSTR |= (1 << 21)); (RCC->AHBRSTR &= ~(1 << 21)); } while(0)
#define GPIOF_REG_RESET()       do { (RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22)); } while(0)
#define GPIOG_REG_RESET()       do { (RCC->AHBRSTR |= (1 << 23)); (RCC->AHBRSTR &= ~(1 << 23)); } while(0)
#define GPIOH_REG_RESET()       do { (RCC->AHBRSTR |= (1 << 16)); (RCC->AHBRSTR &= ~(1 << 16)); } while(0)

#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
                                        (x == GPIOB)?1:\
                                        (x == GPIOC)?2:\
                                        (x == GPIOD)?3:\
                                        (x == GPIOE)?4:\
                                        (x == GPIOF)?5:\
                                        (x == GPIOG)?6:0 )

/*
 * IRQ Number of STM32F303x MCU
 */
#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1            7
#define IRQ_NO_EXTI2            8
#define IRQ_NO_EXTI3            9
#define IRQ_NO_EXTI4            10
#define IRQ_NO_EXTI9_5          23
#define IRQ_NO_EXTI15_10        40

#define NVIC_IRQ_PRI0           0
#define NVIC_IRQ_PRI1           1
#define NVIC_IRQ_PRI2           2
#define NVIC_IRQ_PRI3           3
#define NVIC_IRQ_PRI4           4
#define NVIC_IRQ_PRI5           5
#define NVIC_IRQ_PRI6           6
#define NVIC_IRQ_PRI7           7
#define NVIC_IRQ_PRI8           8
#define NVIC_IRQ_PRI9           9
#define NVIC_IRQ_PRI10          10
#define NVIC_IRQ_PRI11          11
#define NVIC_IRQ_PRI12          12
#define NVIC_IRQ_PRI13          13
#define NVIC_IRQ_PRI14          14
#define NVIC_IRQ_PRI15          15

/*
 * Some generic macros
 */
#define ENABLE                  1
#define DISABLE                 0
#define SET                     1
#define RESET                   0
#define GPIO_PIN_SET            SET
#define GPIO_PIN_RESET          RESET

#endif /* INC_STM32F303XX_H_ */
