/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_TagType4.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MX_SPI1_Init_NFC(void);
uint16_t ReadGeoL(sGeoInfo *pGeoStruct);
uint16_t setGPOState(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_0
#define LED3_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define RFDIS_Pin GPIO_PIN_6
#define RFDIS_GPIO_Port GPIOC
#define SP1_GPIO3_Pin GPIO_PIN_7
#define SP1_GPIO3_GPIO_Port GPIOC
#define SP1_GPIO3_EXTI_IRQn EXTI4_15_IRQn
#define SP1_SDN_Pin GPIO_PIN_10
#define SP1_SDN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_4
#define RED_LED_GPIO_Port GPIOB
#define CSn_Pin GPIO_PIN_6
#define CSn_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define MX_SPI1_Init	RadioSpiInit

#define GPO_Pin GPIO_PIN_10
#define GPO_GPIO_Port GPIOC
#define GPO_EXTI_IRQn EXTI4_15_IRQn

/* M24SR GPIO mapping -------------------------------------------------------------------------*/
#define M24SR_SDA_PIN 										GPIO_PIN_9
#define M24SR_SDA_PIN_PORT 										GPIOB
#define M24SR_SCL_PIN 										GPIO_PIN_8
#define M24SR_SCL_PIN_PORT 										GPIOB
#define M24SR_GPO_PIN 										GPO_Pin
#define M24SR_GPO_PIN_PORT 										GPO_GPIO_Port
#define M24SR_RFDIS_PIN 									RFDIS_Pin
#define M24SR_RFDIS_PIN_PORT 									RFDIS_GPIO_Port


#if (defined USE_STM32F0XX_NUCLEO || defined USE_STM32F1XX_NUCLEO || defined USE_STM32F3XX_NUCLEO)
#define __GPIOA_CLK_ENABLE() 						__HAL_RCC_GPIOA_CLK_ENABLE()
#define __GPIOB_CLK_ENABLE() 						__HAL_RCC_GPIOB_CLK_ENABLE()
#define INIT_CLK_GPO_RFD() 							__HAL_RCC_GPIOA_CLK_ENABLE()
#define I2Cx_CLK_ENABLE()                   		__HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()       			__HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()       			__HAL_RCC_GPIOB_CLK_ENABLE()
#define I2C1_FORCE_RESET()                          __HAL_RCC_I2C1_FORCE_RESET()
#define I2C1_RELEASE_RESET()                        __HAL_RCC_I2C1_RELEASE_RESET()
#else
#define INIT_CLK_GPO_RFD() 							__GPIOA_CLK_ENABLE()
#define I2Cx_CLK_ENABLE()                   		__I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()       			__GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()       			__GPIOB_CLK_ENABLE()
#define I2C1_FORCE_RESET()               			__I2C1_FORCE_RESET()
#define I2C1_RELEASE_RESET()             			__I2C1_RELEASE_RESET()
#endif
/* 	I2C config	------------------------------------------------------------------------------*/
#define M24SR_I2C                  					       I2C1
/* I2C functionality is not mapped on the same Alternate function regarding the MCU used */
#if (defined USE_STM32F4XX_NUCLEO) || (defined USE_STM32F3XX_NUCLEO) || \
    (defined USE_STM32L0XX_NUCLEO) || (defined USE_STM32L1XX_NUCLEO) || (defined USE_STM32L4XX_NUCLEO)
  #define I2Cx_SCL_AF 											      	GPIO_AF4_I2C1
#elif (defined USE_STM32F0XX_NUCLEO)
	#define I2Cx_SCL_AF 											      	GPIO_AF1_I2C1
#elif (defined USE_STM32F1XX_NUCLEO)
  /* Not supported */
#endif
/* I2C SPEED
 * F4 uses directly the speed (100,400) and F0, L0, L3 use the TIMMINGR register defined below */
#if (defined USE_STM32F4XX_NUCLEO) || (defined USE_STM32F1XX_NUCLEO) || \
    (defined USE_STM32L1XX_NUCLEO)
	#define M24SR_I2C_SPEED_10													10000
	#define M24SR_I2C_SPEED_100													100000
	#define M24SR_I2C_SPEED_400													400000
	#define M24SR_I2C_SPEED_1000												1000000
/* Timing samples with PLLCLK 48MHz set in SystemClock_Config(), I2C CLK on SYSCLK value computed with CubeMx */
#elif (defined USE_STM32F0XX_NUCLEO)
	#define M24SR_I2C_SPEED_10													0x9010DEFF
	#define M24SR_I2C_SPEED_100													0x20303E5D
	#define M24SR_I2C_SPEED_400													0x2010091A
	#define M24SR_I2C_SPEED_1000												0x00200818
/* Timing samples with PLLCLK 32MHz set in SystemClock_Config(), I2C CLK on SYSCLK value computed with CubeMx */
#elif (defined USE_STM32L0XX_NUCLEO)
	#define M24SR_I2C_SPEED_10													0x6010C7FF
	#define M24SR_I2C_SPEED_100													0x00707CBB
	#define M24SR_I2C_SPEED_400													0x00300F38
	#define M24SR_I2C_SPEED_1000												0x00100413
/* Timing samples with PLLCLK 64MHz set in SystemClock_Config(), I2C CLK on SYSCLK value computed with CubeMx */
#elif (defined USE_STM32F3XX_NUCLEO)
	#define M24SR_I2C_SPEED_10													0xE010A9FF
	#define M24SR_I2C_SPEED_100													0x10707DBC
	#define M24SR_I2C_SPEED_400													0x00602173
	#define M24SR_I2C_SPEED_1000												0x00300B29
#elif (defined USE_STM32L4XX_NUCLEO)
	#define M24SR_I2C_SPEED_10													0xF000F3FE /* Clock 80MHz, Fast Mode, Analog Filter ON, Rise time 25ns, Fall time 10ns */
	#define M24SR_I2C_SPEED_100													0x203012F1 /* Clock 80Mhz, Fast Mode, Analog Filter ON, Rise time 50ns, Fall time 10ns */
	#define M24SR_I2C_SPEED_400													0x00B0298B /* Clock 80Mhz, Fast Mode, Analog Filter ON, Rise time 50ns, Fall time 25ns */
	#define M24SR_I2C_SPEED_1000												0x00700E2E /* Clock 80Mhz, Fast Mode Plus, Analog Filter ON, Rise time 50ns, Fall time 25ns */
#else
	#error "You need to update your code to this new microcontroller"
#endif


#define M24SR_I2C_SPEED				M24SR_I2C_SPEED_400

#define M24SR_I2CInit MX_I2C1_Init
#define M24SR_GPOInit MX_GPIO_Init_NFC
#define wait_ms(time) HAL_Delay(time)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
