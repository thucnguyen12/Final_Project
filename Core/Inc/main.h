/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f2xx_hal.h"

#include "stm32f2xx_ll_dma.h"
#include "stm32f2xx_ll_usart.h"
#include "stm32f2xx_ll_rcc.h"
#include "stm32f2xx_ll_bus.h"
#include "stm32f2xx_ll_cortex.h"
#include "stm32f2xx_ll_system.h"
#include "stm32f2xx_ll_utils.h"
#include "stm32f2xx_ll_pwr.h"
#include "stm32f2xx_ll_gpio.h"

#include "stm32f2xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
uint32_t sys_get_ms(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_D0_Pin GPIO_PIN_2
#define LCD_D0_GPIO_Port GPIOE
#define LCD_D1_Pin GPIO_PIN_3
#define LCD_D1_GPIO_Port GPIOE
#define LCD_D2_Pin GPIO_PIN_4
#define LCD_D2_GPIO_Port GPIOE
#define LCD_D3_Pin GPIO_PIN_5
#define LCD_D3_GPIO_Port GPIOE
#define LCD_D4_Pin GPIO_PIN_6
#define LCD_D4_GPIO_Port GPIOE
#define LCD_D5_Pin GPIO_PIN_13
#define LCD_D5_GPIO_Port GPIOC
#define LCD_D6_Pin GPIO_PIN_14
#define LCD_D6_GPIO_Port GPIOC
#define LCD_D7_Pin GPIO_PIN_15
#define LCD_D7_GPIO_Port GPIOC
#define RST_LCD_Pin GPIO_PIN_0
#define RST_LCD_GPIO_Port GPIOF
#define LED1_G_Pin GPIO_PIN_1
#define LED1_G_GPIO_Port GPIOF
#define LED1_R_Pin GPIO_PIN_2
#define LED1_R_GPIO_Port GPIOF
#define LED2_R_Pin GPIO_PIN_3
#define LED2_R_GPIO_Port GPIOF
#define LED2_G_Pin GPIO_PIN_4
#define LED2_G_GPIO_Port GPIOF
#define BT_IN_1_Pin GPIO_PIN_5
#define BT_IN_1_GPIO_Port GPIOF
#define BT_IN_2_Pin GPIO_PIN_6
#define BT_IN_2_GPIO_Port GPIOF
#define GSM_PWRKEY_Pin GPIO_PIN_7
#define GSM_PWRKEY_GPIO_Port GPIOF
#define GSM_RESET_Pin GPIO_PIN_8
#define GSM_RESET_GPIO_Port GPIOF
#define WDI_Pin GPIO_PIN_9
#define WDI_GPIO_Port GPIOF
#define PWM_LCD_Pin GPIO_PIN_3
#define PWM_LCD_GPIO_Port GPIOA
#define ADC_VBATRF_Pin GPIO_PIN_4
#define ADC_VBATRF_GPIO_Port GPIOA
#define ADC_1V8_Pin GPIO_PIN_5
#define ADC_1V8_GPIO_Port GPIOA
#define ADC_3V3_Pin GPIO_PIN_6
#define ADC_3V3_GPIO_Port GPIOA
#define ADC_VSYS_Pin GPIO_PIN_0
#define ADC_VSYS_GPIO_Port GPIOB
#define ADC_VBAT_Pin GPIO_PIN_1
#define ADC_VBAT_GPIO_Port GPIOB
#define MAIN_PW_Pin GPIO_PIN_2
#define MAIN_PW_GPIO_Port GPIOB
#define PRE_PW_Pin GPIO_PIN_11
#define PRE_PW_GPIO_Port GPIOF
#define RELAY_NO_Pin GPIO_PIN_13
#define RELAY_NO_GPIO_Port GPIOF
#define RELAY_NO_EXTI_IRQn EXTI15_10_IRQn
#define RELAY_NC_Pin GPIO_PIN_14
#define RELAY_NC_GPIO_Port GPIOF
#define RELAY_NC_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_Pin GPIO_PIN_15
#define BUTTON_GPIO_Port GPIOF
#define ALARM_IN_Pin GPIO_PIN_0
#define ALARM_IN_GPIO_Port GPIOG
#define FAULT_IN_Pin GPIO_PIN_1
#define FAULT_IN_GPIO_Port GPIOG
#define IN_LB4_Pin GPIO_PIN_7
#define IN_LB4_GPIO_Port GPIOE
#define IN_LR5_Pin GPIO_PIN_8
#define IN_LR5_GPIO_Port GPIOE
#define IN_LB5_Pin GPIO_PIN_11
#define IN_LB5_GPIO_Port GPIOE
#define IN_LB6_Pin GPIO_PIN_12
#define IN_LB6_GPIO_Port GPIOE
#define IN_LR6_Pin GPIO_PIN_13
#define IN_LR6_GPIO_Port GPIOE
#define ESP_EN_Pin GPIO_PIN_14
#define ESP_EN_GPIO_Port GPIOE
#define ESP_IO0_Pin GPIO_PIN_15
#define ESP_IO0_GPIO_Port GPIOE
#define ESP_RX_Pin GPIO_PIN_10
#define ESP_RX_GPIO_Port GPIOB
#define ESP_TX_Pin GPIO_PIN_11
#define ESP_TX_GPIO_Port GPIOB
#define JIG_CS_Pin GPIO_PIN_15
#define JIG_CS_GPIO_Port GPIOB
#define EN_STWD_Pin GPIO_PIN_8
#define EN_STWD_GPIO_Port GPIOD
#define STATUS_Pin GPIO_PIN_9
#define STATUS_GPIO_Port GPIOD
#define MODE1_Pin GPIO_PIN_12
#define MODE1_GPIO_Port GPIOD
#define MODE2_Pin GPIO_PIN_13
#define MODE2_GPIO_Port GPIOD
#define IN_LR4_Pin GPIO_PIN_14
#define IN_LR4_GPIO_Port GPIOD
#define IN_LB3_Pin GPIO_PIN_15
#define IN_LB3_GPIO_Port GPIOD
#define IN_LR3_Pin GPIO_PIN_2
#define IN_LR3_GPIO_Port GPIOG
#define IN_LB2_Pin GPIO_PIN_3
#define IN_LB2_GPIO_Port GPIOG
#define IN_LR2_Pin GPIO_PIN_4
#define IN_LR2_GPIO_Port GPIOG
#define IN_LB1_Pin GPIO_PIN_5
#define IN_LB1_GPIO_Port GPIOG
#define IN_LR1_Pin GPIO_PIN_6
#define IN_LR1_GPIO_Port GPIOG
#define TX6_Pin GPIO_PIN_6
#define TX6_GPIO_Port GPIOC
#define RX6_Pin GPIO_PIN_7
#define RX6_GPIO_Port GPIOC
#define DIO_PCB_Pin GPIO_PIN_8
#define DIO_PCB_GPIO_Port GPIOC
#define CLK_PCB_Pin GPIO_PIN_9
#define CLK_PCB_GPIO_Port GPIOC
#define RS485_Pin GPIO_PIN_8
#define RS485_GPIO_Port GPIOA
#define TX1_Pin GPIO_PIN_9
#define TX1_GPIO_Port GPIOA
#define RX1_Pin GPIO_PIN_10
#define RX1_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define GSM_RX_Pin GPIO_PIN_10
#define GSM_RX_GPIO_Port GPIOC
#define GSM_TX_Pin GPIO_PIN_11
#define GSM_TX_GPIO_Port GPIOC
#define BUZZ_Pin GPIO_PIN_3
#define BUZZ_GPIO_Port GPIOD
#define W_ENET_RST_Pin GPIO_PIN_12
#define W_ENET_RST_GPIO_Port GPIOG
#define EN_LCD_Pin GPIO_PIN_8
#define EN_LCD_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_9
#define LCD_EN_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_0
#define LCD_RW_GPIO_Port GPIOE
#define LCD_DI_Pin GPIO_PIN_1
#define LCD_DI_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
