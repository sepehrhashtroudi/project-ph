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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Up_Pin GPIO_PIN_2
#define Up_GPIO_Port GPIOE
#define Up_EXTI_IRQn EXTI2_IRQn
#define Left_Pin GPIO_PIN_3
#define Left_GPIO_Port GPIOE
#define Left_EXTI_IRQn EXTI3_IRQn
#define Esc_Pin GPIO_PIN_4
#define Esc_GPIO_Port GPIOE
#define Esc_EXTI_IRQn EXTI4_IRQn
#define Down_Pin GPIO_PIN_5
#define Down_GPIO_Port GPIOE
#define Down_EXTI_IRQn EXTI9_5_IRQn
#define modbus_led_Pin GPIO_PIN_1
#define modbus_led_GPIO_Port GPIOA
#define Status_led_Pin GPIO_PIN_2
#define Status_led_GPIO_Port GPIOA
#define pH_Pin GPIO_PIN_6
#define pH_GPIO_Port GPIOA
#define TEMP_Pin GPIO_PIN_7
#define TEMP_GPIO_Port GPIOA
#define REL_4_Pin GPIO_PIN_9
#define REL_4_GPIO_Port GPIOE
#define REL_3_Pin GPIO_PIN_10
#define REL_3_GPIO_Port GPIOE
#define REL_2_Pin GPIO_PIN_11
#define REL_2_GPIO_Port GPIOE
#define REL_1_Pin GPIO_PIN_12
#define REL_1_GPIO_Port GPIOE
#define RS485_RTS_Pin GPIO_PIN_14
#define RS485_RTS_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_10
#define LCD_RS_GPIO_Port GPIOC
#define LCD_RW_Pin GPIO_PIN_11
#define LCD_RW_GPIO_Port GPIOC
#define LCD_EN_Pin GPIO_PIN_12
#define LCD_EN_GPIO_Port GPIOC
#define LCD_D0_Pin GPIO_PIN_0
#define LCD_D0_GPIO_Port GPIOD
#define LCD_D1_Pin GPIO_PIN_1
#define LCD_D1_GPIO_Port GPIOD
#define LCD_D2_Pin GPIO_PIN_2
#define LCD_D2_GPIO_Port GPIOD
#define LCD_D3_Pin GPIO_PIN_3
#define LCD_D3_GPIO_Port GPIOD
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOD
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOD
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOD
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOD
#define LCD_CS1_Pin GPIO_PIN_3
#define LCD_CS1_GPIO_Port GPIOB
#define LCD_CS2_Pin GPIO_PIN_4
#define LCD_CS2_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_5
#define LCD_RST_GPIO_Port GPIOB
#define Right_Pin GPIO_PIN_0
#define Right_GPIO_Port GPIOE
#define Right_EXTI_IRQn EXTI0_IRQn
#define Enter_Pin GPIO_PIN_1
#define Enter_GPIO_Port GPIOE
#define Enter_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
