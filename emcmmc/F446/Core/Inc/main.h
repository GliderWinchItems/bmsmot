/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_WAKE_FETGATE_Pin GPIO_PIN_13
#define RTC_WAKE_FETGATE_GPIO_Port GPIOC
#define JP9_THERM_Pin GPIO_PIN_0
#define JP9_THERM_GPIO_Port GPIOC
#define JP8_THERM_Pin GPIO_PIN_1
#define JP8_THERM_GPIO_Port GPIOC
#define JP10_THERM_Pin GPIO_PIN_2
#define JP10_THERM_GPIO_Port GPIOC
#define JP11_THERM_Pin GPIO_PIN_3
#define JP11_THERM_GPIO_Port GPIOC
#define M_RESET_Pin GPIO_PIN_5
#define M_RESET_GPIO_Port GPIOA
#define JP34_ACtransfrmr_Pin GPIO_PIN_6
#define JP34_ACtransfrmr_GPIO_Port GPIOA
#define PWR12V__Pin GPIO_PIN_7
#define PWR12V__GPIO_Port GPIOA
#define JP17_SPARE_Pin GPIO_PIN_4
#define JP17_SPARE_GPIO_Port GPIOC
#define JP24_Pressure_Pin GPIO_PIN_5
#define JP24_Pressure_GPIO_Port GPIOC
#define STRING_PLUS_Pin GPIO_PIN_0
#define STRING_PLUS_GPIO_Port GPIOB
#define STRING_MINUS_Pin GPIO_PIN_1
#define STRING_MINUS_GPIO_Port GPIOB
#define HDR_OA3_Pin GPIO_PIN_2
#define HDR_OA3_GPIO_Port GPIOB
#define LED5_GRN_Pin GPIO_PIN_12
#define LED5_GRN_GPIO_Port GPIOB
#define LED6_RED_Pin GPIO_PIN_13
#define LED6_RED_GPIO_Port GPIOB
#define OA4_Pin GPIO_PIN_14
#define OA4_GPIO_Port GPIOB
#define OA2_Pin GPIO_PIN_15
#define OA2_GPIO_Port GPIOB
#define OB3_Pin GPIO_PIN_6
#define OB3_GPIO_Port GPIOC
#define OB1_Pin GPIO_PIN_7
#define OB1_GPIO_Port GPIOC
#define OB2_Pin GPIO_PIN_8
#define OB2_GPIO_Port GPIOC
#define OB4_Pin GPIO_PIN_9
#define OB4_GPIO_Port GPIOC
#define OC3_Pin GPIO_PIN_8
#define OC3_GPIO_Port GPIOA
#define OC1_Pin GPIO_PIN_9
#define OC1_GPIO_Port GPIOA
#define OC4_Pin GPIO_PIN_10
#define OC4_GPIO_Port GPIOA
#define OC2_Pin GPIO_PIN_11
#define OC2_GPIO_Port GPIOA
#define Keyswsense_Pin GPIO_PIN_12
#define Keyswsense_GPIO_Port GPIOA
#define ACzeroxing_Pin GPIO_PIN_15
#define ACzeroxing_GPIO_Port GPIOA
#define ACzeroxing_EXTI_IRQn EXTI15_10_IRQn
#define JP21_FETSUB104_Pin GPIO_PIN_3
#define JP21_FETSUB104_GPIO_Port GPIOB
#define JP14_IN_Pin GPIO_PIN_4
#define JP14_IN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
// USART/UART assignments
#define HUARTMON  huart3 // uart  for PC monitoring

#define DEFAULTTASKBIT00 (1 << 0)  // Task notification bit (from ADCtask.c)
#define DEFAULTTASKBIT01 (1 << 1)  // Task notification bit (from BQtask.c)
#define DEFAULTTASKBIT02 (1 << 2)  // Task notification bit (from Mailbox)

#define MAINFORLOOPDELAY 50 // main.c Delay of 'for' loop (ms)

#define CONFIGCAN2  // Configure for CAN2

// Gateway task (for Mailbox use)
#define GATEWAYTASKINCLUDED // Include gateway

extern const uint32_t i_am_canid;

/* Enable discharge FET bits testing. */
// Uncomment the following to enable test
//#define TEST_WALK_DISCHARGE_FET_BITS

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
