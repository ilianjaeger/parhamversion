/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define LOG_MSG_SIZE 24
#define LOG_MSG_COMMON_LEN 1

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void send_log_msg(uint8_t logMsgBuffer[LOG_MSG_SIZE]);
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* Configuration enum */
typedef enum
{
    LongData_Fast,
    ShortData_Fast,
    LongData_Range
} configSel_t;

/* FSM state enum */
typedef enum
{
    IDLE,
    RECEIVE_I,
    WAIT,
    PROCESS,
    INITIATOR,
    SEND_LOG,
    PRINT_LOG
} tag_FSM_state_t;

extern tag_FSM_state_t state;
extern uint8_t rx_log_msg[];

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DW_IRQn_Pin GPIO_PIN_2
#define DW_IRQn_GPIO_Port GPIOA
#define DW_IRQn_EXTI_IRQn EXTI2_IRQn
#define DW_RESET_Pin GPIO_PIN_3
#define DW_RESET_GPIO_Port GPIOA
#define DW_NSS_Pin GPIO_PIN_2
#define DW_NSS_GPIO_Port GPIOB
#define Button_Pin GPIO_PIN_4
#define Button_GPIO_Port GPIOB
#define Button_EXTI_IRQn EXTI4_IRQn
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOB
#define DW_WU_Pin GPIO_PIN_7
#define DW_WU_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
