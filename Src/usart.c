/**
 ******************************************************************************
 * File Name          : USART.c
 * Description        : This file provides code for the configuration
 *                      of the USART instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "string.h"
#include <stdio.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart3;
uint8_t msgIdBuffer;
uint8_t logMsgBuffer[LOG_MSG_SIZE];
uint8_t log_available = 0;

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init (&huart3) != HAL_OK)
    {
      Error_Handler ();
    }
  // HAL_UART_Receive_IT (&huart3, &msgIdBuffer, sizeof(msgIdBuffer));
}

void HAL_UART_MspInit (UART_HandleTypeDef *uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (uartHandle->Instance == USART3)
    {
      /* USER CODE BEGIN USART3_MspInit 0 */

      /* USER CODE END USART3_MspInit 0 */
      /* USART3 clock enable */
      __HAL_RCC_USART3_CLK_ENABLE();

      __HAL_RCC_GPIOB_CLK_ENABLE();
      /**USART3 GPIO Configuration
       PB10     ------> USART3_TX
       PB11     ------> USART3_RX
       */
      GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
      HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

      /* UART3 interrupt Init */
      HAL_NVIC_SetPriority (USART3_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ (USART3_IRQn);
      // __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

      /* USER CODE BEGIN USART3_MspInit 1 */

      /* USER CODE END USART3_MspInit 1 */
    }
}

void HAL_UART_MspDeInit (UART_HandleTypeDef *uartHandle)
{

  if (uartHandle->Instance == USART3)
    {
      /* USER CODE BEGIN USART3_MspDeInit 0 */

      /* USER CODE END USART3_MspDeInit 0 */
      /* Peripheral clock disable */
      __HAL_RCC_USART3_CLK_DISABLE();

      /**USART3 GPIO Configuration
       PB10     ------> USART3_TX
       PB11     ------> USART3_RX
       */
      HAL_GPIO_DeInit (GPIOB, GPIO_PIN_10 | GPIO_PIN_11);

      /* USER CODE BEGIN USART3_MspDeInit 1 */

      /* USER CODE END USART3_MspDeInit 1 */
    }
}

void USART3_IRQHandler (void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */

  // Handle Interrupt
  HAL_UART_IRQHandler (&huart3);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
    // HAL_NVIC_DisableIRQ(EXTI2_IRQn);
    if(memcmp(logMsgBuffer, rx_log_msg, LOG_MSG_COMMON_LEN) == 0)
    {
      /* print message counter*/
      // uint8_t ctr = logMsgBuffer[3];
      // printf("%d\n", ctr);
      /* toggle LED */
      // if(ctr%2==0)
      //   HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
      // else
      //   HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);

      log_available = 1;
      /* enable reception of next log message */
      HAL_UART_Receive_IT(&huart3, logMsgBuffer, sizeof(logMsgBuffer));
    }
    else
    {
      /* enable reception of log message */
      HAL_UART_Receive_IT(&huart3, logMsgBuffer, sizeof(logMsgBuffer));
    }
    state = SEND_LOG;

    /* prepare for reception of message ID */
    // /* ranging request received */
    // if ((char)msgIdBuffer == initSequence)
    // {
    //   /* initiate ranging */
    //   state = INITIATOR;

    //   /* reset messageID buffer */
    //   msgIdBuffer = '0';

    //   /* prepare for reception of message ID */
    //   HAL_UART_Receive_IT(&huart3, &msgIdBuffer, sizeof(msgIdBuffer));

    // }
    // /* logging request received */
    // else if((char)msgIdBuffer == logSequence)
    // {
    //   /* reset messageID buffer */
    //   msgIdBuffer = '0';

    //   /* prepare for reception of log message */
    //   HAL_UART_Receive_IT(&huart3, logMsgBuffer, sizeof(logMsgBuffer));
    // }
    // /* log message received */
    // else if(memcmp(logMsgBuffer, logMsgSequence, 2) == 0)
    // {
    //   /* forward log message to uwb node */
    //   // send_log_msg(logMsgBuffer);

    //   /* print message counter */
    //   uint8_t ctr = logMsgBuffer[2];
    //   // printf("%d\n", ctr);
    //   if(ctr%2==0)
    //     HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
    //   else
    //     HAL_GPIO_WritePin (LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);


    //   /* prepare for reception of message ID */
    //   HAL_UART_Receive_IT (&huart3, &msgIdBuffer, sizeof(msgIdBuffer));
    // }
    // /* nonsense received */
    // else
    // {
    //   msgIdBuffer = '0';
    //   HAL_UART_Receive_IT(&huart3, &msgIdBuffer, sizeof(msgIdBuffer));
    // }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
    /* prepare for reception of next message */
    // READ_REG(huart3.Instance->RDR);
    HAL_UART_Receive_IT(&huart3, logMsgBuffer, sizeof(logMsgBuffer));
  }
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
