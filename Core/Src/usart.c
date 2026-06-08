/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "main.h"

/* USER CODE BEGIN 0 */
extern uint8_t rxBuffer[1];
extern uint8_t messageBuffer[MAX_MESSAGE_SIZE];
extern uint16_t messageIndex;
extern uint8_t messageComplete;

extern double roll;
extern double yaw;
extern double pitch;
extern double baseDuty;
extern int flag;
/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint8_t receivedChar = rxBuffer[0];
    
    // Check for end of message (CR, LF, or both)
    if (receivedChar == '\r' || receivedChar == '\n')
    {
      if (messageIndex > 0)  // Only process if we have data
      {
        messageBuffer[messageIndex] = '\0';  // Null terminate
        messageComplete = 1;  // Signal main loop
        
        // Echo the complete message
        char response[100];
        snprintf(response, sizeof(response), "received: %s\r\n", messageBuffer);
        HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);

      if (strcmp(messageBuffer, "reset") == 0) {
        roll = 0;
        // pitch = 0; // useless
        // yaw = 0;
        HAL_UART_Transmit(&huart2, (uint8_t*)"status reset\r\n", 12, 100);
      } else if (strcmp(messageBuffer, "0") == 0) {
        baseDuty = 0; // 'sometimes' it has problem, similarity with NULL \0
        HAL_UART_Transmit(&huart2, (uint8_t*)"thrust 0\r\n", 10, 100);
      } else if (strcmp(messageBuffer, "25") == 0) {
        baseDuty = 25;
        HAL_UART_Transmit(&huart2, (uint8_t*)"thrust 25\r\n", 11, 100);
      } else if (strcmp(messageBuffer, "50") == 0) {
        baseDuty = 50;
        HAL_UART_Transmit(&huart2, (uint8_t*)"thrust 50\r\n", 11, 100);
      } else if (strcmp(messageBuffer, "75") == 0) {
        baseDuty = 75;
        HAL_UART_Transmit(&huart2, (uint8_t*)"thrust 75\r\n", 11, 100);
      } else if (strcmp(messageBuffer, "100") == 0) {
        baseDuty = 100;
        HAL_UART_Transmit(&huart2, (uint8_t*)"thrust 100\r\n", 12, 100);
      } else if (strcmp(messageBuffer, "mode1") == 0) {
        flag = 4;
        HAL_UART_Transmit(&huart2, (uint8_t*)"set mode 1\r\n", 12, 100);
      } else if (strcmp(messageBuffer, "pid") == 0) {
        flag = 5;
        HAL_UART_Transmit(&huart2, (uint8_t*)"pid mode\r\n", 10, 100);
      } else if (strcmp(messageBuffer, "m0") == 0) {
        flag = 0;
        HAL_UART_Transmit(&huart2, (uint8_t*)"motor 0\r\n", 9, 100);
      } else if (strcmp(messageBuffer, "m1") == 0) {
        flag = 1;
        HAL_UART_Transmit(&huart2, (uint8_t*)"motor 1\r\n", 9, 100);
      } else if (strcmp(messageBuffer, "m2") == 0) {
        flag = 2;
        HAL_UART_Transmit(&huart2, (uint8_t*)"motor 2\r\n", 9, 100);
      } else if (strcmp(messageBuffer, "m3") == 0) {
        flag = 3;
        HAL_UART_Transmit(&huart2, (uint8_t*)"motor 3\r\n", 9, 100);
      } // it is too slow, because too heavy
      // Reset for next message
        messageIndex = 0;
      }
    }
    else if (messageIndex < MAX_MESSAGE_SIZE - 1)  // Prevent buffer overflow
    {
      messageBuffer[messageIndex++] = receivedChar;
    }
    
    // Restart reception for next character
    HAL_UART_Receive_IT(&huart2, rxBuffer, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    // Reset message state on error
    messageIndex = 0;
    messageComplete = 0;
    
    // Restart reception
    HAL_UART_Receive_IT(&huart2, rxBuffer, 1);
  }
}

/* USER CODE END 1 */
