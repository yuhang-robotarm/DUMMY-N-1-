/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "circle_buffer.h"
#include <string.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "kinematics_task.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}
static volatile int g_tx_cplt = 0;
static volatile int g_rx_cplt = 0;

static uint8_t g_RecvBuf[24];
static circle_buf g_uart1_rx_bufs;

struct us_data gdata;  // 定义全局的结构体变量
uint8_t g_RecvTmpBuf[100];  // 临时缓冲区，用于DMA接收
size_t array_length = sizeof(g_RecvTmpBuf) / sizeof(g_RecvTmpBuf[0]);  // 计算临时缓冲区的长度

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	g_rx_cplt = 1;
	
	/* re-use dma+idle to recv */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_RecvTmpBuf, array_length);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 解析接收到的数据
    parse_received_data(g_RecvTmpBuf, Size);
    xQueueSendFromISR(xQueueUS, &gdata, NULL);
    /* re-use dma+idle to recv */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_RecvTmpBuf, array_length);
}

void parse_received_data(uint8_t *buffer, uint16_t size) {
    // 临时数组用于保存解析后的数据
    char temp_buffer[100];
    strncpy(temp_buffer, (char *)buffer, size);
    temp_buffer[size] = 'a';  // 确保字符串以'\0'结尾

    // 使用 strtok �? atof 分割字符串并转换�? float
    char *token = strtok(temp_buffer, ",");
    if (token != NULL) gdata.dec_x = atof(token);  // 解析 dec_x
    
    token = strtok(NULL, ",");
    if (token != NULL) gdata.dec_y = atof(token);  // 解析 dec_y
    
    token = strtok(NULL, ",");
    if (token != NULL) gdata.dec_z = atof(token);  // 解析 dec_z
    
    token = strtok(NULL, ",");
    if (token != NULL) gdata.euler_Z1 = atof(token);  // 解析 euler_Z1
    
    token = strtok(NULL, ",");
    if (token != NULL) gdata.euler_y = atof(token);  // 解析 euler_y
    
    token = strtok(NULL, ",");
    if (token != NULL) gdata.euler_Z2 = atof(token);  // 解析 euler_Z2

    // 打印解析后的结构体内�?

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	g_tx_cplt = 1;
}

void Wait_Tx_Complete(void)
{
	while (g_tx_cplt == 0);
	g_tx_cplt = 0;
}

void Wait_Rx_Complete(void)
{
	while (g_rx_cplt == 0);
	g_rx_cplt = 0;
}

void StartUART1Recv(void)
{
	/* init circle buffer */
	circle_buf_init(&g_uart1_rx_bufs, 100, g_RecvBuf);
	
	/* use dma+idle to recv */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_RecvTmpBuf, array_length);
}

int UART1GetChar(uint8_t *pVal)
{
	return circle_buf_read(&g_uart1_rx_bufs, pVal);
}






/* USER CODE END 1 */
