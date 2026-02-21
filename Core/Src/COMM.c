/*
 * COMM.c
 *
 *  Created on: Mar 1, 2025
 *      Author: 700538
 */

#include "main.h"
#include "COMM.H"
#include "GSM.H"

#include <string.h>
#include <stdlib.h>
#include <stm32g0xx.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;


//*****************************************************************************
// INTERNAL VARIABLES

TsUARTData UARTData;
extern TsGSMData GSMData;

//*****************************************************************************

void vCOMM_eUSART1Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

void vCOMM_eUSART2Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
void vCOMM_eUSART3Init(void)
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
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

// ----------------------------------------------------------------------------
// Name		: vCOMM_eTxString_Exe
// Objective	: SEND ASCII STRING
// Input  		: USART Num
//			  Pointer of String to be sent
// Output 	: none
// Return		: none
// ----------------------------------------------------------------------------
void vCOMM_eTxString_Exe(UART_HandleTypeDef *USARTx, char *pcMessage, uint16_t uiTimeOut)
{
  uint16_t uiLen;
  uiLen = strlen(pcMessage);
  if (HAL_UART_Transmit(USARTx, (uint8_t *)pcMessage, uiLen, uiTimeOut) != HAL_OK)
  //	if (HAL_UART_Transmit_IT(USARTx, (uint8_t *)pcMessage, uiLen) != HAL_OK)
  {
    Error_Handler();
  }

  UARTData.uiUARTTxCntr = uiLen;
}

// ----------------------------------------------------------------------------
// Name		: HAL_UART_RxCpltCallback
// Objective	: UART Data Receive in Interrupt Mode
//		  Interrupt callback routine
// Input  	: UART Handle Type
// Output 	: UART Data at Data Buffer
//			  UART1 : UARTData.ucUART1Buff[]
//			  UART2 : UARTData.ucUART2Buff[]
//			  UART3 : UARTData.ucUART3Buff[]
// Return	: none
// Version	: -
// ----------------------------------------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    if (GSM_RxCntr < GSM_BuffSize - 1)
    {
      GSM_DataBuffer[GSM_RxCntr++] = GSM_RxOneByte[0];
      GSM_DataBuffer[GSM_RxCntr] = 0;
    }
    else
    {
      GSM_RxCntr = GSM_BuffSize - 1;
    }

    GSM_TimeOut = 0;
    HAL_UART_Receive_IT(GSMComPortHandle, (uint8_t *)GSM_RxOneByte, 1);
  }
}

/**
 * @brief  UART error callbacks
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  if (UartHandle == &huart2)
  {
    __HAL_UART_CLEAR_PEFLAG(&huart2);
    HAL_UART_MspDeInit(&huart2);
    HAL_UART_MspInit(&huart2);
  }
}
