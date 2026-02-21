// /*
//  * ADC.c
//  *
//  *  Created on: Mar 1, 2025
//  *      Author: Brij.Z
//  */

// #include "ADC.h"
// #include "main.H"

// static void MX_ADC1_Init(void);
// static void MX_DMA_Init(void);
// void vADC_eInit(void);
// void vBATT_eReadMainBATTVolt_Exe(void);
// void vBATT_eReadIntBATTVolt_Exe(void);

// ADC_HandleTypeDef hadc1;
// DMA_HandleTypeDef hdma_adc1;

// 	TsBATTData		BATTData;

// /**
//   * @brief ADC Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_ADC1_Init(void)
// {

//   /* USER CODE BEGIN ADC1_Init 0 */

//   /* USER CODE END ADC1_Init 0 */

//   ADC_ChannelConfTypeDef sConfig = {0};

//   /* USER CODE BEGIN ADC1_Init 1 */

//   /* USER CODE END ADC1_Init 1 */

//   /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
//   */
//   hadc1.Instance = ADC1;
//   hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
//   hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//   hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//   hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
//   hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
//   hadc1.Init.LowPowerAutoWait = DISABLE;
//   hadc1.Init.LowPowerAutoPowerOff = DISABLE;
//   hadc1.Init.ContinuousConvMode = ENABLE;
//   hadc1.Init.NbrOfConversion = 1;
//   hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//   hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//   hadc1.Init.DMAContinuousRequests = ENABLE;
//   hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//   hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
//   hadc1.Init.OversamplingMode = DISABLE;
//   hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
//   if (HAL_ADC_Init(&hadc1) != HAL_OK)
//   {
// 	Error_Handler();
//   }

//   /** Configure Regular Channel
//   */
//   sConfig.Channel = ADC_CHANNEL_6;
//   sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
//   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//   {
// 	Error_Handler();
//   }

//   /** Configure Regular Channel
//   */
//   sConfig.Channel = ADC_CHANNEL_7;
//   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//   {
// 	Error_Handler();
//   }

//   /** Configure Regular Channel
//   */
//   sConfig.Channel = ADC_CHANNEL_8;
//   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//   {
// 	Error_Handler();
//   }
//   /* USER CODE BEGIN ADC1_Init 2 */

//   /* USER CODE END ADC1_Init 2 */

// }

// /**
//   * Enable DMA controller clock
//   */
// static void MX_DMA_Init(void)
// {

//   /* DMA controller clock enable */
//   __HAL_RCC_DMA1_CLK_ENABLE();

//   /* DMA interrupt init */
//   /* DMA1_Channel1_IRQn interrupt configuration */
//   HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
//   HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
// }

// void vADC_eInit(void)
// {
// 	GPIO_InitTypeDef GPIO_InitStruct = {0};

// 	// GPIO Ports Clock Enable
// 	  __HAL_RCC_GPIOB_CLK_ENABLE();
// 	  // __HAL_RCC_GPIOC_CLK_ENABLE();
// 	  // __HAL_RCC_GPIOA_CLK_ENABLE();

// 	MX_DMA_Init();
// 	MX_ADC1_Init();

// //	if(APPData.ucHWVer == 'D')
// //	{
// 		GPIO_InitStruct.Pin 	= INTBATT_CHG_DRV_Pin;
// 		GPIO_InitStruct.Mode	= GPIO_MODE_OUTPUT_PP;
// 		GPIO_InitStruct.Pull	= GPIO_NOPULL;
// 		GPIO_InitStruct.Speed	= GPIO_SPEED_FREQ_LOW;
// 		HAL_GPIO_Init(INTBATT_CHG_DRV_GPIO_Port, &GPIO_InitStruct);
// //	}
// //	else
// //	{
// //	}

// //	BATTStatus.bIntBattMOSFETChgStatus 	= OFF;
// //	BATTStatus.bBATTStatus				= BATTERY_PRESENT;
// //	BATTStatus.bBATTStatus				= BATTERY_PRESENT;
// }

// void vBATT_eReadMainBATTVolt_Exe(void)
// {
// 	HAL_ADC_Start_DMA(&hadc1, BATTData.ulRawADCVal, TOTAL_ADC_CHANNEL);
// 	BATTData.ucBATTVolt = (uint8_t) (BATTData.ulRawADCVal[RAW_MAIN_BATT_ADC_VAL] * ADC_CFAC_MAINBATTVOL);

// //	if (BATTData.ucBATTVolt < 10)
// //	{
// //		BATTStatus.bBATTVoltStatus = NO_BATTERY_VOLTAGE;
// //	}
// //	else
// //	{
// //		BATTStatus.bBATTVoltStatus = BATTERY_VOLTAGE_OK;
// //	}
// }

// void vBATT_eReadIntBATTVolt_Exe(void)
// {
// 	HAL_ADC_Start_DMA(&hadc1, BATTData.ulRawADCVal, TOTAL_ADC_CHANNEL);
// 	BATTData.uiIntBATTVolt = (uint16_t)(BATTData.ulRawADCVal[RAW_INT_BATT_ADC_VAL] * ADC_CFAC_INTBATTVOL);
// }
