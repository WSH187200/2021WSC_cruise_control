/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t IC1Value, IC2Value; /* 각종 변수들 선언 */
float Cycle, Width;
int Input;
int Output;
int Error;
int Setpoint = 0;
uint8_t Spacing[] = "\t";
uint8_t NextLine[] = "\r";
float Kp = 70;
float Ki, pterm, iterm, dt;
int radix = 10;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void Func_Tim(TIM_HandleTypeDef *htim); /* PWM 주기 계산을 위한 함수 선언 */
void WSC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void WSC_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void WSC_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void WSC_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Func_Tim(TIM_HandleTypeDef *htim) /* PWM 주기 계산 함수 */
{
	if(((TIM4->DIER)&(TIM4->SR)&(0x1fff)) == 0x01)
	{
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET)
		{
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
			WSC_TIM_PeriodElapsedCallback(htim);
		}
	}
	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1) != RESET)
		{
			{
				__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
				htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;

				if ((htim->Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U)
				{
					WSC_TIM_IC_CaptureCallback(htim);
				}
				else
				{
					WSC_TIM_OC_DelayElapsedCallback(htim);
					WSC_TIM_PWM_PulseFinishedCallback(htim);
				}
				htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
			}
		}
	}

	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC2) != RESET)
		{
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
			htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;

			if ((htim->Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00U)
			{
				WSC_TIM_IC_CaptureCallback(htim);
			}
			else
			{
				WSC_TIM_OC_DelayElapsedCallback(htim);
				WSC_TIM_PWM_PulseFinishedCallback(htim);
			}
			htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
		}
	}
}

void WSC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim4.Instance)
	{

		if(((TIM4->SR)&0x06) == 0x00)
		{
			Cycle = 0;
		}
	}
}

void WSC_TIM_IC_CaptureCallback(TIM_HandleTypeDef*htim)
{
	if(htim -> Instance == TIM4)
	{
		if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			IC1Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			TIM4 -> CNT = 0;
			Cycle = (float)IC1Value;
		}
		else if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			IC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			Width = (float)IC2Value;
		}
	}
}

void WSC_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	;
}

void WSC_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { /* 스위치 입력 인터럽트 콜백 함수 */
	if(GPIO_Pin == GPIO_PIN_0) {
		for(int i=0;i<300000;i++)
		{
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==0) {
			Setpoint++;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==1) {}
	}
	if(GPIO_Pin == GPIO_PIN_1) {
		for(int j=0;j<300000;j++)
		{
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==0) {
			Setpoint--;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==1) {}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  HAL_DAC_Start(&hdac, DAC_CHANNEL_1); /* DAC 시작하는 함수 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1); /* PWM 주기를 읽는 타이머 채널1번 시작하는 함수 */
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2); /* PWM 주기를 읽는 타이머 채널2번 시작하는 함수 */
  HAL_TIM_Base_Start(&htim7);
  char CurrentSpeed[sizeof(Input)-2]; /* 현재 속도값을 나타내는 배열 선언  */
  char WantedSpeed[sizeof(Setpoint)-2]; /* 목표 속도값을 나타내는 배열 선언 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Input = 1696/Cycle; /* 구한 PWM의 주기를 속도로 환산함. 바퀴의 지름은 임의로 설정 */
	  Error = Setpoint-Input; /* 목표 속도값-현재 속도값=오차 */

	  if(Input>=30 && Input<=150) { /* 구한 PWM의 주기를 속도로 환산함. 바퀴의 지름은 임의로 설정 */
		  if(Error>=0) {
			  pterm = Kp*Error; /* P항 계산 */
			  iterm += 0.1*Error; /* I항 계산 */
			  Output = pterm+iterm; /* 제어값 도출 */
			  if (Output>4095) {
				  Output=4095;
			  }
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Output);
		  }
		  else {
			  Output = 0;
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Output);
		  }
	  }
	  else {
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1024);
	  }
	  itoa( Input, CurrentSpeed, radix ); /* 정수형인 현재 속도값을 문자형으로 변환 */
	  itoa( Setpoint, WantedSpeed, radix ); /* 정수형인 목표 속도값을 문자형으로 변환 */
	  HAL_UART_Transmit(&huart2, (uint8_t *)CurrentSpeed, sizeof(CurrentSpeed) , 1000); /* 현재 속도값 출력 */
	  HAL_UART_Transmit(&huart2, Spacing, sizeof(Spacing)-1, 1000); /* 글자 간격 띄우기 */
	  HAL_UART_Transmit(&huart2, (uint8_t *)WantedSpeed, sizeof(WantedSpeed) , 1000); /* 목표 속도값 출력 */
	  HAL_UART_Transmit(&huart2, NextLine, sizeof(NextLine)-1, 1000); /* 다음 줄로 변경 */
	  HAL_Delay(20); /* 피드백 시간 간격 0.02초 */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 42000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 21000-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 20-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 4200-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
