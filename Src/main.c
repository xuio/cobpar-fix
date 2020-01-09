/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

DMA_HandleTypeDef hdma_usart1_rx;

IWDG_HandleTypeDef iwdg;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

uint8_t dmx_buffer[514];
uint8_t dmx_buffer_shadow[512];
uint16_t pwm_data[3];

uint16_t fan_buf = 0;
const float alpha = 0.001;

const float max_duty[3] = {0.25, 0.5, 0.45};
const uint16_t gamma_map[] = {0,1,1,1,2,2,3,4,5,7,9,11,14,17,20,25,29,34,40,47,54,61,70,79,89,99,111,123,136,150,165,180,197,215,233,253,274,296,318,342,367,394,421,450,479,511,543,576,611,648,685,724,765,807,850,895,941,989,1038,1089,1141,1195,1251,1308,1367,1428,1490,1554,1620,1687,1757,1828,1901,1976,2052,2131,2211,2294,2378,2464,2553,2643,2735,2830,2926,3025,3125,3228,3333,3440,3549,3661,3775,3891,4009,4129,4252,4377,4505,4635,4767,4902,5039,5178,5320,5465,5612,5761,5913,6068,6225,6384,6547,6712,6879,7050,7223,7398,7577,7758,7942,8128,8318,8510,8705,8903,9104,9308,9515,9724,9937,10152,10371,10592,10817,11044,11275,11508,11745,11985,12228,12474,12723,12976,13231,13490,13752,14018,14286,14558,14833,15112,15394,15679,15968,16260,16555,16854,17156,17462,17771,18084,18400,18720,19043,19370,19701,20035,20372,20714,21059,21407,21760,22116,22476,22839,23206,23577,23952,24331,24714,25100,25490,25884,26282,26684,27090,27500,27913,28331,28753,29178,29608,30042,30480,30922,31368,31818,32272,32731,33193,33660,34131,34606,35086,35569,36057,36550,37046,37547,38052,38562,39076,39594,40117,40644,41175,41711,42252,42796,43346,43900,44458,45021,45589,46161,46738,47319,47905,48496,49091,49691,50296,50905,51519,52138,52761,53390,54023,54661,55304,55951,56604,57261,57923,58590,59262,59939,60621,61308,62000,62697,63399,64106,64818,65535};

uint16_t fan_speed(uint16_t values[3]){
  // calculate avg brightness among colors
  uint16_t avg_val = ((values[0]/max_duty[0]) + (values[1]/max_duty[1]) + (values[2]/max_duty[2])) / (3*16);
  fan_buf = (alpha * avg_val) + (1.0 - alpha) * fan_buf;
  return fan_buf;
}

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // nothing
}

uint16_t getAddress(){
  uint16_t ret = 0;
  /*uint8_t map = {
    HAL_GPIO_ReadPin(GPIOA, DIP0_Pin),
    HAL_GPIO_ReadPin(GPIOA, DIP1_Pin),
    HAL_GPIO_ReadPin(GPIOA, DIP2_Pin),
    HAL_GPIO_ReadPin(GPIOA, DIP3_Pin),
    HAL_GPIO_ReadPin(GPIOA, DIP4_Pin),
    HAL_GPIO_ReadPin(GPIOB, DIP5_Pin),
    HAL_GPIO_ReadPin(GPIOB, DIP6_Pin),
    HAL_GPIO_ReadPin(GPIOB, DIP7_Pin),
    HAL_GPIO_ReadPin(GPIOB, DIP8_Pin),
  };*/

  ret =
    !HAL_GPIO_ReadPin(GPIOA, DIP0_Pin)
  | !HAL_GPIO_ReadPin(GPIOA, DIP1_Pin) << 1
  | !HAL_GPIO_ReadPin(GPIOA, DIP2_Pin) << 2
  | !HAL_GPIO_ReadPin(GPIOA, DIP3_Pin) << 3
  | !HAL_GPIO_ReadPin(GPIOA, DIP4_Pin) << 4
  | !HAL_GPIO_ReadPin(GPIOB, DIP5_Pin) << 5
  | !HAL_GPIO_ReadPin(GPIOB, DIP6_Pin) << 6
  | !HAL_GPIO_ReadPin(GPIOB, DIP7_Pin) << 7
  | !HAL_GPIO_ReadPin(GPIOB, DIP8_Pin) << 8;

  return ret;
}

size_t old_systick = 0;

size_t systick_delta = 0;

size_t reset_counter = 0;

uint32_t speed = 0;

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // blue
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // green
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // white
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // red

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0xFFF);

  // start DMA
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, (uint8_t *)&dmx_buffer, sizeof(dmx_buffer));

  // init watchdog
  // LSI = 40kHz
  iwdg.Instance       = IWDG;
  iwdg.Init.Prescaler = IWDG_PRESCALER_32;
  iwdg.Init.Reload    = 40000/16;
  iwdg.Init.Window    = 0;


  //HAL_IWDG_Init(&iwdg);

  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  old_systick = HAL_GetTick();

  while (1)
  {
    uint32_t errorcode = huart1.ErrorCode;
    size_t curr_systick = HAL_GetTick();
    if(errorcode != 0 && huart1.RxXferSize == 514){
      old_systick = curr_systick;
      // HAL_IWDG_Refresh(&iwdg);

      HAL_UART_DMAStop(&huart1);
      uint16_t address =  getAddress();
      HAL_UART_Receive_DMA(&huart1, (uint8_t *)&dmx_buffer, sizeof(dmx_buffer));
      memcpy(dmx_buffer_shadow, dmx_buffer + 1, sizeof(dmx_buffer_shadow));
      pwm_data[0] = gamma_map[dmx_buffer_shadow[address + 0]] * max_duty[0];
      pwm_data[1] = gamma_map[dmx_buffer_shadow[address + 1]] * max_duty[1];
      pwm_data[2] = gamma_map[dmx_buffer_shadow[address + 2]] * max_duty[2];

      // set fan speed
      speed = fan_speed(pwm_data) + 2000;
      if(speed < 2500){
        speed = 0;
      } else if (speed > 4096) {
        speed = 4096;
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);

      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_data[0]);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_data[2]);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_data[1]);

      huart1.ErrorCode = 0;
      //__HAL_IWDG_RELOAD_COUNTER(&iwdg);
    }else{
      systick_delta = curr_systick - old_systick;
      if(systick_delta > 500){
        NVIC_SystemReset();
      }
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim3.Init.Prescaler = 255;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 250000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : DIP0_Pin DIP1_Pin DIP2_Pin DIP3_Pin 
                           DIP4_Pin */
  GPIO_InitStruct.Pin = DIP0_Pin|DIP1_Pin|DIP2_Pin|DIP3_Pin 
                          |DIP4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP5_Pin DIP6_Pin DIP7_Pin DIP8_Pin */
  GPIO_InitStruct.Pin = DIP5_Pin|DIP6_Pin|DIP7_Pin|DIP8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
