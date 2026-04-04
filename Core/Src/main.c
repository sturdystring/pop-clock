/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7789.c"
#include "fonts.c"
#include <stdio.h>
#include <string.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_MAIN_SCREEN,
    STATE_MENU,
    STATE_MENU_ITEM_Time,
	STATE_MENU_ITEM_Date,
	STATE_MENU_ITEM_Alarm,
	STATE_MENU_ITEM_Color_Back,
	STATE_MENU_ITEM_Bright,
	STATE_MENU_ITEM_auto_off,
	STATE_MENU_ITEM_fraze_select,
	STATE_MENU_ITEM_service
} AppState;
AppState state = STATE_MAIN_SCREEN;

/*typedef enum {
    MENU_COLOR,
    MENU_MAX
} MenuItem;
MenuItem menuIndex = 0;
typedef enum {
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_DONE
} ColorEditState;*/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HDC2080_ADDR      (0x40 << 1)   // 7-bit -> 8-bit ������������ HAL
#define HDC2080_TEMP_LOW  0x00
#define HDC2080_TEMP_HIGH 0x01
#define HDC2080_HUM_LOW   0x02
#define HDC2080_HUM_HIGH  0x03
#define HDC2080_CONFIG    0x0E
#define HDC2080_MEAS_CONF 0x0F
#define LONG_PRESS_TIME 800   // мс
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint16_t adc_data[3];
float Temp, Hum;
uint8_t need_sleep;
int16_t lastEnc = 0;
float v_bat_filt = 3;


uint32_t buttonDownTime = 0;
uint8_t buttonPrev = 1;
uint8_t buttonPressed = 0;



uint8_t Hou, Min, Sec, Dat, Week, Mou, Year;

uint8_t menu_index;




uint8_t hou_set,min_set,sec_set;


uint8_t dat_set=1,weekday_set,mou_set=1,year_set=26;


uint8_t hou_set_alarm,min_set_alarm;


uint8_t dat_meet, hou_meet,min_meet,sec_meet;


uint8_t bat_state;


uint8_t alarm_active, alarm_set;

uint32_t now_time;
uint32_t time_was;

uint8_t color_back_red = 0x1F,color_back_green = 0x3F,color_back_blue = 0x1F;
uint16_t color_back=WHITE, color_back_now = WHITE, color_text = BLACK;


uint8_t auto_switch_off=1;
uint8_t time_to_switch_off=30;
uint32_t time_set_to_switch_off=30*1000;



uint16_t Bright_set=10;
uint8_t auto_bright=1;



uint8_t need_reset;

uint8_t number_fraze;



uint8_t auto_change=1;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void HDC2080_Init(void);
void HDC2080_Reset(void);
void HDC2080_Read(float *temperature, float *humidity);
void WakeUp(void);
void GetTimeDate(uint8_t *hour,uint8_t *minutes,uint8_t *seconds,uint8_t *day,uint8_t *weekday, uint8_t *mounth,uint8_t *year);
void GoSleep(void);
void WriteToDisplay(uint8_t hour,uint8_t minutes,uint8_t seconds,uint8_t day,uint8_t weekday, uint8_t mounth,uint8_t year);
uint8_t ReadButton(void);
void HandleButton(void);
int16_t GetEncoder(void);
void HandleEncoder(void);
void ButtonShortPressHandler(void);
void ButtonLongPressHandler(void);
void EncoderRight(void);
void EncoderLeft(void);
uint32_t rtc_to_seconds(RTC_DateTypeDef *d, RTC_TimeTypeDef *t);
void Now_Time(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef DateToUpdate = {0};
RTC_AlarmTypeDef sAlarm = {0};

RTC_DateTypeDef targerDate = {.Date = 8, .Month = 2, .Year = 26};
RTC_TimeTypeDef targetTime = {.Hours = 18, .Minutes = 0, .Seconds = 0};
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  for(uint8_t i=0;i<=1;i++)ST7789_Init();
  ST7789_Fill_Color(color_back_now);
  ST7789_DrawRectangle(290, 10, 310, 50, color_text);
  ST7789_DrawFilledRectangle(295, 5, 11, 6, color_text);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  for(uint8_t i=0;i<=1;i++)HDC2080_Init();
  GetTimeDate(&Hou, &Min, &Sec, &Dat, &Week, &Mou, &Year);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_data, 3);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  //HAL_TIM_Base_Start_IT(&htim4);
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  //EnterInMenu();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HandleButton();
	  HandleEncoder();


	  if (state==STATE_MAIN_SCREEN && alarm_active==0 && auto_switch_off==1)	  Now_Time();
	  else
		  {
		  now_time = HAL_GetTick();
		  time_was = now_time;
		  }
	  if(need_sleep==1)
	  {
		  GoSleep();
	  }
	  GetTimeDate(&Hou, &Min, &Sec, &Dat, &Week, &Mou, &Year);
		HDC2080_Read(&Temp, &Hum);
		WriteToDisplay(Hou, Min, Sec, Dat, Week, Mou, Year);
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = 32767;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
	/*HAL_PWR_EnableBkUpAccess();
	if(HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1)==0x32F2)
	{
		return;
	}*/
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 14;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_TUESDAY;
  DateToUpdate.Month = RTC_MONTH_FEBRUARY;
  DateToUpdate.Date = 3;
  DateToUpdate.Year = 26;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  //HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 36000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 12000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 36000/2;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 287;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_Pin|RST_Pin|DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin RST_Pin DC_Pin */
  GPIO_InitStruct.Pin = CS_Pin|RST_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc)
{
	if(adc_data[2]>3800 && auto_bright==1)
	{
		TIM2->CCR3 = TIM2->ARR*0.1f;
	}
	else if(adc_data[2]<800 && auto_bright==1)
	{
		TIM2->CCR3 = TIM2->ARR;
	}
	else if(adc_data[2]>1200 && adc_data[2]<2000 && auto_bright==1)
	{
		TIM2->CCR3 = TIM2->ARR*0.7f;
	}
	else if(adc_data[2]>2300 && adc_data[2]<3500 && auto_bright==1)
	{
		TIM2->CCR3 = TIM2->ARR*0.3f;
	}
	else if (auto_bright==0) TIM2->CCR3 = TIM2->ARR/100*Bright_set;



	float v_bat = adc_data[1]*3.3f/4095.0f/0.666f;
	v_bat_filt = v_bat*0.2f+v_bat_filt*(1.0f-0.2f);



	if (v_bat_filt >= 3.8) {
		bat_state = 3;
	} else if (v_bat_filt >= 3.4f && v_bat_filt < 3.7f) {
		bat_state = 2;
	} else if (v_bat_filt >= 3.0f && v_bat_filt < 3.3f) {
		bat_state = 1;
	} else if (v_bat_filt >= 2.7f && v_bat_filt < 2.9f) {
		bat_state = 0;
	}






	if(v_bat_filt<2.7 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)==1)
	{
		need_sleep=1;
	}
}
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{
	/*if(state==STATE_MAIN_SCREEN)
	{
		GoSleep();
	}*/
	static uint8_t poloj;
	if (poloj==0)
	{
		poloj = 1;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	}
	else
	{
		poloj = 0;
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
	}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*if(v_bat_filt<2.7)
	{
		GoSleep();
	}*/
	if(need_sleep==1)
	{
		WakeUp();
	}
}
void HDC2080_Init(void)
{
	HDC2080_Reset();
	HAL_Delay(250);
    uint8_t config = 0x00;

    /* ������������������������:
       BIT7 = 0 ��� �������������� ����������
       BIT6 = 0 ��� �������� ������������ ������ ��������������������
       BIT5..4 = 10 ��� 14-bit Temp
       BIT3..2 = 10 ��� 14-bit Humidity
       BIT1..0 = 00 ��� �������������������� �������������� ��������������
    */
    config = 0x50;
    HAL_I2C_Mem_Write(&hi2c1, HDC2080_ADDR, HDC2080_CONFIG, 1, &config, 1, HAL_MAX_DELAY);

    /* ������������ ������������������ (Temp + Humidity) */
    uint8_t meas = 0x01; // Start Measurement
    HAL_I2C_Mem_Write(&hi2c1, HDC2080_ADDR, HDC2080_MEAS_CONF, 1, &meas, 1, HAL_MAX_DELAY);
}
void HDC2080_Reset(void)
{
	uint8_t HDC2080_Reset_Bit[1]={0x80};
	HAL_I2C_Mem_Write(&hi2c1,HDC2080_ADDR, 0x0E, 1, HDC2080_Reset_Bit, 1, HAL_MAX_DELAY);
}
void HDC2080_Read(float *temperature, float *humidity)
{
    uint8_t temp_low, temp_high;
    uint8_t hum_low, hum_high;

    HAL_I2C_Mem_Read(&hi2c1, HDC2080_ADDR, HDC2080_TEMP_LOW, 1, &temp_low, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, HDC2080_ADDR, HDC2080_TEMP_HIGH, 1, &temp_high, 1, HAL_MAX_DELAY);

    HAL_I2C_Mem_Read(&hi2c1, HDC2080_ADDR, HDC2080_HUM_LOW, 1, &hum_low, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, HDC2080_ADDR, HDC2080_HUM_HIGH, 1, &hum_high, 1, HAL_MAX_DELAY);

    uint16_t rawTemp = (temp_high << 8) | temp_low;
    uint16_t rawHum  = (hum_high << 8) | hum_low;

    /* �������������� ���� ���������������� */
    *temperature = ((float)rawTemp / 65536.0f) * 165.0f - 41.0f;
    *humidity    = ((float)rawHum / 65536.0f) * 100.0f;
}
void WakeUp(void)
{
	need_sleep=0;
	SystemClock_Config();
	ST7789_SleepModeExit();
	//HAL_TIM_Base_Start_IT(&htim4);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_data, 3);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}
void GetTimeDate(uint8_t *hour,uint8_t *minutes,uint8_t *seconds,uint8_t *day,uint8_t *weekday, uint8_t *mounth,uint8_t *year)
{
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
	*hour = sTime.Hours;
	*minutes = sTime.Minutes;
	*seconds = sTime.Seconds;
	*day = DateToUpdate.Date;
	*weekday = DateToUpdate.WeekDay;
	*mounth = DateToUpdate.Month;
	*year = DateToUpdate.Year;
}
void WriteToDisplay(uint8_t hour, uint8_t minutes, uint8_t seconds, uint8_t day,
		uint8_t weekday, uint8_t mounth, uint8_t year) {
	char buf[64] = { };
	switch (state) {
	case STATE_MAIN_SCREEN:

		/*uint32_t now = rtc_to_seconds(&DateToUpdate, &sTime);
		uint32_t targer = rtc_to_seconds(&targerDate, &targetTime);
		int32_t diff = targer - now;
		if(diff<0) diff=0;
		uint32_t days_to_meet = diff/86400;
		uint32_t hour_to_meet = (diff%86400)/3600;
		uint32_t minutes_to_meet = (diff%3600)/60;
		uint32_t seconds_to_meet = diff%60;


		sprintf(buf,"%02li дн",days_to_meet);
		ST7789_WriteString(5, 5, buf, Font_16x26, color_text, color_back_now);


		sprintf(buf,"%02li ч",hour_to_meet);
		ST7789_WriteString(5, 31, buf, Font_16x26, color_text, color_back_now);


		sprintf(buf,"%02li мин",minutes_to_meet);
		ST7789_WriteString(5, 57, buf, Font_16x26, color_text, color_back_now);


		sprintf(buf,"%02li сек до встречи",seconds_to_meet);
		ST7789_WriteString(5, 83, buf, Font_16x26, color_text, color_back_now);*/
		static uint8_t hou_past;
		if(hou_past!=hour && auto_change==1)
		{
		ST7789_DrawFilledRectangle(5, 5, 285, 95, color_back_now);
			number_fraze++;
			if(number_fraze>16)number_fraze=0;
			hou_past=hour;
		}
		if ((Dat==1 && Mou==1) || (Dat == 31 && Mou ==12))
		{
			ST7789_WriteString_ramk(5, 5, 290, 240, "С новым годом! С новым счастьем!",
								Font_16x26, color_text, color_back_now);
		}
		else if (Dat==13 && Mou == 7)
		{
			ST7789_WriteString_ramk(5, 5, 290, 240, "С днем рождения  моя красотка!    Счастья, здоровьяи любви от меня!",
											Font_16x26, color_text, color_back_now);
		}
		else if (Dat==16 && Mou==2)
		{
			ST7789_WriteString_ramk(5, 5, 290, 240, "С нашим праздни- ком, очень тебя  люблю и целую!",
														Font_16x26, color_text, color_back_now);
		}
		else
		{
		switch (number_fraze) {
		case 0:
			ST7789_WriteString_ramk(5, 5, 290, 240, "Ты моя любимая   девочка",
					Font_16x26, color_text, color_back_now);
			break;
		case 1:
			ST7789_WriteString_ramk(5, 5, 290, 240,
					"Ты самая лучшая вэтой вселенной", Font_16x26, color_text,
					color_back_now);
			break;
		case 2:
			ST7789_WriteString_ramk(5, 5, 290, 240,
					"Ты богиня, спус- тившаяся с небес", Font_16x26, color_text,
					color_back_now);
			break;
		case 3:
			ST7789_WriteString_ramk(5, 5, 290, 240,
					"Ты самая красиваядевочка в этом   мире", Font_16x26, color_text,
					color_back_now);
			break;
		case 4:
			ST7789_WriteString_ramk(5, 5, 290, 240,
					"Ты лучшее, что сомной случалось", Font_16x26, color_text,
					color_back_now);
			break;
		case 5:
			ST7789_WriteString_ramk(5, 5, 290, 240,
					"Твоя красота     покоряет мое     сердце", Font_16x26, color_text,
					color_back_now);
			break;
		case 6:
			ST7789_WriteString_ramk(5, 5, 290, 240,
					"Моя любовь к тебене знает границ", Font_16x26, color_text,
					color_back_now);
			break;
		case 7:
			ST7789_WriteString_ramk(5, 5, 290, 240, "Not fake, true   love",
					Font_16x26, color_text, color_back_now);
			break;
		case 8:
			ST7789_WriteString_ramk(5, 5, 290, 240, "Горжусь тобой", Font_16x26,
					color_text, color_back_now);
			break;
		case 9:
			ST7789_WriteString_ramk(5, 5, 290, 240, "Ти самая умничка",
					Font_16x26, color_text, color_back_now);
			break;
		case 10:
			ST7789_WriteString_ramk(5, 5, 290, 240,
					"Очень тебя люблю и очень скучаю", Font_16x26, color_text,
					color_back_now);
			break;
		case 11:
			ST7789_WriteString_ramk(5, 5, 290, 240, "Приезжай скорее",
					Font_16x26, color_text, color_back_now);
			break;
		case 12:
			ST7789_WriteString_ramk(5, 5, 290, 240, "Ты лучше всех,   никого не слушай",
					Font_16x26, color_text, color_back_now);
			break;
		case 13:
			ST7789_WriteString_ramk(5, 5, 290, 240, "Красивая как     ангел", Font_16x26,
					color_text, color_back_now);
			break;
		case 14:
			ST7789_WriteString_ramk(5, 5, 290, 240, "Очень скучаю, мойкотик",
					Font_16x26, color_text, color_back_now);
			break;
		}
		}


		uint8_t charge_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
		static uint8_t count=3;
		switch (charge_state) {
		case 1:
			if(bat_state ==3)
			{
				ST7789_DrawFilledRectangle(292, 12, 17, 11, color_text);
				ST7789_DrawFilledRectangle(292, 25, 17, 11, color_text);
				ST7789_DrawFilledRectangle(292, 38, 17, 11, color_text);
			}
			if (bat_state == 2) {
				ST7789_DrawFilledRectangle(292, 12, 17, 11, color_back_now);
				ST7789_DrawFilledRectangle(292, 25, 17, 11, color_text);
				ST7789_DrawFilledRectangle(292, 38, 17, 11, color_text);
			} else if (bat_state == 1) {
				ST7789_DrawFilledRectangle(292, 12, 17, 11, color_back_now);
				ST7789_DrawFilledRectangle(292, 25, 17, 11, color_back_now);
				ST7789_DrawFilledRectangle(292, 38, 17, 11, color_text);
			} else if (bat_state == 0) {
				ST7789_DrawFilledRectangle(292, 12, 17, 11, color_back_now);
				ST7789_DrawFilledRectangle(292, 25, 17, 11, color_back_now);
				ST7789_DrawFilledRectangle(292, 38, 17, 11, RED);
			}
			break;
		case 0:
			static uint8_t times_blint;
			if (bat_state == 3 && count==5) {
				if (times_blint == 0)
					ST7789_DrawFilledRectangle(292, 12, 17, 11, color_text);
				else
					ST7789_DrawFilledRectangle(292, 12, 17, 11, color_back_now);
				times_blint++;
				if (times_blint > 2)
					times_blint = 0;
				ST7789_DrawFilledRectangle(292, 25, 17, 11, color_text);
				ST7789_DrawFilledRectangle(292, 38, 17, 11, color_text);
			} else if (bat_state == 2 && count==5) {
				if (times_blint == 0) {
					ST7789_DrawFilledRectangle(292, 12, 17, 11, color_back_now);
					ST7789_DrawFilledRectangle(292, 25, 17, 11, color_back_now);
				} else if (times_blint == 1) {
					ST7789_DrawFilledRectangle(292, 12, 17, 11, color_back_now);
					ST7789_DrawFilledRectangle(292, 25, 17, 11, color_text);
				} else if (times_blint == 2) {
					ST7789_DrawFilledRectangle(292, 12, 17, 11, color_text);
					ST7789_DrawFilledRectangle(292, 25, 17, 11, color_text);
				}
				times_blint++;
				if (times_blint > 3)
					times_blint = 0;
				ST7789_DrawFilledRectangle(292, 38, 17, 11, color_text);
			} else if ((bat_state == 0 || bat_state == 1) && count==5) {
				if (times_blint == 0) {
					ST7789_DrawFilledRectangle(292, 12, 17, 11, color_back_now);
					ST7789_DrawFilledRectangle(292, 25, 17, 11, color_back_now);
					ST7789_DrawFilledRectangle(292, 38, 17, 11, color_back_now);
				} else if (times_blint == 1) {
					ST7789_DrawFilledRectangle(292, 12, 17, 11, color_back_now);
					ST7789_DrawFilledRectangle(292, 25, 17, 11, color_back_now);
					ST7789_DrawFilledRectangle(292, 38, 17, 11, color_text);
				} else if (times_blint == 2) {
					ST7789_DrawFilledRectangle(292, 12, 17, 11, color_back_now);
					ST7789_DrawFilledRectangle(292, 25, 17, 11, color_text);
					ST7789_DrawFilledRectangle(292, 38, 17, 11, color_text);
				} else if (times_blint == 3) {
					ST7789_DrawFilledRectangle(292, 12, 17, 11, color_text);
					ST7789_DrawFilledRectangle(292, 25, 17, 11, color_text);
					ST7789_DrawFilledRectangle(292, 38, 17, 11, color_text);
				}
				times_blint++;
				if (times_blint > 3)
					times_blint = 0;
			}
		}
		count++;
		if(count>5) count=0;
		switch (weekday) {
		case 0:
			ST7789_WriteString(250, 110, "Вс", Font_16x26, color_text, color_back_now);
			break;
		case 1:
			ST7789_WriteString(250, 110, "Пн", Font_16x26, color_text, color_back_now);
			break;
		case 2:
			ST7789_WriteString(250, 110, "Вт", Font_16x26, color_text, color_back_now);
			break;
		case 3:
			ST7789_WriteString(250, 110, "Ср", Font_16x26, color_text, color_back_now);
			break;
		case 4:
			ST7789_WriteString(250, 110, "Чт", Font_16x26, color_text, color_back_now);
			break;
		case 5:
			ST7789_WriteString(250, 110, "Пт", Font_16x26, color_text, color_back_now);
			break;
		case 6:
			ST7789_WriteString(250, 110, "Сб", Font_16x26, color_text, color_back_now);
			break;
		}
		sprintf(buf, "%02i:%02i:%02i", hour, minutes, seconds);
		ST7789_WriteString(80, 110, buf, Font_16x26, color_text, color_back_now);
		sprintf(buf, "%02i.%02i.20%02i", day, mounth, year);
		ST7789_WriteString(80, 140, buf, Font_16x26, color_text, color_back_now);
		sprintf(buf, "%.1fC %.0f%%", Temp, Hum);
		static uint8_t buf_size_past = 0xFF;
		uint8_t buf_size_now = strlen(buf);
		if (buf_size_now < buf_size_past) {
			ST7789_DrawFilledRectangle(150, 200, 320 - 151, 29, color_back_now);
			buf_size_past = buf_size_now;
		}
		else buf_size_past = buf_size_now;
		ST7789_WriteString(150, 200, buf, Font_16x26, color_text, color_back_now);
		break;
	case STATE_MENU:
		ST7789_WriteString(5, 5, "Время", Font_16x26, color_text, color_back_now);
		ST7789_WriteString(5, 35, "Дата", Font_16x26, color_text, color_back_now);
		ST7789_WriteString(5, 65, "Будильник", Font_16x26, color_text, color_back_now);
		ST7789_WriteString(5, 95, "Цвет фона", Font_16x26, color_text, color_back_now);
		ST7789_WriteString(5, 125, "Яркость", Font_16x26, color_text, color_back_now);
		ST7789_WriteString(5, 155, "Авт. выкл", Font_16x26, color_text, color_back_now);
		ST7789_WriteString(5, 185, "Выбор фразы", Font_16x26, color_text, color_back_now);
		ST7789_WriteString(5, 215, "Сервис", Font_16x26, color_text, color_back_now);
		static uint8_t menu_index_past;
		if(menu_index!=menu_index_past)
		{
			ST7789_DrawFilledRectangle(250, 5, 16*2, 240-6, color_back_now);
			menu_index_past=menu_index;
		}
		ST7789_WriteString(250, 5+(30*menu_index), "<-", Font_16x26, color_text, color_back_now);
		break;
	case STATE_MENU_ITEM_Time:
		ST7789_WriteString(5, 5, "Время", Font_16x26, color_text, color_back_now);
		sprintf(buf,"Часы: %02i",hou_set);
		ST7789_WriteString(5, 50, buf, Font_16x26, color_text, color_back_now);
		sprintf(buf,"Минуты: %02i",min_set);
		ST7789_WriteString(5, 100, buf, Font_16x26, color_text, color_back_now);
		sprintf(buf,"Секунды: %02i",sec_set);
		ST7789_WriteString(5, 150, buf, Font_16x26, color_text, color_back_now);
		if(menu_index!=menu_index_past)
				{
					ST7789_DrawFilledRectangle(250, 5, 16*2, 240-11, color_back_now);
					menu_index_past=menu_index;
				}
				ST7789_WriteString(250, 50+(50*menu_index), "<-", Font_16x26, color_text, color_back_now);
		break;
	case STATE_MENU_ITEM_Date:
		ST7789_WriteString(5, 5, "Дата", Font_16x26, color_text, color_back_now);
		sprintf(buf, "Число: %02i", dat_set);
		ST7789_WriteString(5, 50, buf, Font_16x26, color_text, color_back_now);
		//sprintf(buf, "День недели: %01i", weekday_set);
		ST7789_WriteString(5, 100, "День недели:", Font_16x26, color_text, color_back_now);
		switch (weekday_set) {
				case 0:
					ST7789_WriteString(213, 100, "Вс", Font_16x26, color_text, color_back_now);
					break;
				case 1:
					ST7789_WriteString(213, 100, "Пн", Font_16x26, color_text, color_back_now);
					break;
				case 2:
					ST7789_WriteString(213, 100, "Вт", Font_16x26, color_text, color_back_now);
					break;
				case 3:
					ST7789_WriteString(213, 100, "Ср", Font_16x26, color_text, color_back_now);
					break;
				case 4:
					ST7789_WriteString(213, 100, "Чт", Font_16x26, color_text, color_back_now);
					break;
				case 5:
					ST7789_WriteString(213, 100, "Пт", Font_16x26, color_text, color_back_now);
					break;
				case 6:
					ST7789_WriteString(213, 100, "Сб", Font_16x26, color_text, color_back_now);
					break;
				}
		sprintf(buf, "Месяц: %02i", mou_set);
		ST7789_WriteString(5, 150, buf, Font_16x26, color_text, color_back_now);
		sprintf(buf, "Год: %02i", year_set);
		ST7789_WriteString(5, 2000, buf, Font_16x26, color_text, color_back_now);
		if (menu_index != menu_index_past) {
			ST7789_DrawFilledRectangle(250, 5, 16 * 2, 240 - 11, color_back_now);
			menu_index_past = menu_index;
		}
		ST7789_WriteString(250, 50 + (50 * menu_index), "<-", Font_16x26, color_text,
				color_back_now);
		break;
	case STATE_MENU_ITEM_Alarm:
		ST7789_WriteString(5, 5, "Будильник", Font_16x26, color_text,
				color_back_now);
		if(alarm_set==1) ST7789_WriteString(5, 50, "Будильник вкл ", Font_16x26, color_text,
				color_back_now);
		else ST7789_WriteString(5, 50, "Будильник выкл", Font_16x26, color_text,
				color_back_now);
		sprintf(buf, "Часы: %02i", hou_set_alarm);
		ST7789_WriteString(5, 100, buf, Font_16x26, color_text, color_back_now);
		sprintf(buf, "Минуты: %02i", min_set_alarm);
		ST7789_WriteString(5, 150, buf, Font_16x26, color_text, color_back_now);
		if (menu_index != menu_index_past) {
			ST7789_DrawFilledRectangle(250, 5, 16 * 2, 240 - 11,
					color_back_now);
			menu_index_past = menu_index;
		}
		ST7789_WriteString(250, 50 + (50 * menu_index), "<-", Font_16x26, color_text,
				color_back_now);
		break;
	case STATE_MENU_ITEM_Color_Back:
		ST7789_WriteString(5, 5, "Цвет фона", Font_16x26, color_text, color_back_now);
		sprintf(buf, "Красный: %02i", color_back_red);
		ST7789_WriteString(5, 50, buf, Font_16x26, color_text, color_back_now);
		sprintf(buf, "Зеленый: %02i", color_back_green);
		ST7789_WriteString(5, 100, buf, Font_16x26, color_text, color_back_now);
		sprintf(buf, "Синий: %02i", color_back_blue);
		ST7789_WriteString(5, 150, buf, Font_16x26, color_text, color_back_now);
		color_back = (color_back_red<<11) | (color_back_green<<5) | color_back_blue;
		ST7789_DrawFilledRectangle(5, 180, 100, 50, color_back);
		if (menu_index != menu_index_past) {
			ST7789_DrawFilledRectangle(250, 5, 16 * 2, 240 - 11,
					color_back_now);
			menu_index_past = menu_index;
		}
		ST7789_WriteString(250, 50 + (50 * menu_index), "<-", Font_16x26, color_text,
				color_back_now);
		break;
	case STATE_MENU_ITEM_Bright:
		ST7789_WriteString(5, 5, "Яркость", Font_16x26, color_text, color_back_now);
		if(auto_bright==0)	ST7789_WriteString(5, 50, "Авт ярк выкл", Font_16x26, color_text, color_back_now);
		else ST7789_WriteString(5, 50, "Авт ярк вкл ", Font_16x26, color_text, color_back_now);
		sprintf(buf,"Яркость: %i%%",Bright_set);
		static uint8_t buf_size_past_bright = 0xFF;
		uint8_t buf_size_now_bright = strlen(buf);
		if (buf_size_now_bright < buf_size_past_bright) {
			ST7789_DrawFilledRectangle(128, 100, 90, 26, color_back_now);
			buf_size_past_bright = buf_size_now_bright;
		}
		else buf_size_past_bright = buf_size_now_bright;
		ST7789_WriteString(5, 100, buf, Font_16x26, color_text, color_back_now);




		if (menu_index != menu_index_past) {
			ST7789_DrawFilledRectangle(250, 5, 16 * 2, 240 - 11,
					color_back_now);
			menu_index_past = menu_index;
		}
		ST7789_WriteString(250, 50 + (50 * menu_index), "<-", Font_16x26, color_text,
				color_back_now);
		break;
	case STATE_MENU_ITEM_auto_off:
		ST7789_WriteString(5, 5, "Авт. выкл", Font_16x26, color_text, color_back_now);
		if(auto_switch_off == 0)
		{
			ST7789_WriteString(5, 50, "Авт.выкл-Выкл", Font_16x26, color_text, color_back_now);
		}
		else
		{
			ST7789_WriteString(5, 50, "Авт.выкл-Вкл ", Font_16x26, color_text, color_back_now);
		}
		sprintf(buf, "Время выкл: %02i", time_to_switch_off);
		ST7789_WriteString(5, 100, buf, Font_16x26, color_text, color_back_now);
		if (menu_index != menu_index_past) {
					ST7789_DrawFilledRectangle(250, 5, 16 * 2, 240 - 11,
							color_back_now);
					menu_index_past = menu_index;
				}
				ST7789_WriteString(250, 50 + (50 * menu_index), "<-", Font_16x26, color_text,
						color_back_now);
		break;
	case STATE_MENU_ITEM_fraze_select:
		ST7789_WriteString(5, 5, "Выбор фразы", Font_16x26, color_text,
				color_back_now);
		sprintf(buf, "Фраза: %02i", number_fraze);
		ST7789_WriteString(5, 50, buf, Font_16x26, color_text, color_back_now);
		switch (number_fraze) {
		case 0:
			ST7789_WriteString(5, 50 + 26, "Ты моя любимая де-вочка", Font_16x26,
					color_text, color_back_now);
			break;
		case 1:
			ST7789_WriteString(5, 50 + 26, "Ты самая лучшая в этой вселенной",
					Font_16x26, color_text, color_back_now);
			break;
		case 2:
			ST7789_WriteString(5, 50 + 26, "Ты богиня, спус-  тившаяся с небес",
					Font_16x26, color_text, color_back_now);
			break;
		case 3:
			ST7789_WriteString(5, 50 + 26,
					"Ты самая красивая девочка в этом ми-ре", Font_16x26, color_text,
					color_back_now);
			break;
		case 4:
			ST7789_WriteString(5, 50 + 26, "Ты лучшее, что со мной случалось",
					Font_16x26, color_text, color_back_now);
			break;
		case 5:
			ST7789_WriteString(5, 50 + 26, "Твоя красота поко-ряет мое сердце",
					Font_16x26, color_text, color_back_now);
			break;
		case 6:
			ST7789_WriteString(5, 50 + 26, "Моя любовь к тебе не знает границ",
					Font_16x26, color_text, color_back_now);
			break;
		case 7:
			ST7789_WriteString(5, 50 + 26, "Not fake, true lo-ve", Font_16x26,
					color_text, color_back_now);
			break;
		case 8:
			ST7789_WriteString(5, 50 + 26, "Горжусь тобой", Font_16x26,
			color_text, color_back_now);
			break;
		case 9:
			ST7789_WriteString(5, 50 + 26, "Ти самая умничка", Font_16x26,
					color_text, color_back_now);
			break;
		case 10:
			ST7789_WriteString(5, 50 + 26, "Очень тебя люблю иочень скучаю",
					Font_16x26, color_text, color_back_now);
			break;
		case 11:
			ST7789_WriteString(5, 50 + 26, "Приезжай скорее", Font_16x26, color_text,
					color_back_now);
			break;
		case 12:
			ST7789_WriteString(5, 50 + 26, "Ты лучше всех, ни-кого не слушай", Font_16x26, color_text,
					color_back_now);
			break;
		case 13:
			ST7789_WriteString(5, 50 + 26, "Красивая как ангел", Font_16x26, color_text,
					color_back_now);
			break;
		case 14:
			ST7789_WriteString(5, 50 + 26, "Очень скучаю, мой котик", Font_16x26, color_text,
					color_back_now);
			break;
		}
		if(auto_change==0) ST7789_WriteString(5, 200, "Авто смена нет", Font_16x26, color_text,
				color_back_now);
		else ST7789_WriteString(5, 200, "Авто смена да ", Font_16x26, color_text,
				color_back_now);
		if (menu_index != menu_index_past) {
					ST7789_DrawFilledRectangle(250, 5, 16 * 2, 240 - 11,
							color_back_now);
					menu_index_past = menu_index;
				}
				if(menu_index==0) ST7789_WriteString(250, 50, "<-", Font_16x26, color_text,
						color_back_now);
				else ST7789_WriteString(250, 200, "<-", Font_16x26, color_text,
						color_back_now);
		break;
	case STATE_MENU_ITEM_service:
		ST7789_WriteString(5, 5, "Сервис", Font_16x26, color_text, color_back_now);
		sprintf(buf,"v_bat: %.2fV",v_bat_filt);
		ST7789_WriteString(5, 31, buf, Font_16x26, color_text, color_back_now);
		sprintf(buf,"adc0: %4i",adc_data[0]);
		ST7789_WriteString(5, 57, buf, Font_16x26, color_text, color_back_now);
		sprintf(buf,"adc1: %4i",adc_data[1]);
		ST7789_WriteString(5, 83, buf, Font_16x26, color_text, color_back_now);
		sprintf(buf,"adc2: %4i",adc_data[2]);
		ST7789_WriteString(5, 109, buf, Font_16x26, color_text, color_back_now);
		if(need_reset==0) ST7789_WriteString(5, 135, "Сброс - нет", Font_16x26, color_text, color_back_now);
		else ST7789_WriteString(5, 135, "Сброс - да ", Font_16x26, color_text, color_back_now);
		ST7789_WriteString(250, 135, "<-", Font_16x26, color_text,
						color_back_now);
		break;
	}
}
void GoSleep(void)
{
	need_sleep=1;
	HAL_TIM_Base_Stop_IT(&htim4);
	HAL_ADC_Stop_DMA(&hadc1);
	ST7789_EnterSleepMode();
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}
uint8_t ReadButton(void) {
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); // <<< Замените на вашу кнопку
}

void HandleButton(void) {
    uint8_t b = ReadButton();

    if(buttonPrev && !b){ // нажали
        buttonDownTime = HAL_GetTick();
        buttonPressed = 1;
    }

    if(!buttonPrev && b){ // отпустили
        uint32_t dt = HAL_GetTick() - buttonDownTime;
        if(dt < LONG_PRESS_TIME)
            ButtonShortPressHandler();
        buttonPressed = 0;
    }

    // долгое нажатие
    if(buttonPressed && (HAL_GetTick() - buttonDownTime) > LONG_PRESS_TIME){
        buttonPressed = 0;
        ButtonLongPressHandler();
    }

    buttonPrev = b;
}
int16_t GetEncoder(void){
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

void HandleEncoder(void){
    int16_t cur = GetEncoder();
    int16_t diff = cur - lastEnc;

    if(diff == 0) return;

    if(diff > 0) EncoderRight();
    else EncoderLeft();

    lastEnc = cur;
}
void ButtonShortPressHandler(void){
	switch (state) {
	case STATE_MAIN_SCREEN:
		if (alarm_active == 1)
		{
			alarm_active=0;
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
		HAL_TIM_Base_Stop_IT(&htim4);
		}
		break;
	case STATE_MENU:
		state = menu_index + 2;
		ST7789_Fill_Color(color_back_now);
		menu_index = 0;
		break;
	case STATE_MENU_ITEM_Time:
		menu_index++;
		if (menu_index > 2)
			menu_index = 0;
		break;
	case STATE_MENU_ITEM_Date:
		menu_index++;
		if (menu_index > 3)
			menu_index = 0;
		break;
	case STATE_MENU_ITEM_Alarm:
		menu_index++;
		if (menu_index > 2)
			menu_index = 0;
		break;
	case STATE_MENU_ITEM_Color_Back:
		menu_index++;
		if (menu_index > 2)
			menu_index = 0;
		break;
	case STATE_MENU_ITEM_Bright:
		menu_index++;
		if (menu_index > 1)
			menu_index = 0;
		break;
	case STATE_MENU_ITEM_auto_off:
		menu_index++;
		if (menu_index > 1)
			menu_index = 0;
		break;
	case STATE_MENU_ITEM_fraze_select:
		menu_index++;
				if (menu_index > 1)
					menu_index = 0;
		break;
	case STATE_MENU_ITEM_service:
		break;
	}
}

void ButtonLongPressHandler(void){
	switch (state) {
	case STATE_MAIN_SCREEN:
		ST7789_Fill_Color(color_back_now);
		state = STATE_MENU;
		break;

	case STATE_MENU:
		ST7789_Fill_Color(color_back_now);
		ST7789_DrawRectangle(290, 10, 310, 50, color_text);
		ST7789_DrawFilledRectangle(295, 5, 11, 6, color_text);
		menu_index=0;
		state = STATE_MAIN_SCREEN;
		break;
	case STATE_MENU_ITEM_Time:
		ST7789_Fill_Color(color_back_now);
		menu_index=0;
		sTime.Hours = hou_set;
		sTime.Minutes = min_set;
		sTime.Seconds = sec_set;
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		state = STATE_MENU;
		break;
	case STATE_MENU_ITEM_Date:
		ST7789_Fill_Color(color_back_now);
		menu_index=0;
		DateToUpdate.Date=dat_set;
		DateToUpdate.WeekDay=weekday_set;
		DateToUpdate.Month=mou_set;
		DateToUpdate.Year=year_set;
		HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
		state = STATE_MENU;
		break;
	case STATE_MENU_ITEM_Alarm:
		ST7789_Fill_Color(color_back_now);
		menu_index=0;
		if(alarm_set==1)
		{
		sAlarm.AlarmTime.Hours = hou_set_alarm;
		sAlarm.AlarmTime.Minutes = min_set_alarm;
		HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);
		}
		else
		{
			HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
		}
		//HAL_RTC_GetAlarm(&hrtc, &sAlarm, RTC_ALARM_A, RTC_FORMAT_BIN);
		state = STATE_MENU;
		break;
	case STATE_MENU_ITEM_Color_Back:
		if (color_back_blue <=0x5 && color_back_red<=0x5 && color_back_green<=0x5) color_text = WHITE;
		else color_text = BLACK;
		color_back_now = color_back;
		ST7789_Fill_Color(color_back_now);
		menu_index=0;
		state = STATE_MENU;
		break;
	case STATE_MENU_ITEM_Bright:
		ST7789_Fill_Color(color_back_now);
		menu_index=0;
		state = STATE_MENU;
		break;
	case STATE_MENU_ITEM_auto_off:
		ST7789_Fill_Color(color_back_now);
		time_set_to_switch_off = time_to_switch_off*1000;
		menu_index=0;
		state = STATE_MENU;
		break;
	case STATE_MENU_ITEM_fraze_select:
		ST7789_Fill_Color(color_back_now);
		menu_index = 0;
		state = STATE_MENU;
		break;
	case STATE_MENU_ITEM_service:
		if(need_reset==1) NVIC_SystemReset();
		ST7789_Fill_Color(color_back_now);
		menu_index = 0;
		state = STATE_MENU;
		break;
	}
}
void EncoderRight(void){
	switch (state) {
		case STATE_MAIN_SCREEN:
			break;
		case STATE_MENU:
			menu_index++;
			if(menu_index>7) menu_index=0;
			break;
		case STATE_MENU_ITEM_Time:
			if(menu_index==0)hou_set++;
			if(hou_set>23) hou_set=0;
			if(menu_index==1)min_set++;
			if(min_set>59) min_set=0;
			if(menu_index==2)sec_set++;
			if(sec_set>59) sec_set=0;
			break;
		case STATE_MENU_ITEM_Date:
			if(menu_index==0)dat_set++;
			if(dat_set>31) dat_set=1;
			if(menu_index==1)weekday_set++;
			if(weekday_set>6) weekday_set=0;
			if(menu_index==2)mou_set++;
			if(mou_set>12) mou_set=1;
			if(menu_index==3)year_set++;
			if(year_set>100) year_set=0;
			break;
		case STATE_MENU_ITEM_Alarm:
			if(menu_index==0)alarm_set++;
			if(alarm_set>1)alarm_set=0;
			if(menu_index==1)hou_set_alarm++;
			if(hou_set_alarm>23) hou_set_alarm=0;
			if(menu_index==2)min_set_alarm++;
			if(min_set_alarm>59) min_set_alarm=0;
			break;
		case STATE_MENU_ITEM_Color_Back:
			if(menu_index==0)color_back_red++;
			if(color_back_red>0x1F) color_back_red=0;
			if(menu_index==1)color_back_green++;
			if(color_back_green>0x3F) color_back_green=0;
			if(menu_index==2)color_back_blue++;
			if(color_back_blue>0x1F) color_back_blue=0;
			break;
		case STATE_MENU_ITEM_Bright:
			if(menu_index==0)auto_bright++;
			if(auto_bright>1) auto_bright=0;
			if(menu_index==1)Bright_set++;
			if(Bright_set>100)Bright_set=1;
			break;
		case STATE_MENU_ITEM_auto_off:
			if(menu_index==0) auto_switch_off++;
			if(auto_switch_off>1) auto_switch_off=0;
			if(menu_index==1) time_to_switch_off++;
			if(time_to_switch_off>60) time_to_switch_off=5;
			break;
		case STATE_MENU_ITEM_fraze_select:
			if(menu_index==0)
				{
				ST7789_DrawFilledRectangle(5, 50+26, 319-5, 80, color_back_now);
				number_fraze++;
				}
			if(number_fraze>24)number_fraze=0;
			if(menu_index==1) auto_change++;
			if(auto_change>1)auto_change=0;
			break;
		case STATE_MENU_ITEM_service:
			need_reset++;
			if(need_reset>1)need_reset=0;
			break;
		}
}

void EncoderLeft(void) {
	switch (state) {
	case STATE_MAIN_SCREEN:
		break;
	case STATE_MENU:
		if (menu_index == 0)
			menu_index = 7;
		else
			menu_index--;
		break;
	case STATE_MENU_ITEM_Time:
		if (hou_set == 0 && menu_index == 0)
			hou_set = 23;
		else if (menu_index == 0)
			hou_set--;
		if (min_set == 0 && menu_index == 1)
			min_set = 59;
		else if (menu_index == 1)
			min_set--;
		if (sec_set == 0 && menu_index == 2)
			sec_set = 59;
		else if (menu_index == 2)
			sec_set--;
		break;
	case STATE_MENU_ITEM_Date:
		if (dat_set == 1 && menu_index == 0)
			dat_set = 31;
		else if (menu_index == 0)
			dat_set--;
		if (weekday_set == 0 && menu_index == 1)
			weekday_set = 6;
		else if (menu_index == 1)
			weekday_set--;
		if (mou_set == 1 && menu_index == 2)
			mou_set = 12;
		else if (menu_index == 2)
			mou_set--;
		if (year_set == 0 && menu_index == 3)
			year_set = 99;
		else if (menu_index == 3)
			year_set--;
		break;
	case STATE_MENU_ITEM_Alarm:
		if (alarm_set == 0 && menu_index == 0)
			alarm_set = 1;
		else if (menu_index == 0)
			alarm_set--;
		if (hou_set_alarm == 0 && menu_index == 1)
			hou_set_alarm = 23;
		else if (menu_index == 1)
			hou_set_alarm--;
		if (min_set_alarm == 0 && menu_index == 2)
			min_set_alarm = 59;
		else if (menu_index == 2)
			min_set_alarm--;
		break;
	case STATE_MENU_ITEM_Color_Back:
		if (color_back_red == 0 && menu_index == 0)
			color_back_red = 0x1F;
				else if (menu_index == 0)
					color_back_red--;
				if (color_back_green == 0 && menu_index == 1)
					color_back_green = 0x3F;
				else if (menu_index == 1)
					color_back_green--;
				if (color_back_blue == 0 && menu_index == 2)
					color_back_blue = 0x1F;
				else if (menu_index == 2)
					color_back_blue--;
		break;
	case STATE_MENU_ITEM_Bright:
		if (auto_bright == 0 && menu_index == 0)
			auto_bright = 1;
		else if (menu_index == 0)
			auto_bright--;
		if (Bright_set == 1 && menu_index == 1)
			Bright_set = 100;
		else if (menu_index == 1)
			Bright_set--;
		break;
	case STATE_MENU_ITEM_auto_off:
		if (auto_switch_off == 0 && menu_index == 0)
			auto_switch_off = 1;
				else if (menu_index == 0)
					auto_switch_off--;
				if (time_to_switch_off == 5 && menu_index == 1)
					time_to_switch_off = 60;
				else if (menu_index == 1)
					time_to_switch_off--;
		break;
	case STATE_MENU_ITEM_fraze_select:
		if (number_fraze == 0 && menu_index == 0) {
			ST7789_DrawFilledRectangle(5, 50+26, 319-5, 80, color_back_now);
			number_fraze = 24;
		} else if (menu_index == 0)
		{
			ST7789_DrawFilledRectangle(5, 50+26, 319-5, 80, color_back_now);
			number_fraze--;
		}
		if (auto_change == 0 && menu_index == 1)
			auto_change = 1;
		else if (menu_index == 1)
			auto_change--;
		break;
	case STATE_MENU_ITEM_service:
		if(need_reset==0)need_reset=1;
		else need_reset--;
		break;
	}
}
void HAL_RTC_AlarmAEventCallback (RTC_HandleTypeDef * hrtc)
{
	alarm_active = 1;
	if(need_sleep==1)
	{
		WakeUp();
	}
	HAL_RTC_DeactivateAlarm(hrtc, RTC_ALARM_A);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim4);
}
uint32_t rtc_to_seconds(RTC_DateTypeDef *d, RTC_TimeTypeDef *t)
{
	struct tm timeinfo;
	timeinfo.tm_year = d->Year+100;
	timeinfo.tm_mon = d->Month-1;
	timeinfo.tm_mday = d->Date;
	timeinfo.tm_hour = t->Hours;
	timeinfo.tm_min = t->Minutes;
	timeinfo.tm_sec = t->Seconds;
	return mktime(&timeinfo);
}
void Now_Time(void)
{
	now_time = HAL_GetTick();
	if(now_time - time_was >=time_set_to_switch_off)
	{
		time_was = now_time;
		GoSleep();

	}
}

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
