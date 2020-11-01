/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN PV */
//uint32_t abc = 0;
volatile bool longPush = false;
volatile bool motorEnable = false;
volatile uint8_t target = 0x00U;
volatile uint8_t endPoint = 0x00U;

volatile bool stopAfterLongPush = false;

volatile uint32_t aeration_time_irq = 0;
volatile uint32_t forward_time_irq = 0;
volatile uint32_t back_time_irq = 0;

volatile bool longPushLock = false;
volatile bool enableWrite = false;

volatile uint32_t res_addr = 0;
myBuf_t flashBuffer[BUFFSIZE] = {0x22, 0x00};

volatile MotorState motorState = BRAKETOGND;
volatile Scenario selectScenario = WAIT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM3_Init(void);
void MX_ADC_Init(void);
void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_CRC_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_CRC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* PWM_Start */
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableCounter(TIM3);

  /* timer start */
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);

//  HAL_ADCEx_Calibration_Start(&hadc);
//  HAL_ADC_Start(&hadc);

  MotorDriver__init(FULLBRIDGE);
  LL_GPIO_ResetOutputPin(statusLed_GPIO_Port, statusLed_Pin);
  res_addr = flash_search_address(STARTADDR, BUFFSIZE * DATAWIDTH);
  read_last_data_in_flash(flashBuffer);
  while (flashBuffer[subTarget] == 0x22) {
    read_last_data_in_flash(flashBuffer);
    LL_GPIO_ResetOutputPin(statusLed_GPIO_Port, statusLed_Pin);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while (1) {
    longPush = false;
    if (!(LL_GPIO_ReadInputPort(back_GPIO_Port) & back_Pin)) {
      if ((HAL_GetTick() - back_time_irq) > logPushTime) {
        // засекли нажатие длинее 500мс
        if (longPushLock) {
          longPush = false;
        } else {
          longPush = true;
        }
      }
    }

    if (!(LL_GPIO_ReadInputPort(forward_GPIO_Port) & forward_Pin)) {
      if ((HAL_GetTick() - forward_time_irq) > logPushTime) {
        // засекли нажатие длинее 500мс
        if (longPushLock) {
          longPush = false;
        } else {
          longPush = true;
        }
      }
    }

    if (!(LL_GPIO_ReadInputPort(aeration_GPIO_Port) & aeration_Pin)) {
      if ((HAL_GetTick() - aeration_time_irq) > logPushTime) {
        // засекли нажатие длинее 500мс
        if (longPushLock) {
          longPush = false;
        } else {
          longPush = true;
        }
      }
    }

    //abc = HAL_ADC_GetValue(&hadc);
    //abc = (3.33/4095)*abc; // значение напряжения на пине.

    if (!longPush && stopAfterLongPush) {
      motorEnable = false;
      stopAfterLongPush = false;
    }

    if (flashBuffer[subTarget] == toAeration) {
      LL_GPIO_SetOutputPin(statusLed_GPIO_Port, statusLed_Pin);
      LL_GPIO_ResetOutputPin(statusLed_GPIO_Port, statusLed_Pin);
    }

    switch (selectScenario) {
      case BACK:
        LL_GPIO_SetOutputPin(statusLed_GPIO_Port, statusLed_Pin);
        backScenario();
        break;
      case FORWARD:
        LL_GPIO_SetOutputPin(statusLed_GPIO_Port, statusLed_Pin);
        forwardScenario();
        break;
      case CLOSED_BACK:
        LL_GPIO_SetOutputPin(statusLed_GPIO_Port, statusLed_Pin);
        closedBackScenario();
        break;
      case CLOSED_FRONT:
        LL_GPIO_SetOutputPin(statusLed_GPIO_Port, statusLed_Pin);
        closedFrontScenario();
        break;
      case WAIT:
        LL_GPIO_ResetOutputPin(statusLed_GPIO_Port, statusLed_Pin);
      default:
        MotorDriver__turnOffMotor();
        motorEnable = false;
        break;
    }

    if (longPush) {
      MotorDriver__turnOnMotor(motorState);
      stopAfterLongPush = true;
    } else {
      if (motorEnable) {
        MotorDriver__turnOnMotor(motorState);
      } else {
        MotorDriver__turnOffMotor();
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
#pragma clang diagnostic pop
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_6);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_HSI14_EnableADCControl();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI2_3_IRQn interrupt configuration */
  NVIC_SetPriority(EXTI2_3_IRQn, 0);
  NVIC_EnableIRQ(EXTI2_3_IRQn);
  /* EXTI4_15_IRQn interrupt configuration */
  NVIC_SetPriority(EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**ADC GPIO Configuration
  PB1   ------> ADC_IN9
  */
  GPIO_InitStruct.Pin = motorCurrentSense_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(motorCurrentSense_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_9);
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  ADC_InitStruct.Clock = LL_ADC_CLOCK_ASYNC;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_28CYCLES_5);
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);
  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, 0);
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 2399;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 2279;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM3 GPIO Configuration
  PA7   ------> TIM3_CH2
  */
  GPIO_InitStruct.Pin = PWM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(PWM_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);

  /* TIM14 interrupt Init */
  NVIC_SetPriority(TIM14_IRQn, 0);
  NVIC_EnableIRQ(TIM14_IRQn);

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  TIM_InitStruct.Prescaler = 959;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 12500;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM14, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM14);
  LL_TIM_SetOnePulseMode(TIM14, LL_TIM_ONEPULSEMODE_SINGLE);
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16);

  /* TIM16 interrupt Init */
  NVIC_SetPriority(TIM16_IRQn, 0);
  NVIC_EnableIRQ(TIM16_IRQn);

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  TIM_InitStruct.Prescaler = 959;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 12500;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM16);
  LL_TIM_SetOnePulseMode(TIM16, LL_TIM_ONEPULSEMODE_SINGLE);
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM17);

  /* TIM17 interrupt Init */
  NVIC_SetPriority(TIM17_IRQn, 0);
  NVIC_EnableIRQ(TIM17_IRQn);

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  TIM_InitStruct.Prescaler = 959;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 12500;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM17, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM17);
  LL_TIM_SetOnePulseMode(TIM17, LL_TIM_ONEPULSEMODE_SINGLE);
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(outA_GPIO_Port, outA_Pin);

  /**/
  LL_GPIO_ResetOutputPin(outB_GPIO_Port, outB_Pin);

  /**/
  LL_GPIO_ResetOutputPin(statusLed_GPIO_Port, statusLed_Pin);

  /**/
  GPIO_InitStruct.Pin = endPointA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(endPointA_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = endPointB_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(endPointB_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = outA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(outA_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = outB_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(outB_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = statusLed_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(statusLed_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = motorEN_DIAG_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(motorEN_DIAG_GPIO_Port, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE2);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE3);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE4);

  /**/
  LL_GPIO_SetPinPull(back_GPIO_Port, back_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(forward_GPIO_Port, forward_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(aeration_GPIO_Port, aeration_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(back_GPIO_Port, back_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(forward_GPIO_Port, forward_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(aeration_GPIO_Port, aeration_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_3;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

}

/* USER CODE BEGIN 4 */
void GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  enableWrite = true;
  if (GPIO_Pin == back_Pin) {
    selectScenario = WAIT;
    motorEnable = motorEnable == false;
    HAL_NVIC_DisableIRQ(back_EXTI_IRQn); // сразу же отключаем прерывания на этом пине
    back_time_irq = HAL_GetTick();
    LL_TIM_EnableIT_UPDATE(backTIM); // запускаем таймер
    LL_TIM_EnableCounter(backTIM);
    longPushLock = false;

    if (MotorDriver__getMotorState() == BRAKETOGND) {
      if (endPoint == closed || endPoint == toOpen) {
        selectScenario = BACK;
        return;
      }

      if (endPoint == open) {
        if (flashBuffer[subTarget] == toAeration) {
          selectScenario = CLOSED_BACK;
          return;
        }
      }

      if (endPoint == toAeration) {
        selectScenario = CLOSED_BACK;
        return;
      }
    }
  }

  if (GPIO_Pin == forward_Pin) {
    selectScenario = WAIT;
    motorEnable = motorEnable == false;
    HAL_NVIC_DisableIRQ(forward_EXTI_IRQn); // сразу же отключаем прерывания на этом пине
    forward_time_irq = HAL_GetTick();
    LL_TIM_EnableIT_UPDATE(forwardTIM); // запускаем таймер
    LL_TIM_EnableCounter(forwardTIM);
    longPushLock = false;

    if (MotorDriver__getMotorState() == BRAKETOGND) {
      if (endPoint == closed || endPoint == toAeration) {
        selectScenario = FORWARD;
        return;
      }

      if (endPoint == open) {
        if (flashBuffer[subTarget] == toOpen) {
          selectScenario = CLOSED_FRONT;
          return;
        }
      }

      if (endPoint == toOpen) {
        selectScenario = CLOSED_FRONT;
        return;
      }
    }
  }

  if (GPIO_Pin == aeration_Pin) {
    selectScenario = WAIT;
    motorEnable = motorEnable == false;
    HAL_NVIC_DisableIRQ(aeration_EXTI_IRQn); // сразу же отключаем прерывания на этом пине
    aeration_time_irq = HAL_GetTick();
    LL_TIM_EnableIT_UPDATE(aerationTIM); // запускаем таймер
    LL_TIM_EnableCounter(aerationTIM);
    longPushLock = false;

    if (MotorDriver__getMotorState() == BRAKETOGND) {
      if (endPoint == closed || endPoint == toAeration) {
        selectScenario = FORWARD;
        return;
      }

      if (endPoint == open) {
        if (flashBuffer[subTarget] == toOpen) {
          selectScenario = CLOSED_FRONT;
          return;
        }
        if (flashBuffer[subTarget] == toAeration) {
          selectScenario = CLOSED_BACK;
          return;
        }
      }

      if (endPoint == toOpen) {
        selectScenario = CLOSED_FRONT;
        return;
      }
    }
  }
}

void TIM1_Callback(void) {
  if (LL_TIM_IsActiveFlag_UPDATE(TIM1)) {
    LL_TIM_ClearFlag_UPDATE(TIM1);
    getEndPointStatus();
  }
}

void BACK_Callback(void) {
  if (LL_TIM_IsActiveFlag_UPDATE(backTIM)) {
    LL_TIM_ClearFlag_UPDATE(backTIM);
    LL_TIM_DisableIT_UPDATE(backTIM); // останавливаем таймер
    LL_GPIO_ResetOutputPin(statusLed_GPIO_Port, statusLed_Pin);// разрешаем кнопать не чаще чем раз в 200мс

    __HAL_GPIO_EXTI_CLEAR_IT(back_Pin);  // очищаем бит EXTI_PR
    NVIC_ClearPendingIRQ(back_EXTI_IRQn); // очищаем бит NVIC_ICPRx
    HAL_NVIC_EnableIRQ(back_EXTI_IRQn);   // включаем внешнее прерывание
  }
}

void FORWARD_Callback(void) {
  if (LL_TIM_IsActiveFlag_UPDATE(forwardTIM)) {
    LL_TIM_ClearFlag_UPDATE(forwardTIM);
    LL_TIM_DisableIT_UPDATE(forwardTIM); // останавливаем таймер
    LL_GPIO_ResetOutputPin(statusLed_GPIO_Port, statusLed_Pin);// разрешаем кнопать не чаще чем раз в 200мс

    __HAL_GPIO_EXTI_CLEAR_IT(forward_Pin);  // очищаем бит EXTI_PR
    NVIC_ClearPendingIRQ(forward_EXTI_IRQn); // очищаем бит NVIC_ICPRx
    HAL_NVIC_EnableIRQ(forward_EXTI_IRQn);   // включаем внешнее прерывание
  }
}

void AERATION_Callback(void) {
  if (LL_TIM_IsActiveFlag_UPDATE(aerationTIM)) {
    LL_TIM_ClearFlag_UPDATE(aerationTIM);
    LL_TIM_DisableIT_UPDATE(aerationTIM); // останавливаем таймер
    LL_GPIO_ResetOutputPin(statusLed_GPIO_Port, statusLed_Pin);// разрешаем кнопать не чаще чем раз в 200мс

    __HAL_GPIO_EXTI_CLEAR_IT(aeration_Pin);  // очищаем бит EXTI_PR
    NVIC_ClearPendingIRQ(aeration_EXTI_IRQn); // очищаем бит NVIC_ICPRx
    HAL_NVIC_EnableIRQ(aeration_EXTI_IRQn);   // включаем внешнее прерывание
  }
}

void backScenario(void) {
  if (endPoint == open && motorState == CLOCKWISE) {
    stopMotor();
    flashBuffer[subTarget] = toOpen;
    if (enableWrite) {
      enableWrite = false;
      write_to_flash(flashBuffer);
    }
    return;
  }
  motorEnable = true;
  motorState = CLOCKWISE;
  return;
}

void closedFrontScenario(void) {
  // сначала до проветривания и потом на закрытие.
  if (endPoint == toAeration) {
    // сменили сценарий закрытия.
    selectScenario = CLOSED_BACK;
    return;
  }
  motorEnable = true;
  motorState = COUNTERCLOCKWISE;
  return;
}

void closedBackScenario(void) {
  if (endPoint == closed) {
    stopMotor();
    return;
  }
  motorEnable = true;
  motorState = CLOCKWISE;
  return;
}

void forwardScenario(void) {
  if (endPoint == open && motorState == COUNTERCLOCKWISE) {
    flashBuffer[subTarget] = toAeration;
    stopMotor();
    if (enableWrite) {
      enableWrite = false;
      write_to_flash(flashBuffer);
    }
    return;
  }
  motorEnable = true;
  motorState = COUNTERCLOCKWISE;
}

void stopMotor(void) {
  motorEnable = false;
  motorState = BRAKETOGND;
  longPushLock = true;
  selectScenario = WAIT;
}

void getEndPointStatus() {
  endPoint = (LL_GPIO_ReadInputPort(endPointA_GPIO_Port) & endPointA_Pin) |
             (LL_GPIO_ReadInputPort(endPointB_GPIO_Port) & endPointB_Pin);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
