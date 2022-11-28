/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4s5i_iot01_qspi.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "arm_math.h" //complex header file that works differently for different Cortex processor
#include <stdio.h>
#include <string.h>
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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId GetSpeedAndDirHandle;
osThreadId AcceleroSensorHandle;
osThreadId CounterDirGameHandle;
/* USER CODE BEGIN PV */
// Sampling rate = 20 kHz, duration = 2s, -> 40 000 samples per audio
// We have 10 audios, int = 4 bytes ->
#define SEQUENCE_LENGTH 40000
#define NUMBER_OF_DIRECTION 4
#define NUMBER_OF_DIGITS 5
int32_t SEQUENCE[SEQUENCE_LENGTH];
int32_t SEQUENCE_COPY[SEQUENCE_LENGTH];
int32_t addressDigits[10] = {0x000000, 0x030000, 0x060000, 0x090000, 0x0C0000, 0x0F0000, 0x120000, 0x150000, 0x180000, 0x1B0000};
int32_t addressDirections[4] = {0x1E0000, 0x210000, 0x240000, 0x270000}; // HVSF
uint32_t pushButtonCounter = 0;

//Selecting Game Modes
uint8_t recorder = 0;
uint8_t player = 1;
uint8_t directionGame = 1;
uint8_t digitGame = 0;

uint32_t addr = 0x000000;
uint8_t seqDigits[NUMBER_OF_DIGITS] = {4,1,4,7,9};
uint8_t seqDirections[NUMBER_OF_DIRECTION] = {'X', 'x', 'Y', 'y'};
uint32_t pressed = 0;
uint8_t addressDigitIndex = 0;
uint8_t addressDirectionIndex = 0;
// LEONIDAS VARIABLES
//DIGITS
uint8_t res;
uint8_t *seq;
uint8_t digits[] = "41479";
uint8_t digit_answer[NUMBER_OF_DIGITS +1];

//DIRECTIONS
uint8_t direction_answer[6];
//uint8_t seqDirections[]= "XxYy";

//UART
uint8_t start_yn[2];//player 1 or 2 answer
uint8_t recorded_digits[6];//user types the desired numbers here
uint8_t game_mode[2];//game mode 1 or 0 answer
uint8_t recorder_game_mode[2];//game mode 1 or 0 answer

uint8_t digit_reply[6];
uint8_t digit_ready[2];
uint8_t int_converter[6];

uint8_t direction_reply[6];
uint8_t digit_ready[2];

uint8_t user_Digit_answer[6];
uint8_t user_Direction_answer[6];

// Accelerometer
int16_t accelerometer[3];
int16_t acc_x1; //initial acc. x value
int16_t acc_y1;
float32_t maxX2;
float32_t maxY2;
float32_t  arrayX[2000];
float32_t arrayY[2000];
int16_t arrayIndex = 0;
uint32_t maxIndexX;
uint32_t maxIndexY;

char directionResult[NUMBER_OF_DIRECTION];
uint32_t counterRestart;
int8_t resultIndex = 0;
int8_t counterInitial = 0;
int8_t startedMoving = 0;
int8_t recordingDirectionIndex = 0;

///MESSAGES
uint8_t startMessage[] = "Let's Test Your Memory! \r\n"
					   "Have you played before?\r\n"
					   "1: Yes \r\n"
					   "0: No \r\n";

uint8_t playerMessage[] = "Then, proceed to choose the game mode! \r\n";

uint8_t chooseModeMessage[] = "Choose The Game Mode: \r\n"
							"0: Memory Digits \r\n"
							"1: Memory Directions \r\n";

uint8_t waitForSpeakerDigitMessage[] = "Starting Memory Digits..\r\nPlaying numbers..wait until completion!\r\n";
uint8_t waitForSpeakerDirectionMessage[] = "Starting Memory Directions..\r\nPlaying seqDirections..wait until completion!\r\n";

uint8_t winMessage[] = "You got it right! \r\n";

uint8_t lossMessage[] = "You got it wrong! \r\n"
		               "Game over!\r\n"
					   "The answer was:\r\n";

uint8_t startTypingMessage[] = "Enter what you remember: \r\n";

uint8_t startPlayer2Message[] = "Player 2 \r\n";

uint8_t recorderMessage[] = "Begin typing the 5 digits you want player 2 to memorize \r\n";
uint8_t direcRecorderMessage[] = "Begin typing the 5 coordinates you want player 2 to memorize \r\n";

uint8_t clearCommand[] =  "\033[1J";

uint8_t here[] =  "HERE_____\r\n";
//END MESSAGES
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void StartGetSpeedAndDir(void const * argument);
void StartAcceleroSensor(void const * argument);
void StartCounterDirGame(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void get_ACC_XY_InitialPosition();
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
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_DFSDM1_Init();
  MX_OCTOSPI1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  BSP_ACCELERO_Init();
  BSP_QSPI_Init();
  HAL_TIM_Base_Start_IT(&htim2);

  if(recorder) { // 3 blocks per sound (digits OR seqDirections/speed)
	  //10 digits * 3 = 30 blocks to erase
	  //2 seqDirections (vertical/horizontal) + 2 speeds (fast/slow) = 4 *3 = 12
	  for(int i = 1; i < 42; i++) {
		  if(BSP_QSPI_Erase_Block((uint32_t) addr) != QSPI_OK)
		  	Error_Handler();
		  addr = 0x010000*i;
	  }
  }

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of GetSpeedAndDir */
  osThreadDef(GetSpeedAndDir, StartGetSpeedAndDir, osPriorityNormal, 0, 128);
  GetSpeedAndDirHandle = osThreadCreate(osThread(GetSpeedAndDir), NULL);

  /* definition and creation of AcceleroSensor */
  osThreadDef(AcceleroSensor, StartAcceleroSensor, osPriorityIdle, 0, 128);
  AcceleroSensorHandle = osThreadCreate(osThread(AcceleroSensor), NULL);

  /* definition and creation of CounterDirGame */
  osThreadDef(CounterDirGame, StartCounterDirGame, osPriorityIdle, 0, 128);
  CounterDirGameHandle = osThreadCreate(osThread(CounterDirGame), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 118;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 34;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(redLED_GPIO_Port, redLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(greenLED_GPIO_Port, greenLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : redLED_Pin */
  GPIO_InitStruct.Pin = redLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(redLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : pushButton_Pin */
  GPIO_InitStruct.Pin = pushButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(pushButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : greenLED_Pin */
  GPIO_InitStruct.Pin = greenLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(greenLED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == pushButton_Pin) {
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		HAL_GPIO_TogglePin(greenLED_GPIO_Port, greenLED_Pin);

		if(recorder)
			HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, SEQUENCE, SEQUENCE_LENGTH);

		if(player && digitGame) {
			if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  addressDigits[seqDigits[addressDigitIndex]], sizeof(SEQUENCE)) != QSPI_OK)
				Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);
		} else if(player && directionGame) {

		}
	}
}


void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	if(player && digitGame) {
		addressDigitIndex = addressDigitIndex + 1;
		if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  addressDigits[seqDigits[addressDigitIndex]], sizeof(SEQUENCE)) != QSPI_OK)
			Error_Handler();
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);

		if (addressDigitIndex == NUMBER_OF_DIGITS) {
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			HAL_UART_Transmit(&huart1, startTypingMessage, sizeof(startTypingMessage), 100);
		}
	} else if (player && directionGame) {
		if(addressDirectionIndex == 0) {
			if(seqDirections[addressDirectionIndex] == 'x' || seqDirections[addressDirectionIndex] == 'X') {
				if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  addressDirections[0], sizeof(SEQUENCE)) != QSPI_OK)
					Error_Handler();
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);
			} else if (seqDirections[addressDirectionIndex] == 'y' || seqDirections[addressDirectionIndex] == 'Y') {
				if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  addressDirections[1], sizeof(SEQUENCE)) != QSPI_OK)
					Error_Handler();
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);
			}
			recordingDirectionIndex++;
		}
		addressDirectionIndex = addressDirectionIndex + 1;
		if(addressDirectionIndex % recordingDirectionIndex == 0 ) {
			if(seqDirections[addressDirectionIndex] == 'x' || seqDirections[addressDirectionIndex] == 'y') {
				if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  addressDirections[2], sizeof(SEQUENCE)) != QSPI_OK)
					Error_Handler();
				recordingDirectionIndex++;
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);
			} else if (seqDirections[addressDirectionIndex] == 'X' || seqDirections[addressDirectionIndex] == 'Y') {
				if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  addressDirections[3], sizeof(SEQUENCE)) != QSPI_OK)
					Error_Handler();
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);
			}
			recordingDirectionIndex++;
		} else if (addressDirectionIndex % recordingDirectionIndex == 1) {
			if(seqDirections[addressDirectionIndex] == 'x' || seqDirections[addressDirectionIndex] == 'X') {
				if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  addressDirections[0], sizeof(SEQUENCE)) != QSPI_OK)
					Error_Handler();
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);
			} else if (seqDirections[addressDirectionIndex] == 'y' || seqDirections[addressDirectionIndex] == 'Y') {
				if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  addressDirections[1], sizeof(SEQUENCE)) != QSPI_OK)
					Error_Handler();
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);
			}
			recordingDirectionIndex++;
		}
		if (addressDirectionIndex == NUMBER_OF_DIRECTIONS) {
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			HAL_UART_Transmit(&huart1, startTypingMessage, sizeof(startTypingMessage), 100);
		}
	}
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter ) {

	if(recorder && digitGame) {
	HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
		for(uint32_t i = 0 ; i < SEQUENCE_LENGTH; i++ ){
			SEQUENCE[i] = SEQUENCE[i] >> 8; // 24 bit signed  :  −8,388,608 : 8,388,607
			if(SEQUENCE[i] < 0 ) {
				SEQUENCE[i]= SEQUENCE[i]+ (1<<24);
			}
			if( SEQUENCE[i] >= 4096) {
				SEQUENCE[i] = SEQUENCE[i] >> 12;
			}
		}
		if(BSP_QSPI_Write((uint8_t *) SEQUENCE, (uint32_t) addressDigits[pushButtonCounter], sizeof(SEQUENCE)) != QSPI_OK){
			Error_Handler();
		}
		if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t) addressDigits[pushButtonCounter], sizeof(SEQUENCE)) != QSPI_OK){
			Error_Handler();
		}
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);
		pushButtonCounter = (pushButtonCounter + 1) % 10;
	}


	if(recorder && directionGame) {
	HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
		for(uint32_t i = 0 ; i < SEQUENCE_LENGTH; i++ ){
			SEQUENCE[i] = SEQUENCE[i] >> 8; // 24 bit signed  :  −8,388,608 : 8,388,607
			if(SEQUENCE[i] < 0 ) {
				SEQUENCE[i]= SEQUENCE[i]+ (1<<24);
			}
			if( SEQUENCE[i] >= 4096) {
				SEQUENCE[i] = SEQUENCE[i] >> 12;
			}
		}
		if(BSP_QSPI_Write((uint8_t *) SEQUENCE, (uint32_t) addressDirections[pushButtonCounter], sizeof(SEQUENCE)) != QSPI_OK){
			Error_Handler();
		}
		if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t) addressDirections[pushButtonCounter], sizeof(SEQUENCE)) != QSPI_OK){
			Error_Handler();
		}
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);
		pushButtonCounter = (pushButtonCounter + 1) % 4;
	}

}

//get the data byte by byte
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if (start_yn[0] == '0'){
	  //you are a recorder (player 1)
	  recorder = 1;
	  player = 0;
	  memset(start_yn,0,2);
	  // send player message
	  HAL_UART_Transmit(&huart1, chooseModeMessage, sizeof(chooseModeMessage), 100);
	  //game mode 0 or 1
	  HAL_UART_Receive_IT(&huart1, recorder_game_mode, 2);
  }

  if (recorder_game_mode[0] == '1'){
	  directionGame = 1;
	  digitGame = 0;
	  HAL_UART_Transmit(&huart1, direcRecorderMessage, sizeof(direcRecorderMessage), 100);
	  //receive the answer in digit_reply
	  HAL_UART_Receive_IT(&huart1, direction_reply, 6);


  }

  if (direction_reply[0] != '\000'){

	  HAL_UART_Transmit(&huart1, clearCommand, sizeof(clearCommand), 100);

	  HAL_UART_Transmit(&huart1, startPlayer2Message, sizeof(startPlayer2Message), 100);

	  ////TODO: change this for N S E W
//	  for (int i = 0; i < (sizeof(direction_reply))/(sizeof(direction_reply[0])); i++){
//		  int_converter[i] = digit_reply[i] - '0';
//   	  }
//	  seq = int_converter;
//	  strcpy(digit_answer,  digit_reply);
	  ///////////////////////////////////////

	  direction_reply[0] = '\000';

	  HAL_UART_Transmit(&huart1, waitForSpeakerDirectionMessage, sizeof(waitForSpeakerDirectionMessage), 100);

	  //send to HAL_DAC...once cmplt user should move the board
//	  if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  address[seq[0]], sizeof(SEQUENCE)) != QSPI_OK)
//	  					Error_Handler();
//	  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);

	  HAL_UART_Receive_IT(&huart1, user_Direction_answer, 6);
  }


  /////////////// Recorder Digits ////////////////////////
  if (recorder_game_mode[0] == '0'){
	  memset(recorder_game_mode,0,2);
	  directionGame = 0;
	  digitGame = 1;
	  HAL_UART_Transmit(&huart1, recorderMessage, sizeof(recorderMessage), 100);

	  //receive the answer in digit_reply
	  HAL_UART_Receive_IT(&huart1, digit_reply, 6);

  }


  if (digit_reply[0] != '\000'){

	  HAL_UART_Transmit(&huart1, clearCommand, sizeof(clearCommand), 100);

	  HAL_UART_Transmit(&huart1, startPlayer2Message, sizeof(startPlayer2Message), 100);


	  for (int i = 0; i < (sizeof(digit_reply))/(sizeof(digit_reply[0])); i++){
		  int_converter[i] = digit_reply[i] - '0';
   	  }
	  seq = int_converter;
	  strcpy(digit_answer,  digit_reply);

	  digit_reply[0] = '\000';

	  HAL_UART_Transmit(&huart1, waitForSpeakerDigitMessage, sizeof(waitForSpeakerDigitMessage), 100);

	  //send to HAL_DAC...once cmplt should ask user to start typing
	  if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  address[seq[0]], sizeof(SEQUENCE)) != QSPI_OK)
	  					Error_Handler();
	  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);


	  HAL_UART_Receive_IT(&huart1, user_Digit_answer, 6);

  }
  /////////////////////////////////////////////////////////////


  if (start_yn[0] == '1'){
	  //you are a player (player 2)
	  //game mode 0 or game mode 1
	  memset(start_yn,0,2);
	  player = 1 ;
	  recorder = 0;
	  // send player message
	  HAL_UART_Transmit(&huart1, playerMessage, sizeof(playerMessage), 100);
	  HAL_UART_Transmit(&huart1, chooseModeMessage, sizeof(chooseModeMessage), 100);

	  HAL_UART_Receive_IT(&huart1, game_mode, 2);
  }


/////////////////////// PLAYER Digit ////////////////////////
  if (game_mode[0] == '0'){
	  //you chose game mode 0 (Digits)
	  //get ready to hear the digits
	  strcpy(digit_answer,  digits);
	  seq = seqDigits;
	  directionGame = 0 ;
	  digitGame = 1;
	  memset(game_mode,0,2);
	  HAL_UART_Transmit(&huart1, waitForSpeakerDigitMessage, sizeof(waitForSpeakerDigitMessage), 100);

	  //send to HAL_DAC...once cmplt should ask user to start typing
	  if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  address[seq[0]], sizeof(SEQUENCE)) != QSPI_OK)
	  					Error_Handler();
	  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);

	  //HAL_UART_Transmit(&huart1, here, sizeof(here), 100);

	  HAL_UART_Receive_IT(&huart1, user_Digit_answer, 6);
  }


  //check the answer for digits
  if (user_Digit_answer[0] != '\000'){

	  res= strncmp(user_Digit_answer, digit_answer, 5);
	  if (res == 0){
		  HAL_UART_Transmit(&huart1, winMessage, sizeof(winMessage), 100);
	  }
	  else{
		  HAL_UART_Transmit(&huart1, lossMessage, sizeof(lossMessage), 100);
		  HAL_UART_Transmit(&huart1, digit_answer, sizeof(digit_answer), 100);
	  }

	  user_Digit_answer[0] = '\000';

  }

  ////////////////// PLATER Direction //////////////////////////////////////
  if (game_mode[0] == '1'){
	  //you chose game mode 1 (Directions)
	  //get ready to hear the seqDirections
	  memset(game_mode,0,2);
	  directionGame = 1;
	  digitGame = 0;
	  HAL_UART_Transmit(&huart1, waitForSpeakerDirectionMessage, sizeof(waitForSpeakerDirectionMessage), 100);
	  if(seqDirections[addressDirectionIndex] == 'x' || seqDirections[addressDirectionIndex] == 'y' ) {
		  if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  addressDirections[2], sizeof(SEQUENCE)) != QSPI_OK)
			  Error_Handler();
		  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);
	  } else if(seqDirections[addressDirectionIndex] == 'X' || seqDirections[addressDirectionIndex] == 'Y') {
		  if(BSP_QSPI_Read((uint8_t *) SEQUENCE_COPY, (uint32_t)  addressDirections[3], sizeof(SEQUENCE)) != QSPI_OK)
			  Error_Handler();
		  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) SEQUENCE_COPY, SEQUENCE_LENGTH, DAC_ALIGN_12B_R);
	  }
	  recordingDirectionIndex++;
  }

  ///// might not need this
  //check the answer for seqDirections
  if (directionResult[NUMBER_OF_DIRECTIONS] != '\000'){
	  res= strncmp(directionResult, seqDirections, NUMBER_OF_DIRECTIONS);
	  if (res == 0){
		  HAL_UART_Transmit(&huart1, winMessage, sizeof(winMessage), 100);
	  } else{
		  HAL_UART_Transmit(&huart1, lossMessage, sizeof(lossMessage), 100);
		  HAL_UART_Transmit(&huart1, seqDirections, sizeof(seqDirections), 100);
	  }
	  user_Direction_answer[NUMBER_OF_DIRECTIONS] = '\000';
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartGetSpeedAndDir */
/**
  * @brief  Function implementing the GetSpeedAndDir thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGetSpeedAndDir */
void StartGetSpeedAndDir(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    if(counterRestart == 30) {
		for(int i = 0 ; i < arrayIndex; i ++) { //reinitializing arrays
			arrayX[i] = 0;
			arrayY[i] = 0;
		}
		if(maxX2 > maxY2) {
			if(maxX2 > 500) {
				directionResult[resultIndex] = 'X'; //fast horizontal
			} else {
				directionResult[resultIndex] = 'x'; //slow horizontal
			}
		} else {
			if(maxY2 > 500) {
				directionResult[resultIndex] = 'Y'; //fast vertical
			} else {
				directionResult[resultIndex] = 'y'; //slow vertical
			}
		}
		resultIndex++;
		arrayIndex = 0;
		startedMoving = 0;
		counterRestart = 0;
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartAcceleroSensor */
/**
* @brief Function implementing the AcceleroSensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAcceleroSensor */
void StartAcceleroSensor(void const * argument)
{
  /* USER CODE BEGIN StartAcceleroSensor */
  BSP_ACCELERO_Init();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    if(player && directionGame && !counterInitial) {
    	  osDelay(10000);
   		  BSP_ACCELERO_AccGetXYZ(accelerometer);
   		  acc_x1 = accelerometer[0];
   		  acc_y1 = accelerometer[1];
   		  counterInitial++;
   	  }
    if(player && directionGame && (resultIndex < NUMBER_OF_DIRECTION)) {
   		BSP_ACCELERO_AccGetXYZ(accelerometer);
   		if(accelerometer[0]- acc_x1  > 100 || accelerometer[1] - acc_y1 > 100) {
   		  arrayX[arrayIndex] = (float32_t) accelerometer[0];
   		  arrayY[arrayIndex] = (float32_t) accelerometer[1];
   		  arrayIndex++;
   		  arm_max_f32(&arrayX, (uint32_t) 2000,  &maxX2,  &maxIndexX);
   		  arm_max_f32(&arrayY, (uint32_t) 2000,  &maxY2,  &maxIndexY);
   		  startedMoving = 1;
   		}
   	  }
  }
  /* USER CODE END StartAcceleroSensor */
}

/* USER CODE BEGIN Header_StartCounterDirGame */
/**
* @brief Function implementing the CounterDirGame thread.
*
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCounterDirGame */
void StartCounterDirGame(void const * argument)
{
  /* USER CODE BEGIN StartCounterDirGame */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    if(startedMoving) {
    	osDelay(100);
    	counterRestart++;
    }
  }
  /* USER CODE END StartCounterDirGame */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//	HAL_GPIO_WritePin(redLED_GPIO_Port, redLED_Pin, GPIO_PIN_RESET);
	__BKPT();

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
