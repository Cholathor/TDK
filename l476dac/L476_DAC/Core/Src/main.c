/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "wavegen.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Number of sample points in waveforms
#define NS 64
#define voltageconstant 1.225
#define STIMCHANNELORDERSIZE 30

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int pow_10[10] = {
        1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000
    }; //powers of ten

enum WaveState {upBump = 0, shortFlat = 1, downBump = 2, longFlat = 3};
enum WaveState currentWaveState = longFlat;

const double dev = 10.0; //deviation of bell curves

// A wave duration default value: 250 us
// (1/80e6) * 64 * 312.5 = 0.00025s = 250 us
// (1/(CLK) * NS * ARR = duration) -> ARR = duration * CLK/NS
// where CLK = 80e6, NS = 64
// going below ARR = 64 may cause problems, however it is possible

//these are set into TIM2 during the DMA interrupt calls (see stm32l4xx_it.c)
uint16_t A_ARR = 312;
uint16_t B_ARR = 312;
uint16_t C_ARR = 312;
uint16_t D_ARR = 3125;




uint32_t intensityValues[8][NS]; //stores the 4 levels of intensity bell curves (8 = 4 * 2 because we store the negative parts as well)
uint16_t voltageLevels[4] = {375, 750, 1125, 1500}; //this is for generating the 4 levels of intensities
uint8_t channelLevels[16]; //for every individual channel the 4 levels of intensity can be set during runtime
uint8_t useIndividualIntensities = 1; //if this is 0, all channels have the same intensity by default
uint8_t currentIntensity = 3; //the default intensity level is 3 (voltageLevels[3] = 1500)



uint32_t Line_LUT[NS] = {1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910,
		1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910,
		1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910,
		1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910,
		1910, 1910, 1910, 1910};

//this is used for offsetting, not using this at the moment
uint32_t Line_LUT_tmp[NS] = {1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910,
		1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910,
		1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910,
		1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1910,
		1910, 1910, 1910, 1910};

//UART data buffer
uint8_t Rx_data[7];

//number of waves in a burst
uint16_t maxWaveNumber = 10;
//this gets toggled by sending "ENTER" through USB
uint8_t toggleBurst = 0;
uint16_t enableBurst;

uint16_t amplitude = voltageconstant * 1500;
uint16_t offset = voltageconstant * 1550;
bool startParseInput = false;
bool enableSPITransmit = false;
bool setVoltage = false;
bool setOffset = false;

uint8_t stimMode = 0;
uint16_t channelSwitchTime = 4999;

uint16_t stimChannelOrder[STIMCHANNELORDERSIZE];
uint8_t stimChannelOrderIntensity[STIMCHANNELORDERSIZE];
uint8_t validStimChannelOrderValues = 0;
uint8_t stimChannelBurstNumber[STIMCHANNELORDERSIZE] = {0};
uint8_t stimChannelIndex = 0;
//this is in units of timeBetweenBursts * D_ARR * CLK / NS [us]
uint16_t timeBetweenBursts = 50;

uint8_t SPIdata[2] = {0b00000000, 0b00000000};

uint8_t I2Cdata[1] = {0b00000110};
uint8_t I2CdataTMP[1] = {0b00000110};
float baseIntensity = 1.0f;

uint8_t enableI2Ctransmit = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */



#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

//void XferCpltCallback(DMA_HandleTypeDef *hdma)
//{
//	tick++;
//}


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

	for (int i = 0; i < 4; i++)
	{
		fillWithBell(intensityValues[2*i], NS, NS / 2, dev, voltageconstant * voltageLevels[i], 1910);
		fillWithBell(intensityValues[2*i + 1], NS, NS / 2, dev, -voltageconstant * voltageLevels[i], 1910);
	}
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
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("HELLO \r\n");
  printf("WORLD \r\n");


  //this is for debug purposes
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

  HAL_UART_Receive_IT(&huart2, Rx_data, 7);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *) Line_LUT, NS, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  //default PWM duty cycle: 30%
  TIM3->CCR1 = 300;

  //disable the half transfer complete interrupt for the DMA
  __HAL_DMA_DISABLE_IT(&hdma_dac_ch1, DMA_IT_HT);

  //uint8_t testData[2] = {255, 255};


  HAL_I2C_Master_Transmit(&hi2c3, 0b0000000001011110, 0b00000000, 1, 100);
  HAL_I2C_Master_Transmit(&hi2c1, 0b0000000001011110, 0b00000000, 1, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if (enableSPITransmit)
	 {

		 HAL_SPI_Transmit(&hspi1, SPIdata, 2, 1000);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		 enableSPITransmit = false;
		 for (int i = 0; i < 20; i++)
		 {
			 i--;
			 i++;
		 }
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		 HAL_SPI_Transmit(&hspi1, SPIdata, 2, 1000);
	 }

	 if (enableI2Ctransmit == 1)
	 {

		 enableI2Ctransmit = 0;
		 if (I2Cdata[0] > 127)
		 {
			 I2Cdata[0] = 127;
			 I2CdataTMP[0] = 127;
		 }
		 if (I2Cdata[0] < 6)
		 {
			 I2Cdata[0] = 6;
			 I2CdataTMP[0] = 6;
		 }
		 HAL_StatusTypeDef i2c3Status = HAL_I2C_Master_Transmit(&hi2c3, 0b0000000001011110, I2Cdata, 1, 100);
		 HAL_StatusTypeDef i2c1Status = HAL_I2C_Master_Transmit(&hi2c1, 0b0000000001011110, I2CdataTMP, 1, 100);
		 if (i2c3Status != HAL_OK)
		 {
			  MX_I2C3_Init();
		 }
		 if (i2c1Status != HAL_OK)
		 {
			 MX_I2C1_Init();
		 }
	 }

	 if (startParseInput)
	 {
		 parseUserInput();
	 }

	 if (setVoltage)
	 {
		 setVoltage = false;
//		 if (setOffset)
//		 {
//			 setOffset = false;
//			 for (int i = 0; i < NS; i++)
//			 {
//				 Line_LUT[i] = offset;
//			 }
//		 }
	 }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_ENABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  sConfig.DAC_SampleAndHoldConfig.DAC_SampleTime = 20;
  sConfig.DAC_SampleAndHoldConfig.DAC_HoldTime = 700;
  sConfig.DAC_SampleAndHoldConfig.DAC_RefreshTime = 5;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00702991;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Period = 320;
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  sConfigOC.Pulse = 0;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 799;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


// check if "ENTER" is the string inside Rx_data
bool startBurst()
{
	return (Rx_data[0] == 'E' && Rx_data[1] == 'N' && Rx_data[2] == 'T' && Rx_data[3] == 'E' && Rx_data[4] == 'R');
}


void parseUserInput()
{
	uint16_t tmpValue = 0;

	tmpValue = (Rx_data[1] - 48) * 100 + (Rx_data[2] - 48) * 10 + (Rx_data[3] - 48);
	tmpValue *= pow_10[Rx_data[4]-48];


	//1.25 = 80.0/64.0 (Main CLK / number of data points in bell curve arrays)
	switch (Rx_data[0])
	{
		case 'A':
			A_ARR = 1.25 * tmpValue;
			break;
		case 'B':
			B_ARR = 1.25 * tmpValue;
			break;
		case 'C':
			C_ARR = 1.25 * tmpValue;
			break;
		case 'D':
			D_ARR = 1.25 * tmpValue;
			break;
		case 'N':
			maxWaveNumber = tmpValue;
			break;
		case 'm':
			timeBetweenBursts = tmpValue;
			break;
		case 'P': //PWM duty cycle
			TIM3->CCR1 = tmpValue*10;
			break;
		case 'S': //spi transmit data (for analog switch)
			if (tmpValue == 17)
			{
				break;
			}
			if (tmpValue > 7)
			{
				SPIdata[0] = 1<<(tmpValue-8);
				SPIdata[1] = 0;
			} else
			{
				SPIdata[0] = 0;
				SPIdata[1] = 1<<tmpValue;
			}
			enableSPITransmit = true;
			break;
		case 'V':
			if (Rx_data[1] == '0')
			{
				useIndividualIntensities = 1;
				setVoltage = true;
				uint8_t channelIndex = (Rx_data[5] - '0') * 10 + Rx_data[6] - '0';
				channelLevels[channelIndex] = (Rx_data[2] - '0') * 100 + (Rx_data[3] - '0') * 10 + Rx_data[4] - '0';

			}
			//else
//			{
//				useIndividualIntensities = 0;
//				currentIntensity = (Rx_data[2] - '0') * 10 + Rx_data[3] - '0';
//				for (int i = 0; i < 16; i++)
//				{
//					channelLevels[i] = currentIntensity;
//				}
//				I2Cdata[0] = (Rx_data[2] - '0') * 100 + (Rx_data[3] - '0') * 10 + Rx_data[4] - '0';
//			}


			break;
		case 'O':
			offset = voltageconstant * tmpValue;
			if (offset > 4095)
			{
				offset = 4095;
			}
			if (offset < 0)
			{
				offset = 0;
			}
			setOffset = true;
			setVoltage = true;
			break;
		case 'M': //mode
			stimMode = tmpValue;
			break;
		case 's':


			;
            uint8_t channelOrderIndex = Rx_data[1];

			uint8_t channelOrderIntensity = Rx_data[2];

			uint16_t stimChannelIndexToSet = (Rx_data[3] << 8) +  Rx_data[4];

			uint8_t stimChannelNumberOfBurstsToSet = (Rx_data[5] - '0') * 10 + (Rx_data[6] - '0');

			stimChannelOrder[channelOrderIndex] = stimChannelIndexToSet;

			stimChannelOrderIntensity[channelOrderIndex] = channelOrderIntensity;

			stimChannelBurstNumber[channelOrderIndex] = stimChannelNumberOfBurstsToSet;


			break;
		case 'v':
			validStimChannelOrderValues = tmpValue;
			break;
		case 'R':
			//I2Cdata[0] = tmpValue;
			//enableI2Ctransmit = 1;
			baseIntensity = tmpValue / 127.0f;
			break;

	}
	startParseInput = false;
}

/*
 * gets called whenever data is sent through UART
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (startBurst())
	{
		toggleBurst = 1 - toggleBurst;

		if (toggleBurst == 1)
		{
			enableBurst = maxWaveNumber;
			stimChannelIndex = 0;
		}
		if (toggleBurst == 0)
		{
			htim2.Instance->ARR = D_ARR;
			hdma_dac_ch1.Instance->CMAR = (uint32_t) &Line_LUT;
		}

	} else
	if (!startParseInput)
	{
		startParseInput = true;
	}
	HAL_UART_Receive_IT(&huart2, Rx_data, 7);
}

/*
 * not using this at the moment
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
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
