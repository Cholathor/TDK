/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STIMCHANNELORDERSIZE 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

extern const int numberOfBursts;
enum WaveState {upBump = 0, shortFlat = 1, downBump = 2, longFlat = 3};
extern enum WaveState currentWaveState;

extern uint16_t *Line_LUT;

extern uint16_t intensityValues[4][2][64];
extern uint8_t currentIntensity;
extern uint8_t channelLevels[16];
extern uint8_t useIndividualIntensities;

extern uint16_t A_ARR;
extern uint16_t B_ARR;
extern uint16_t C_ARR;
extern uint16_t D_ARR;

extern uint16_t maxWaveNumber;
extern uint16_t timeBetweenBursts;
extern uint8_t toggleBurst;
extern uint16_t enableBurst;
uint16_t burstPauseIndex = 0;

extern uint8_t stimMode;
extern uint8_t SPIdata[];
extern bool enableSPITransmit;

extern uint16_t stimChannelOrder[STIMCHANNELORDERSIZE];
extern uint8_t stimChannelOrderIntensity[STIMCHANNELORDERSIZE];
extern uint8_t validStimChannelOrderValues;
extern uint8_t stimChannelBurstNumber[STIMCHANNELORDERSIZE];

uint8_t currentChannelBurstNumber = 0;

extern uint8_t stimChannelIndex;
extern uint16_t channelSwitchTime;

extern SPI_HandleTypeDef hspi1;
extern float baseIntensity;

extern uint8_t I2Cdata[1];
extern uint8_t I2CdataTMP[1];
extern uint8_t enableI2Ctransmit;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_dac_ch1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dac_ch1);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  //toggleBurst is controlled by sending "ENTER" through USB
  if (toggleBurst == 0)
  {
	  return;
  }

  //enableBurst counts the number of bursts remaining after the toggleBurst flag is set to true
  //if it is 0, we skip timeBetweenBursts * D_ARR / 1.25 microseconds of stimulation
  if (enableBurst == 0)
  {
	  if (burstPauseIndex < timeBetweenBursts) // == it's pausing time
	  {
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);

		  burstPauseIndex++;
	  } else //start a burst of waves
	  {
		  burstPauseIndex = 0;
		  htim2.Instance->ARR = A_ARR;
		  enableBurst = maxWaveNumber;
		  currentChannelBurstNumber++;
		  if (currentChannelBurstNumber >= stimChannelBurstNumber[stimChannelIndex])
		  {
			  currentChannelBurstNumber = 0;
			  stimChannelIndex++;
 			  if (stimChannelIndex >= validStimChannelOrderValues)
			  {
 				  stimChannelIndex = 0;
 				  toggleBurst = 0;
			  }
		  }
	  }
	  return;
  }


  switch (currentWaveState)
  {
  	  case upBump:
  		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
  		  htim2.Instance->ARR = A_ARR;
  		  hdma_dac_ch1.Instance->CMAR = (uint32_t) &(*intensityValues[2 * currentIntensity]); //horrible, have to redo this properly
  		  hdma_dac_ch1.Instance->CNDTR = 64;
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
  		  break;

  	  case shortFlat:
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);

  		  htim2.Instance->ARR = B_ARR;
  		  hdma_dac_ch1.Instance->CMAR = (uint32_t) &Line_LUT;
  		  hdma_dac_ch1.Instance->CNDTR = 64;
  		  break;

  	  case downBump:
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);

  		  htim2.Instance->ARR = C_ARR;
  		  hdma_dac_ch1.Instance->CMAR = (uint32_t) &(*intensityValues[2 * currentIntensity + 1]);
  		  hdma_dac_ch1.Instance->CNDTR = 64;
  		  break;

  	  case longFlat:
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);

  		  htim2.Instance->ARR = D_ARR;
  		  hdma_dac_ch1.Instance->CMAR = (uint32_t) &Line_LUT;
  		  hdma_dac_ch1.Instance->CNDTR = 64;
  		  break;
  }


  currentWaveState++;
  if (currentWaveState > longFlat)
  {
	  enableBurst--;
	  currentWaveState = upBump;
  }

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac1);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  	if (enableBurst != 0)
  	{
  		TIM6->ARR = 100;
	}
  	else
  	{
//  		if (stimChannelOrder[stimChannelIndex] == 16 || validStimChannelOrderValues == 0)
//  		{
//  			SPIdata[0] = 0;
//  			SPIdata[1] = 0;
//  			enableSPITransmit = true;
//  			return;
//  		}
		if (stimChannelBurstNumber[stimChannelIndex] == 0)
		{
			SPIdata[0] = 0;
			SPIdata[1] = 0;
			enableSPITransmit = true;
			stimChannelIndex++;
			if (stimChannelIndex >= validStimChannelOrderValues)
			{
				stimChannelIndex = 0;
			}
			return;
		}
		uint16_t tmpValue = stimChannelOrder[stimChannelIndex];

		SPIdata[0] = (uint8_t) (tmpValue >> 8);
		SPIdata[1] = tmpValue & 0xff;
		enableSPITransmit = true;


		uint8_t intensityLevel = stimChannelOrderIntensity[stimChannelIndex];
	    I2Cdata[0] = intensityLevel * baseIntensity;
	    I2CdataTMP[0] = intensityLevel * baseIntensity;
	    enableI2Ctransmit = 1;
  	}


  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
