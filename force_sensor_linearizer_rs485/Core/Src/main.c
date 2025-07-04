/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "crc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "force_sensor_lineralazer_lib.h"
#include "globals.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  uint32_t crc_test_value = 462;
  uint32_t crcInput_value = 0;
  uint32_t adc_uint32_Buffer[4] = {0};

  // Allow USART 1 to work
  NVIC_EnableIRQ(USART1_IRQn);

  USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
  LL_GPIO_ResetOutputPin(RE_DE_GPIO_Port, RE_DE_Pin); // Set MAX485 to Rx mode
  // Clear variables and arrays to calm my anxiety (￢‿￢ )
  memset(rxBuffer, 0, sizeof(rxBuffer));
  rxBufferCounter = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  LL_mDelay(1);
	  switch(rxBuffer[0]) // Pointer to command
		  {
			 case 'r':
				// ADC value equal from 0 to 4095, because we have 12 bit ADC
				memset(adc_uint32_Buffer, 0, sizeof(adc_uint32_Buffer));
				calculated_CRC_32=0;
				crcInput_value = 0;

				readADCchannels(); // ADC conversation for 4 channels
				for (uint8_t i = 0; i < 4; i++)
					adc_uint32_Buffer[i] = (uint32_t)adcBuffer[i];
				calculated_CRC_32 = HAL_CRC_Calculate(&hcrc, &adc_uint32_Buffer, 4);

				 for(uint8_t i=0; i<4; i++)
					memcpy(txBuffer + (i * sizeof(adcBuffer[0])), (uint8_t*)(&adcBuffer[i]), sizeof(adcBuffer[0]));
				 memcpy(txBuffer + (4*sizeof(adcBuffer[0])), (uint8_t*)(&calculated_CRC_32), sizeof(calculated_CRC_32));

				 transmitData(USART1, (uint8_t*)txBuffer, ((4 * sizeof(adcBuffer[0])) + sizeof(calculated_CRC_32)));
				 clearBufferAndCounter();
				 break;

			 case 'f':
				 calculated_CRC_32 = HAL_CRC_Calculate(&hcrc, &crc_test_value, 1); // 462
				 memcpy(txBuffer, (uint8_t*)(&calculated_CRC_32), sizeof(calculated_CRC_32));
				 transmitData(USART1, (uint8_t*)txBuffer, 4);
				 clearBufferAndCounter();
				 break;

			 default:
				 clearBufferAndCounter();
				 continue;
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_2, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSRC_PLL_DIV_1);
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
