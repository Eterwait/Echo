/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bunraky_lib.h"
#include "globals.h"
#include <stdint.h>
#include "usbd_cdc_if.h"
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
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  	uint32_t crc_test_value = 462;



	// Start Tim 2 to work, it will produce right motor PWM
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM2);

	// Set En and Dir pins for right motor
	LL_GPIO_SetOutputPin(En_right_motor_GPIO_Port, En_right_motor_Pin);
	LL_GPIO_ResetOutputPin(Dir_right_motor_GPIO_Port, Dir_right_motor_Pin);

	// Set PWM for right motor to 0 (equal ZERO current)
	LL_TIM_OC_SetCompareCH1(TIM2,0);



	// Allow Tim 3 to work, it will produce left motor PWM
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM3);

	// Set En and Dir pins for left motor
	LL_GPIO_SetOutputPin(En_left_motor_GPIO_Port, En_left_motor_Pin);
	LL_GPIO_ResetOutputPin(Dir_left_motor_GPIO_Port, Dir_left_motor_Pin);

	// Set PWM for left motor to 0 (equal ZERO current)
	LL_TIM_OC_SetCompareCH1(TIM3,0);



	// Allow USART 6 to work
	NVIC_EnableIRQ(USART6_IRQn);

	//  LL_USART_Enable(USART6);
	USART6->CR1 |= USART_CR1_TE | USART_CR1_RE;
	LL_USART_EnableIT_RXNE(USART6);
	LL_USART_EnableIT_ERROR(USART6);

	LL_GPIO_ResetOutputPin(RE_DE_GPIO_Port, RE_DE_Pin); // Set MAX485 to Rx mode

//    LL_TIM_EnableIT_UPDATE(TIM2); // Start timer 2 for interruption each 35 ms for buzzers
//    LL_TIM_EnableCounter(TIM2);   // Start timer 2

	// Load calibration values from flash
	uint8_t init_load_calibration_error = init_load_calibration_values(); // if 1 == some problems
	uint32_t error = 0;
	uint32_t error_zero_dof_6 = 0;
	uint32_t error_zero_dof_7 = 0;
	uint32_t error_zero_dof_8 = 0;
	uint32_t stop_motor_counter = 0;

	//	uint8_t count_write_flash = sizeof(my_array) / sizeof(my_array[0]);
	//	uint32_t error = flash_write_words(FLASH_SECTOR_6, ADDR_FLASH_SECTOR_6, CCW_limits_uint32_flash, count_write_flash);
	//	uint32_t error = flash_write_words(FLASH_SECTOR_6, ADDR_FLASH_SECTOR_6, CW_limits_uint32_flash, count_write_flash);
	LL_GPIO_SetOutputPin(Sense_Lvl_1_GPIO_Port, Sense_Lvl_1_Pin);
	start_melody();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  LL_mDelay(1);

//	  readADCchannels();
//	  is_min_max_calibrated();
//	  calibration_ADC_values();

//	  USB_Rx_buffer[0]='r'; // Command to calibration
//	  LL_mDelay(2000);
//	  USB_Rx_buffer[0]='c'; // Command to calibration
//	  USB_Rx_buffer[1]='l'; // Left arm choice
//	  USB_Rx_buffer[2]='6'; // DoF mode
//	  USB_Rx_buffer[3]='1'; // DoF to calibrate
//	  USB_Rx_buffer[4]='0'; // Min Max choice
	  switch(USB_Rx_buffer[0])
		  {
	  	  	 case 'c': // command to calibration
	  	  		 switch(USB_Rx_buffer[1])
	  	  		 {
	  	  		 	 case 'l': // left arm choice
	  	  		 		 switch(USB_Rx_buffer[2])
	  	  		 		 {
							 case'6': //  6 DoF mode (without gripper) choice
								 switch(USB_Rx_buffer[3])
								 {
									 case '1':  //  DoF to calibrate choice
										 // The first parameter is the CCW or CW command, the second parameter is the DoF number
										 process_calibration(USB_Rx_buffer[4], 1);
									 break;

									 case '2':
										 process_calibration(USB_Rx_buffer[4], 2);
									 break;

									 case '3':
										 process_calibration(USB_Rx_buffer[4], 3);
									 break;

									 case '4':
										 process_calibration(USB_Rx_buffer[4], 4);
									 break;

									 case '5':
										 process_calibration(USB_Rx_buffer[4], 5);
									 break;

									 case '6':
										 error_zero_dof_8 = wrote_zero_to_dof(7);
										 error_zero_dof_8 = wrote_zero_to_dof(8);
										 process_calibration(USB_Rx_buffer[4], 6);

									 break;
								 }
								 break;

	  	  		 		 	 case'7': //  7 DoF mode choice
	  		  	  		 		 switch(USB_Rx_buffer[3])
	  		  	  		 		 {
	  		  	  		 		 	 case '1':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 1);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '2':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 2);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '3':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 3);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '4':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 4);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '5':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 5);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '6':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 6);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '8':
	  		  	  		 		 		 error_zero_dof_7 = wrote_zero_to_dof(7);
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 8);
	  		  	  		 		 	 break;
	  		  	  		 		 }
	  	  		 		 		 break;

	  	  		 		 	 case'8': //  8 DoF mode choice
	  		  	  		 		 switch(USB_Rx_buffer[3])
	  		  	  		 		 {
	  		  	  		 		 	 case '1':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 1);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '2':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 2);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '3':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 3);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '4':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 4);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '5':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 5);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '6':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 6);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '7':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 7);
	  		  	  		 		 	 break;

	  		  	  		 		 	 case '8':
	  		  	  		 		 		 process_calibration(USB_Rx_buffer[4], 8);
	  		  	  		 		 	 break;
	  		  	  		 		 }
	  	  		 		 		 break;
	  	  		 		 }
	  	  		 		 break;


	  	  		 	 case 'r': // Right arm choice  9-16 == 1-8 DoF
	  	  		 		 switch(USB_Rx_buffer[2])
	  	  		 		 {
							 case'6': //  6 DoF mode (without gripper) choice
								 switch(USB_Rx_buffer[3])
								 {
									 case '1':
										 process_calibration(USB_Rx_buffer[4], 9);
									 break;

									 case '2':
										 process_calibration(USB_Rx_buffer[4], 10);
									 break;

									 case '3':
										 process_calibration(USB_Rx_buffer[4], 11);
									 break;

									 case '4':
										 process_calibration(USB_Rx_buffer[4], 12);
									 break;

									 case '5':
										 process_calibration(USB_Rx_buffer[4], 13);
									 break;

									 case '6':
										 error_zero_dof_7 = wrote_zero_to_dof(15);
										 error_zero_dof_7 = wrote_zero_to_dof(16);
										 process_calibration(USB_Rx_buffer[4], 14);
									 break;
								 }
								 break;

	  	  		 		 	 case'7': //  7 DoF mode choice
	  		  	  		 		 switch(USB_Rx_buffer[3])
	  		  	  		 		 {
	  								 case '1':
	  									process_calibration(USB_Rx_buffer[4], 9);
	  								 break;

	  								 case '2':
	  									process_calibration(USB_Rx_buffer[4], 10);
	  								 break;

	  								 case '3':
	  									process_calibration(USB_Rx_buffer[4], 11);
	  								 break;

	  								 case '4':
	  									process_calibration(USB_Rx_buffer[4], 12);
	  								 break;

	  								 case '5':
	  									process_calibration(USB_Rx_buffer[4], 13);
	  								 break;

	  								 case '6':
	  									process_calibration(USB_Rx_buffer[4], 14);
	  								 break;

	  								 case '8':
	  									error_zero_dof_7 = wrote_zero_to_dof(15);
	  									process_calibration(USB_Rx_buffer[4], 16);
	  								 break;
	  		  	  		 		 }
	  	  		 		 		 break;

	  	  		 		 	 case'8': //  8 DoF mode choice
	  		  	  		 		 switch(USB_Rx_buffer[3])
	  		  	  		 		 {
	  								 case '1':
	  									process_calibration(USB_Rx_buffer[4], 9);
	  								 break;

	  								 case '2':
	  									process_calibration(USB_Rx_buffer[4], 10);
	  								 break;

	  								 case '3':
	  									process_calibration(USB_Rx_buffer[4], 11);
	  								 break;

	  								 case '4':
	  									process_calibration(USB_Rx_buffer[4], 12);
	  								 break;

	  								 case '5':
	  									process_calibration(USB_Rx_buffer[4], 13);
	  								 break;

	  								 case '6':
	  									process_calibration(USB_Rx_buffer[4], 14);
	  								 break;

	  								 case '7':
	  									process_calibration(USB_Rx_buffer[4], 15);
	  								 break;

	  								 case '8':
	  									process_calibration(USB_Rx_buffer[4], 16);
	  								 break;
	  		  	  		 		 }
	  	  		 		 		 break;
	  	  		 		 }
	  	  		 		 break;
	  	  		 }
				 clearBufferAndCounter();
				 clearUsbBuffers();
				 break;

			 case 'r':
	  		 		 switch(USB_Rx_buffer[1])
	  		 		 {
						 case '1':
							 // Function with communication with force sensor pcb
							 if (readForseSensor() == 1)
							 {
								 stop_motor_counter=0;
								 set_new_force();
								 readADCchannels();
								 is_min_max_calibrated();
								 calibration_ADC_values();
								 memcpy(USB_Tx_buffer + 0, (uint8_t*)(&adcBuffer), sizeof(adcBuffer));
								 memcpy(USB_Tx_buffer + sizeof(adcBuffer), (uint8_t*)(&force_sensor_4_value), sizeof(force_sensor_4_value));
								 memcpy(USB_Tx_buffer + sizeof(adcBuffer) + sizeof(force_sensor_4_value), (uint8_t*)(&start_dataset_collection_flag), sizeof(start_dataset_collection_flag));
								 memcpy(USB_Tx_buffer + sizeof(adcBuffer) + sizeof(force_sensor_4_value) + sizeof(start_dataset_collection_flag), (uint8_t*)(&sense_flag), sizeof(sense_flag));
								 while (CDC_Transmit_FS(USB_Tx_buffer, sizeof(adcBuffer) + sizeof(force_sensor_4_value) + sizeof(start_dataset_collection_flag) + sizeof(sense_flag)) == USBD_BUSY)
									 ; // Wait for end of transmission
							 }
							 else
							 {
								 LL_mDelay(1);
								 if (readForseSensor() == 1)
								 {
									 stop_motor_counter=0;
									 set_new_force();
									 readADCchannels();
									 is_min_max_calibrated();
									 calibration_ADC_values();
									 memcpy(USB_Tx_buffer + 0, (uint8_t*)(&adcBuffer), sizeof(adcBuffer));
									 memcpy(USB_Tx_buffer + sizeof(adcBuffer), (uint8_t*)(&force_sensor_4_value), sizeof(force_sensor_4_value));
									 memcpy(USB_Tx_buffer + sizeof(adcBuffer) + sizeof(force_sensor_4_value), (uint8_t*)(&start_dataset_collection_flag), sizeof(start_dataset_collection_flag));
									 memcpy(USB_Tx_buffer + sizeof(adcBuffer) + sizeof(force_sensor_4_value) + sizeof(start_dataset_collection_flag), (uint8_t*)(&sense_flag), sizeof(sense_flag));
									 while (CDC_Transmit_FS(USB_Tx_buffer, sizeof(adcBuffer) + sizeof(force_sensor_4_value) + sizeof(start_dataset_collection_flag) + sizeof(sense_flag)) == USBD_BUSY)
										 ; // Wait for end of transmission
								 }
							 }
							 break;

						 case '2':
							 readADCchannels();
							 is_min_max_calibrated();
							 calibration_ADC_values();
							 memcpy(USB_Tx_buffer + 0, (uint8_t*)(&adcBuffer), sizeof(adcBuffer));
							 memcpy(USB_Tx_buffer + sizeof(adcBuffer), (uint8_t*)(&start_dataset_collection_flag), sizeof(start_dataset_collection_flag));
							 memcpy(USB_Tx_buffer + sizeof(adcBuffer) + sizeof(start_dataset_collection_flag), (uint8_t*)(&sense_flag), sizeof(sense_flag));
							 while (CDC_Transmit_FS(USB_Tx_buffer, sizeof(adcBuffer) + sizeof(start_dataset_collection_flag) + sizeof(sense_flag)) == USBD_BUSY)
								 ; // Wait for end of transmission
							 break;
	  		 		 }

				 clearBufferAndCounter();
				 clearUsbBuffers();
				 break;

			 case 'e':
				 erase_flash(FLASH_SECTOR_6);
				 erase_flash(FLASH_SECTOR_7);
				 clearBufferAndCounter();
				 clearUsbBuffers();
				 break;

			 case 'x':
				 load_calibration_values();
				 memcpy(USB_Tx_buffer + 0, (uint8_t*)(&CCW_limits), sizeof(CCW_limits));
				 memcpy(USB_Tx_buffer + sizeof(CW_limits), (uint8_t*)(&CW_limits), sizeof(CW_limits));
				 while (CDC_Transmit_FS(USB_Tx_buffer, sizeof(CCW_limits) + sizeof(CW_limits)) == USBD_BUSY)
					 ; // Wait for end of transmission
				 clearBufferAndCounter();
				 clearUsbBuffers();
				 break;

			 case 's':
//				 stop_motor();
				 memset(USB_Rx_buffer, 0, sizeof(USB_Rx_buffer)); // Clear USB Rx buffer
				 break;

			 default:
				 stop_motor_counter ++;
				 if (stop_motor_counter>=2000)
				 {
					 stop_motor_counter=3000;
					 stop_motor();
				 }
				 information_leds_activation();
				 clearUsbBuffers();
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
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLQ_DIV_7);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(84000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
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
