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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "quad_sseg.h"
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
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t unhandled_exti;
uint32_t last_flank_causing_exti;
int b1_pressed;
int pressed;
uint32_t last_managed_exti = 0;
uint32_t last_flank_causing_exti = 0;
uint32_t button_exti_count = 0;
uint32_t button_debounced_count = 0;

uint8_t secondsOnes = 0;
uint8_t secondsTens = 4;
uint8_t minutesOnes = 9;
uint8_t minutesTens = 5;
uint8_t hoursOnes = 3;
uint8_t hoursTens = 2;
uint8_t colonFlicker = 0;
int freq_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uart_print_menu(){
	  char str[100];

	sprintf(str,"Choose an option: \r\n"
			"Enter 1 for Clock mode.\r\n"
			"Enter 2 for Button mode.\r\n");

	  HAL_UART_Transmit(&huart2, (uint8_t *) str, strlen(str), HAL_MAX_DELAY);

}

int uart_get_menu_choice()
 {
	  char str[1] = { '\0'};
	  uint16_t str_len = 1;
	  HAL_UART_Receive(&huart2, (uint8_t *) str, str_len, HAL_MAX_DELAY);
	  int ret = -1;
	  sscanf(str, "%d", &ret);
	  return ret;
 }

void uart_print_bad_choice() {
		char str[50];
		sprintf(str, "Invalid input. Try again. Choose 1 or 2.\r\n\n");
		HAL_UART_Transmit(&huart2, (uint8_t *) str, strlen(str), HAL_MAX_DELAY);
}

void clock_mode()
{
/*** init segment ***/
	HAL_TIM_Base_Start_IT(&htim9);

/*** main loop ***/
	while (1)
	{
		b1_pressed = GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

		if (b1_pressed) {
			qs_put_digits(hoursTens, hoursOnes, minutesTens, minutesOnes, colonFlicker);
		}
		else {
			qs_put_digits(minutesTens, minutesOnes, secondsTens, secondsOnes, colonFlicker);
		}

	}
}

void button_mode()
{
/*** init segment ***/

/*** main loop ***/
	while (1)
	{
		/* deal with debouncing the button... */
		/*if (1) // For initial testing
		{
		button_debounced_count = button_exti_count;
		}*/

		//-------------------
		uint32_t current_tick = HAL_GetTick();

		b1_pressed = GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0); // check b1 button (on board, active low)
		qs_put_big_num(b1_pressed ? button_exti_count : button_debounced_count);


		if (unhandled_exti && (last_flank_causing_exti != last_managed_exti)) // was set in interrupt
		{
			if(current_tick - last_flank_causing_exti >= 50) {


				pressed = GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);

				if (pressed) {
					button_debounced_count++;
				}

				last_managed_exti = last_flank_causing_exti;
			}

		unhandled_exti = 0;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == GPIO_PIN_0) {

	last_flank_causing_exti = HAL_GetTick();
	unhandled_exti = 1; //sätter flagga till 1 för att flagga unhandled_exit
	button_exti_count++;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	freq_counter++;
	colonFlicker = 1;

	if (freq_counter >= 2){
		freq_counter = 0;
		colonFlicker = 0;
		secondsOnes++;
	}
	if (secondsOnes > 9){
		secondsOnes = 0;
		secondsTens++;
	}
	if (secondsTens > 5){
		secondsTens = 0;
		minutesOnes++;
	}
	if (minutesOnes > 9){
		minutesOnes = 0;
		minutesTens++;
	}
	if (minutesTens > 5){
		minutesTens = 0;
		hoursOnes++;
	}
	if(hoursOnes > 9){
		hoursOnes = 0;
		hoursTens++;
	}
	if((hoursOnes == 4) & (hoursTens == 2)){
		hoursOnes = 0;
		hoursTens = 0;
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
  MX_USART2_UART_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 uart_print_menu();
	 int menu_choice = uart_get_menu_choice();

	  {
		  switch (menu_choice)
		  {
		  case 1:
			  clock_mode();
			  break;

		  case 2:
			  button_mode();
			  break;

		  default:
			  uart_print_bad_choice();
			  break;
		  }
	  }


	  /*//Display testing code------
	  for (int i = 0; i < 10; i++)
	  {
	  uint32_t dly = 250;
	  qs_put_big_num(i);
	  HAL_Delay(dly);
	  qs_put_digits(i, i, i, i, 0);
	  HAL_Delay(dly);
	  qs_put_digits(i, i, i, i, 1);
	  HAL_Delay(dly);
	  }
	  //--------------------------*/


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 1000;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 41999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_CLK_Pin|SEG_DIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 Button_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SEG_CLK_Pin */
  GPIO_InitStruct.Pin = SEG_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEG_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SEG_DIO_Pin */
  GPIO_InitStruct.Pin = SEG_DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEG_DIO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
