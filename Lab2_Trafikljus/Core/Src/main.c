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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum event
{
	ev_none = 0,
	ev_button_push,
	ev_state_timeout
};


enum state
{
	s_init,
	s_car_go,
	s_pushed_wait,
	s_cars_stopping,
	s_walk_go,
	s_walk_wait,
	s_car_ready,
	s_car_start
};

void set_traffic_lights(enum state s)
{
    switch(s)
    {
        case s_init:
            // Sätt alla lampor på
            GPIOB->ODR |= (1 << 15); // Tänd CarRed
            GPIOB->ODR |= (1 << 14); // Tänd CarYellow
            GPIOB->ODR |= (1 << 13); // Tänd CarGreen
            GPIOA->ODR |= (1 << 12); // Tänd WalkRed
            GPIOA->ODR |= (1 << 11); // Tänd WalkGreen
            break;

        case s_car_go:

            GPIOB->ODR &= ~(1 << 15); // Släck CarRed
            GPIOB->ODR &= ~(1 << 14); // Släck CarYellow
            GPIOB->ODR |= (1 << 13);  // Tänd CarGreen
            GPIOA->ODR |= (1 << 12);  // Tänd WalkRed
            GPIOA->ODR &= ~(1 << 11); // Släck WalkGreen
            break;

        case s_pushed_wait:

            GPIOB->ODR &= ~(1 << 15); // Släck CarRed
            GPIOB->ODR |= (1 << 14);  // Tänd CarYellow
            GPIOB->ODR &= ~(1 << 13); // Släck CarGreen
            GPIOA->ODR |= (1 << 12);  // Tänd WalkRed
            GPIOA->ODR &= ~(1 << 11); // Släck WalkGreen
            break;

        case s_cars_stopping:

            GPIOB->ODR |= (1 << 15);  // Tänd CarRed
            GPIOB->ODR &= ~(1 << 14); // Släck CarYellow
            GPIOB->ODR &= ~(1 << 13); // Släck CarGreen
            GPIOA->ODR |= (1 << 12);  // Tänd WalkRed
            GPIOA->ODR &= ~(1 << 11); // Släck WalkGreen
            break;

        case s_walk_go:

            GPIOB->ODR &= ~(1 << 15); // Släck CarRed
            GPIOB->ODR &= ~(1 << 14); // Släck CarYellow
            GPIOB->ODR &= ~(1 << 13); // Släck CarGreen
            GPIOA->ODR &= ~(1 << 12); // Släck WalkRed
            GPIOA->ODR |= (1 << 11);  // Tänd WalkGreen
            break;

        case s_walk_wait:

            GPIOA->ODR |= (1 << 12);  // Tänd WalkRed
            GPIOA->ODR &= ~(1 << 11); // Släck WalkGreen

            GPIOB->ODR &= ~(1 << 15); // Släck CarRed
			GPIOB->ODR &= ~(1 << 14); // Släck CarYellow
			GPIOB->ODR &= ~(1 << 13); // Släck CarGreen
            break;

        case s_car_ready:

            GPIOB->ODR |= (1 << 15); // Tänd CarRed
            GPIOB->ODR |= (1 << 14);  // Tänd CarYellow
            GPIOB->ODR &= ~(1 << 13); // Släck CarGreen
            GPIOA->ODR |= (1 << 12);  // Tänd WalkRed
            GPIOA->ODR &= ~(1 << 11); // Släck WalkGreen
            break;

        case s_car_start:

            GPIOB->ODR &= ~(1 << 15); // Släck CarRed
            GPIOB->ODR &= ~(1 << 14); // Släck CarYellow
            GPIOB->ODR |= (1 << 13);  // Tänd CarGreen
            GPIOA->ODR |= (1 << 12);  // Tänd WalkRed
            GPIOA->ODR &= ~(1 << 11); // Släck WalkGreen
            break;

        default:

            break;
    }
}

int is_button_pressed()
{
	static int last_press = 0;  // Variabel som håller reda på tidigare knappstatus
	    int curr_press;

	    // Läs knappens aktuella tillstånd
	    uint32_t reg_reading = GPIOC->IDR;
	    curr_press = (reg_reading & (1 << 13)) ? 1 : 0;  // Läs bit 13 (knappen)

	    // Kontrollera om det har varit en positiv flank
	    if (curr_press == 1 && last_press == 0)
	    {
	        last_press = curr_press;  // Uppdatera tidigare knappstatus
	        return 1;  // Positiv flank (knapptryckning upptäckt)
	    }

	    // Uppdatera tidigare knappstatus
	    last_press = curr_press;

	    return 0;  // Ingen positiv flank

}

void update_tick(uint32_t* last_tick, int* ticks_left_in_state, int* timeout_handled)
{
    uint32_t curr_tick = HAL_GetTick(); // Hämta nuvarande tick-värde

    // Kontrollera om ett nytt tick har gått (1 millisekund)
    if (curr_tick > *last_tick)
        {
            *last_tick = curr_tick;  // Uppdatera senast lästa tick

            // Minska ticks_left_in_state om det är större än 0
            if (*ticks_left_in_state > 0)
            {
                *ticks_left_in_state--;

                // Kontrollera om tiden har nått 0 och om det inte redan är hanterat
                if (*ticks_left_in_state == 0 && !*timeout_handled)
                {
                    *timeout_handled = 1;  // Sätt flagga för att undvika dubbelhantering
                }
            }
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


  uint32_t last_tick = 0;            // Håller koll på senaste tick från HAL_GetTick()
  int ticks_left_in_state = 0;       // Hur många ticks som är kvar innan timeout
  int timeout_handled = 0;           // Flagga för att säkerställa att ev_state_timeout bara genereras en gång

  /* USER CODE BEGIN 2 */
  enum state st = s_init;
  enum state next_st = st;  // Variabel för nästa tillstånd
  enum event ev = ev_none;

  int curr_press = is_button_pressed();
  int last_press = curr_press;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  ev = ev_none;

	  update_tick(&last_tick, &ticks_left_in_state, &timeout_handled);

   // Kolla efter knapptryckning (positiv flank)
	  if (is_button_pressed() && st == s_car_go)
	  {
		ev = ev_button_push;
	  }
	  else if (timeout_handled)
	          {
	              ev = ev_state_timeout;
	              timeout_handled = 0; // Återställ timeout_handled efter att ha hanterat eventet
	          }
	  else
	  {
		  ev = ev_none;
	  }


	  switch(st)
	      {
	          case s_init:
	              // Alla lampor tända (111 11)
	              set_traffic_lights(s_init);  // Tänder alla lampor

	              if (ev == ev_button_push) {
	            	  next_st = s_walk_go;
	                  set_traffic_lights(st);
	                  ticks_left_in_state = 2000;
	                  timeout_handled = 0;

	              }
	              break;

	          case s_car_go:
	              if (ev == ev_button_push) {
	            	  next_st = s_pushed_wait;
	                  set_traffic_lights(st);
	                  //ticks_left_in_state = 10000;
					  timeout_handled = 0;

	              }
	              break;

	          case s_pushed_wait:
	              if (ev == ev_state_timeout) {
	            	  next_st = s_cars_stopping;
	                  set_traffic_lights(st);
	                  ticks_left_in_state = 2000;
	                  timeout_handled = 0;

	              }
	              break;

	          case s_cars_stopping:
	              if (ev == ev_state_timeout) {
	            	  next_st = s_walk_go;
	                  set_traffic_lights(st);
	                  ticks_left_in_state = 1000;
					  timeout_handled = 0;

	              }
	              break;

	          case s_walk_go:
	              if (ev == ev_state_timeout) {
	            	  next_st = s_walk_wait;
	                  set_traffic_lights(st);
	                  ticks_left_in_state = 3000;
	                  timeout_handled = 0;

	              }
	              break;

	          case s_walk_wait:
	              if (ev == ev_state_timeout) {
	            	  next_st = s_car_ready;
	                  set_traffic_lights(st);
	                  ticks_left_in_state = 1000;
					  timeout_handled = 0;

	              }
	              break;

	          case s_car_ready:
	              if (ev == ev_state_timeout) {
	            	  next_st = s_car_start;
	                  set_traffic_lights(st);
	                  ticks_left_in_state = 1000;
					  timeout_handled = 0;

	              }
	              break;

	          case s_car_start:
	              if (ev == ev_state_timeout) {
	            	  next_st = s_car_go;
	                  set_traffic_lights(st);
	                  ticks_left_in_state = 1000;
					  timeout_handled = 0;

	              }
	              break;

	          default:
	              // Fallback tillstånd
	              break;
	      }

	  // Om tillståndet har ändrats, uppdatera och sätt ljusen
		  if (next_st != st) {
			  st = next_st;
			  set_traffic_lights(st);
	          }
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|WalkGreen_Pin|WalkRed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CarGreen_Pin|CarYellow_Pin|CarRed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin WalkGreen_Pin WalkRed_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|WalkGreen_Pin|WalkRed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CarGreen_Pin CarYellow_Pin CarRed_Pin */
  GPIO_InitStruct.Pin = CarGreen_Pin|CarYellow_Pin|CarRed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
