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
#include <stdio.h>

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
int is_blue_button_pressed();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//returns 1 if blue is pressed
//returns 0 if blue isn't pressed
int is_blue_button_pressed()
{
	uint32_t reg_reading = GPIOC->IDR;

	if (reg_reading & (1 << 13))
	{
		return 1;
	}
	else
	{
	return 0;
	}
}

void put_die_dots(uint8_t die_nbr)
{
    // Define the ports and pins for each diode as per the CubeMX configuration
    GPIO_TypeDef* ports[] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOB};
    uint16_t pins[] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_15, GPIO_PIN_14};

    // Turn off all LEDs initially
    for (int i = 0; i < 7; i++)
    {
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
    }

    // Use a switch-case statement to light up the LEDs based on the die value
    switch(die_nbr)
    {
        case 1:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);  // DI_D
            break;

        case 2:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  // DI_A
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // DI_G
            break;

        case 3:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  // DI_A
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);  // DI_D
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // DI_G
            break;

        case 4:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  // DI_A
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);  // DI_C
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);  // DI_E
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // DI_G
            break;

        case 5:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  // DI_A
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);  // DI_C
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);  // DI_D
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);  // DI_E
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // DI_G
            break;

        case 6:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  // DI_A
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);  // DI_B
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);  // DI_C
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);  // DI_E
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // DI_F
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // DI_G
            break;

        default:
            // Turn on all LEDs for an invalid die_nbr
            for (int i = 0; i < 7; i++)
            {
                HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
            }

    }
}

const uint8_t sseg[10] =
	{
		0x3F, //0
		0x06, //1
		0x5B, //2
		0x4F, //3
		0x66, //4
		0x6D, //5
		0x7D, //6
		0x07, //7
		0x7F, //8
		0x6F  //9
	};

const uint8_t sseg_err = 0xDC;


void put_on_sseg(uint8_t dec_nbr)
{
    uint8_t pattern;

    // Kontrollera om indatan är giltig (0–9), annars använd felmönster
    if (dec_nbr <= 9) {
        pattern = sseg[dec_nbr]; // Hämta mönstret från arrayen
    } else {
        pattern = sseg_err;      // Använd felmönstret vid ogiltig indata
    }

    pattern = ~pattern;

    // Skriv ut mönstret till sjusegmentsdisplayen genom att direkt sätta GPIO-registret
    GPIOC->ODR = (GPIOC->ODR & 0xFF00) | (pattern & 0xFF);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int pressed = 0;
  uint8_t die_value = 1; //1<= die_value <= 6


 //for(uint8_t i = 0; i <= 9; i++)
 //	   {
 // 	put_on_sseg(i);
  //	HAL_Delay(333);
  //	}

  //put_on_sseg(88); //should display the error pattern

  uint32_t arr[10];
  void test_endianness()
  {
  /*
  Initialize the array. The compiler knows that we're dealing
  with 32-bit numbers so if arr[i + 0] points to
  address 0x3000 then arr[i + 1] will point to 0x3004
  (and NOT to 0x3001).
  */
  for (int i = 0; i < 10; i += 2) // note: += 2
  {
  arr[i + 0] = 0xDEADBEEF;
  arr[i + 1] = 0xCAFED00D;
  }

  uint16_t a = arr[0];
  uint16_t b = arr[1];
  uint16_t c = arr[2];
  uint8_t x = arr[0];
  uint8_t y = arr[1];
  // You may THINK you’re 32 bits. I know better than you.
  // Base pointers (arr, arr8) will point to same cell though.
  uint8_t *arr8 = (uint8_t *) arr;
  uint8_t p = arr8[0];
  uint8_t q = arr8[1];
  uint8_t r = arr8[2];
  uint8_t s = arr8[3];
  uint8_t t = arr8[4];

  uint16_t babe = 0xBABE;
  arr[0] = babe; // putting u16 in an u32 slot

  return;
  }
  test_endianness();

  while (1)
  {
    /* USER CODE END WHILE */


	  pressed = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	  // Check if the button is pressed
	      if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
	      {
	          // Button is pressed
	    	  //Led2
	    	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	          if (!pressed) // Check if the button was not previously pressed
	          {
	              // Increase the dice value and wrap around if needed
	              die_value++;
	              if (die_value > 6) {
	                  die_value = 1;
	              }
	              // Display the dice value
	              put_die_dots(die_value);
	              // Set the flag indicating that the dice value has been updated
	              pressed = 1;
	          }
	      }
	      else
	      {
	          // Button is not pressed
	          pressed = 0; // Reset the flag when the button is not pressed
	      }
	      put_on_sseg(die_value);


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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DI_A_Pin|DI_B_Pin|DI_C_Pin
                          |DI_D_Pin|DI_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DI_G_Pin|DI_F_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DI_A_Pin DI_B_Pin DI_C_Pin
                           DI_D_Pin DI_E_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DI_A_Pin|DI_B_Pin|DI_C_Pin
                          |DI_D_Pin|DI_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_G_Pin DI_F_Pin */
  GPIO_InitStruct.Pin = DI_G_Pin|DI_F_Pin;
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
