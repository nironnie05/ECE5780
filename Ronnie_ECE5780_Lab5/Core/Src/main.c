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
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	//I2C2 TIMINGR Setup
	I2C2->TIMINGR |= (I2C_TIMINGR_SCLL_Msk & 0x13);                             //SCLL = 0x13
	I2C2->TIMINGR |= (I2C_TIMINGR_SCLH_Msk & (0xF << I2C_TIMINGR_SCLH_Pos));    //SCLH = 0xF
	I2C2->TIMINGR |= (I2C_TIMINGR_SDADEL_Msk & (0x2 << I2C_TIMINGR_SDADEL_Pos));//SDADEL = 0x2
	I2C2->TIMINGR |= (I2C_TIMINGR_SCLDEL_Msk & (0x4 << I2C_TIMINGR_SCLDEL_Pos));//SCLDEL = 0x4
	
	//I2C2 GPIO setup : (PB11 = I2C2_SDA) | (PB13 = I2C2_SCL)
	GPIOB -> MODER |= GPIO_MODER_MODER11_1;//setting PB11 AF mode
	GPIOB -> MODER |= GPIO_MODER_MODER13_1;//setting PB13 AF mode
	
	GPIOB -> AFR[1] |= 0b0001 << GPIO_AFRL_AFRL2_Pos;//PB11 = AF1 = I2C2_SDA
	GPIOB -> AFR[1] |= 0b0101 << GPIO_AFRL_AFRL5_Pos;//PB13 = AF5 = I2C2_SCL
	
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;//Output type OpenDrain
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13;//Output type OpenDrain
	
	//PC0 PB14 = high w/ pushpull: PC0 = I2C mode select pin, PB14 = Slave Address Select both set high
	GPIOB -> MODER |= GPIO_MODER_MODER14_0;
	GPIOB -> ODR |= GPIO_ODR_14;
	GPIOC -> MODER |= GPIO_MODER_MODER0_0;
	GPIOC -> ODR |= GPIO_ODR_0;
	
	GPIOB -> MODER |= GPIO_MODER_MODER6_0;
	GPIOB -> MODER |= GPIO_MODER_MODER7_0;
	GPIOB -> MODER |= GPIO_MODER_MODER8_0;
	GPIOB -> MODER |= GPIO_MODER_MODER9_0;
	
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	
	//Peripheral Enable I2C2
	I2C2->CR1 |= I2C_CR1_PE; 
	
	
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
		//Relevant Sample Code
		/*
		
		
		
		
		*/


  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
