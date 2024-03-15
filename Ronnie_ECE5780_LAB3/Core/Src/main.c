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
	
		
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	//SYSCFG Clock------------------
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	
	//Enabling TIM2/TIM3 
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	//Setting prescalar/ Auto Reload Value for Tim 2 & Their Interrupts
	TIM2->PSC = 799; // Prescaler value is PSC + 1; 8*10^6 / 800 = 1*10^4 = 10 KHz
	TIM2->ARR = 2500;
	//TIM2->DIER |= TIM_DIER_TDE;
	//TIM2->DIER |= TIM_DIER_TIE; //Clear TIM_SR_TIF to reset interrupt flag;
	TIM2->EGR |= TIM_EGR_TG;
	TIM2->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(SysTick_IRQn,2)
	
  //Setting prescalar/ Auto Reload Value for Tim 3
	TIM3->PSC = 99; // f_count = 8 *10^6 / (PSC+1) == 8,000,000 / 100 = 80,000
	TIM3->ARR = 100; // 80,000hz / 100 = 800hz
	
	//Setting TIM 3 to enable 2 compare channels
	//TIM3->CCMR1 //Configuring CaptureCompare Configuration for Timer3
	//OC1M - PWM mode 2 (111) for Tim 3 Channel 1
	TIM3 -> CCMR1 |= TIM_CCMR1_OC1M_2;
	TIM3 -> CCMR1 |= TIM_CCMR1_OC1M_1;
	TIM3 -> CCMR1 |= TIM_CCMR1_OC1M_0;
	//OC2M - PWM mode (110) 1 for Tim 3 Channel 2
	TIM3 -> CCMR1 |= TIM_CCMR1_OC2M_2;
	TIM3 -> CCMR1 |= TIM_CCMR1_OC2M_1;
	
	TIM3 -> CCMR1 |= TIM_CCMR1_OC1PE;
	TIM3 -> CCMR1 |= TIM_CCMR1_OC2PE;

	
	//TIM3 -> CCRX - At what TIM3 value should PWM output change
	TIM3 -> CCR1 = 20; //0.2 * 100 = 20
	TIM3 -> CCR2 = 20;
	
	//TIM3 -> CCR1 = 10; //Mode 2
	//TIM3 -> CCR2 = 90;// Mode 1
	
	//TIM3 -> CCR1 = 90; //0.2 * 100 = 20
	//TIM3 -> CCR2 = 10;
	
	//TIM3 -> CCER // Configures Channels of Capture mode
	//CC1E - Enable Channel 1 Output
	TIM3 -> CCER |= TIM_CCER_CC1E;
	//CC2E - Enable Channel 2 Output
	TIM3 -> CCER |= TIM_CCER_CC2E;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

	//Configuring PC6/PC7 to Alt Mode using TIM3_CH1/CH2 outputs respectively
	GPIOC -> MODER |= GPIO_MODER_MODER6_1;//setting PC6/PC7 to Alternate MODER
	GPIOC -> MODER |= GPIO_MODER_MODER7_1;
	//THE PC6/PC7 pins alternate functions default to using AF0 ie. what we want
	//If an assignment would be made it would use:
	//GPIOC -> AFR |= GPIO_AFRL_AFSEL0; <-- some not messed up bitwise operation to set 4 bits
	
	GPIOC -> MODER |= GPIO_MODER_MODER8_0;//Setting PC8/PC9 to Generic Output MODER
	GPIOC -> MODER |= GPIO_MODER_MODER9_0;



	GPIOC -> ODR |= GPIO_ODR_9;
	
	// Start TIM2
  TIM2->CR1 |= TIM_CR1_CEN;
	// Start TIM3
  TIM3->CR1 |= TIM_CR1_CEN;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_Delay(500);
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