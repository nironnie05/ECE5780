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
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	//PA1 setup for ADC1 & ADC1 setup
	GPIOA->MODER |= GPIO_MODER_MODER1_0;
	GPIOA->MODER |= GPIO_MODER_MODER1_1;
	//PA4 setup for DAC
	GPIOA->MODER |= GPIO_MODER_MODER4_0;
	GPIOA->MODER |= GPIO_MODER_MODER4_1;
	
	//LED SETUP
	GPIOC->MODER |= (1<<12); // PC6 RED
	GPIOC->MODER |= (1<<14); // PC7 BLUE
  GPIOC->MODER |= (1<<16); // PC8 ORANGE
	GPIOC->MODER |= (1<<18); // PC9 GREEN
	// Set pins to push-pull output type in OTYPER register
	GPIOC->OTYPER &= ~(1<<6); // PC6
	GPIOC->OTYPER &= ~(1<<7); // PC7
	GPIOC->OTYPER &= ~(1<<8); // PC8
	GPIOC->OTYPER &= ~(1<<9); // PC9
	//lowspeed
	GPIOC->OSPEEDR &= ~(1<<12); // PC6
	GPIOC->OSPEEDR &= ~(1<<14); // PC7
	GPIOC->OSPEEDR &= ~(1<<16); // PC8
	GPIOC->OSPEEDR &= ~(1<<18); // PC9
	//no pupd
	GPIOC->PUPDR &= ~((1<<12) | (1<<13));//PC6
	GPIOC->PUPDR &= ~((1<<14) | (1<<15));//PC7
	GPIOC->PUPDR &= ~((1<<16) | (1<<17));//PC8 
	GPIOC->PUPDR &= ~((1<<18) | (1<<19));//PC9
	
	
	ADC1->CR |= ADC_CR_ADEN;
	
	
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	//ADC INPUT SETUP ---------------------------------------------------------
	//setup adc1 for 8 bit mode
	ADC1->CFGR1 &= ~ADC_CFGR1_RES;//clear all bits of the resolution bits
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;//setup adc1 to have a 8 bit resolution
	ADC1->CHSELR |= ADC_CHSELR_CHSEL1;//select PA1 as input to adc1
	ADC1->CFGR1 |= ADC_CFGR1_CONT;//continuous sampling of input
	//ADC Calibration and startup (all registers modified below reset to 0x0)
	if((ADC1->CR & ADC_CR_ADEN) != 0)//make sure required bits are low
	{
		ADC1->CR |= ADC_CR_ADDIS;//not low, fix this
	}
	while(!(ADC1->CR & ADC_CR_ADEN)){}//wait
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
	ADC1->CR |= ADC_CR_ADCAL;//begin calibration
	while((ADC1->CR & ADC_CR_ADCAL) != 0){}//wait for adcal = 0
	if((ADC1->ISR & ADC_ISR_ADRDY) != 0){
		ADC1->ISR |= ADC_ISR_ADRDY;
	}
		//everything ready, enable adc1
	ADC1->CR |= ADC_CR_ADEN;
		//start adc
	ADC1->CR |= ADC_CR_ADSTART;
	//END ADC1 SETUP -----------------------------------------------------------
	//BEGIN DAC SETUP ---------------------------------------------------------
	//DAC->CR |= DAC_CR_TSEL1_0;
	//DAC->CR |= DAC_CR_TSEL1_1;
	//DAC->CR |= DAC_CR_TSEL1_2;
	
	//DAC->CR |= DAC_CR_EN1;
	//END DAC SETUP -----------------------------------------------------------
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	//table for sine wave values for 8-bit values with 32 samples per 1 cycle
	const uint8_t sin_table[32] = {127,151,175,197,216,232,244,251,254,251,244,232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	
	uint8_t sin_index;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int16_t pot_value = 0;
  while (1)
  {
		//PART 1 ---------------------
		///*
		while(!(ADC1->ISR & ADC_ISR_EOC)){
		pot_value = ADC1->DR;

		GPIOC->ODR &= ~(GPIO_ODR_7 | GPIO_ODR_6 | GPIO_ODR_8 | GPIO_ODR_9);

			//12bit adc1 range by default (0 --- 2^8) (2^8 = 256)
			//(0.05 * 256 ~= 13) (0.30 * 256 ~= 77)  (0.55 * 256 ~= 141)   (0.8 * 256 = 205)
			if(pot_value > 13){
				GPIOC -> ODR |= GPIO_ODR_6;
			}
			if(pot_value > 77) {
				GPIOC -> ODR |= GPIO_ODR_9;
			}
			if(pot_value > 141) {
				GPIOC -> ODR |= GPIO_ODR_7;
			}
			if(pot_value > 205){
				GPIOC -> ODR |= GPIO_ODR_8;
			}
		}
		//PART 1 END ------------------
		//*/
		//PART 2 ----------------------
		/*
		HAL_Delay(1);
		
		DAC->DHR8R1 = sin_table[sin_index];
		if(sin_index == 31) {
			sin_index = 0;
		}
		else {
			sin_index = sin_index + 1;
		}
		
		
		//PART 2 END ------------------
		*/
		
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
