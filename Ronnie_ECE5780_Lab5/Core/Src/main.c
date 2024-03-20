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
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	//I2C2 TIMINGR Setup
	I2C2->TIMINGR |= (I2C_TIMINGR_SCLL_Msk & 0x13);                             //SCLL = 0x13
	I2C2->TIMINGR |= (I2C_TIMINGR_SCLH_Msk & (0xF << I2C_TIMINGR_SCLH_Pos));    //SCLH = 0xF
	I2C2->TIMINGR |= (I2C_TIMINGR_SDADEL_Msk & (0x2 << I2C_TIMINGR_SDADEL_Pos));//SDADEL = 0x2
	I2C2->TIMINGR |= (I2C_TIMINGR_SCLDEL_Msk & (0x4 << I2C_TIMINGR_SCLDEL_Pos));//SCLDEL = 0x4
	
	//I2C2 GPIO setup : (PB11 = I2C2_SDA) | (PB13 = I2C2_SCL)
	GPIOB -> MODER |= GPIO_MODER_MODER11_1;//setting PB11 AF mode
	GPIOB -> MODER |= GPIO_MODER_MODER13_1;//setting PB13 AF mode
	
	GPIOB -> AFR[1] |= 0b0001 << GPIO_AFRL_AFRL3_Pos;//PB11 = AF1 = I2C2_SDA
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
	
	//Peripheral Enable I2C2
	I2C2->CR1 |= I2C_CR1_PE; 
	
	
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
		//Relevant Sample Code
		/* Note: defaults to 7 bit I2C address mode
		I2C_CR2 -> NBYTES[7:0]: Number of bytes to transmit/receive
		I2C_CR2 -> RD_WRN: Transfer Direction -> 1 = Write transfer request, 0 = Read transfer request
		I2C_CR2 -> START: Start generation bit: set to 1 by software to begin start of data transfer
		I2C_CR2 -> SADD[7:1]: 7 bit Slave Address
		
		I2C_ISR: I2C State indicators ------------------
		I2C_ISR -> TXIS: TransferRegister Empty/Ready
		I2C_ISR -> NACKF: Slave did not transmit an acknowledge bit - error occurred
		I2C_ISR -> TC: transfer complete bit is set by hardware
		
		I2C_RXDR [7:0]: 8 bit received data
		I2C_TXDR [7:0]: 8 bit transmit data
		
		*/
		
		GPIOC -> ODR ^= GPIO_ODR_6;
		GPIOC -> ODR ^= GPIO_ODR_7;
		GPIOC -> ODR ^= GPIO_ODR_8;
		GPIOC -> ODR ^= GPIO_ODR_9;
		HAL_Delay(250);
		GPIOC -> ODR ^= GPIO_ODR_6;
		GPIOC -> ODR ^= GPIO_ODR_7;
		GPIOC -> ODR ^= GPIO_ODR_8;
		GPIOC -> ODR ^= GPIO_ODR_9;
		HAL_Delay(250);
		
		
		I2C2 -> CR2 |= (I2C_CR2_SADD_Msk & ( 0x6B << 0)); //set the slave address as 0x6B
		I2C2 -> CR2 |= (I2C_CR2_NBYTES_Msk & ( 0x1 << I2C_CR2_NBYTES_Pos)); // transfer 1 byte
		I2C2 -> CR2 |= (I2C_CR2_RD_WRN_Msk & (0x1 << I2C_CR2_RD_WRN_Pos)); //perform write operation
		I2C2 -> CR1 |= (I2C_CR2_START_Msk & (0x1 << I2C_CR2_START_Pos)); //Start bit = 1
		
		
		//TRANSMIT WHO AM I STEP -----------------------------------------------------------------------------
		int init_trans_complete = 0;
		while(init_trans_complete == 0){//repeatidly checking TXIS and NACKF flag for change in state.
			if((I2C2 -> CR2 & I2C_ISR_TXIS_Msk) == 1){
				init_trans_complete = 1;
			}
			else if((I2C2 -> CR2 & I2C_ISR_NACKF_Msk) == 1){//checking for a no acknowledge bit returned
				//no acknowledge bit was returned, ERROR, exit the program.
				GPIOC -> ODR ^= GPIO_ODR_7;//this is temporarily used as an error indicator COMMENT OUT LATER--------
				HAL_Delay(2000);
				return 0;//exit program
			}
		}
		//acknowledge bit from slave was received, proceed with sending of data
		I2C2 -> TXDR = 0b11010011;//send the address of the Who am I register to the slave device
		while((I2C2 -> ISR & I2C_ISR_TC_Msk) == 0){}//Wait until transfer complete bit is set
			
		//RECEIVE WHO AM I DEVICE ID -------------------------------------------------------------------------
		I2C2 -> CR2 |= (I2C_CR2_SADD_Msk & ( 0x6B << 0)); //set the slave address as 0x6B
		I2C2 -> CR2 |= (I2C_CR2_NBYTES_Msk & ( 0x1 << I2C_CR2_NBYTES_Pos)); // transfer 1 byte
		I2C2 -> CR2 |= (I2C_CR2_RD_WRN_Msk & (0x0 << I2C_CR2_RD_WRN_Pos)); //perform read operation
		I2C2 -> CR1 |= (I2C_CR2_START_Msk & (0x1 << I2C_CR2_START_Pos)); //Start bit = 1
		
		init_trans_complete = 0;
		while(init_trans_complete == 0){//repeatidly checking RXNE and NACKF flag for change in state.
			if((I2C2 -> CR2 & I2C_ISR_RXNE_Msk) == 1){
				init_trans_complete = 1;
			}
			else if((I2C2 -> CR2 & I2C_ISR_NACKF_Msk) == 1){//checking for a no acknowledge bit returned
				//no acknowledge bit was returned, ERROR, exit the program.
				GPIOC -> ODR ^= GPIO_ODR_7;//this is temporarily used as an error indicator COMMENT OUT LATER--------
				HAL_Delay(2000);
				return 0;//exit program
			}
		}
		
		//Checking if received id is correct-----------------------------------------------------------------
		int receivedData = I2C2 -> RXDR;
		if( receivedData != 0xD4 ){
			GPIOC -> ODR ^= GPIO_ODR_7;//this is temporarily used as an error indicator COMMENT OUT LATER--------
			HAL_Delay(2000);
			return 0;//exit program
		}
		
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		GPIOC -> ODR ^= GPIO_ODR_6;
		GPIOC -> ODR ^= GPIO_ODR_7;
		GPIOC -> ODR ^= GPIO_ODR_8;
		GPIOC -> ODR ^= GPIO_ODR_9;
		HAL_Delay(250);
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
