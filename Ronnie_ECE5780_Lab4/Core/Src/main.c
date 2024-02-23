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
volatile char globalinput;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sendUSART3(char* stringinput);
void sendCharUSART3(char singleChar);
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
	//SYSCFG Clock------------------
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	//USART Clock-------------------
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	//CONFIGURATION OF THE USART3 Peripheral -----------------
	//USART3_CR1_M1 = 00 by default ie. 8 bit transmission width
	USART3 -> BRR = 69; //BRR = f_clk / Targ_Baud = 8,000,000 / 115200put
	//USART3_CR2_STOP == 00 by defualt = 1 stop bits.
	USART3 -> CR1 |= USART_CR1_UE; //enable this USART (usart3)
	USART3 -> CR1 |= USART_CR1_TE; //Transmission enable sends idle frame as first transmission.
	USART3 -> CR1 |= USART_CR1_RE; //Receiving Enable bit 
	
	//CHECKOFF #2 SETUP OF INTERRUPTS BASED ON DATA INPUT -----------
	USART3 -> CR1 |= USART_CR1_RXNEIE; //generate interrupt when rxne or ore goes high.
	USART3 -> CR1 |= USART_CR1_RE;
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn,2);
	NVIC_SetPriority(SysTick_IRQn,0);
	
	//CONFIGURATION of PB10-14 pins for USART usage -------------------
	GPIOC -> MODER |= GPIO_MODER_MODER4_1;//setting PC10 Alternate MODER
	GPIOC -> MODER |= GPIO_MODER_MODER5_1;//setting PC11
	
	GPIOC -> AFR[0] |= 0x1 << GPIO_AFRL_AFRL4_Pos;//AF1
	GPIOC -> AFR[0] |= 0x1 << GPIO_AFRL_AFRL5_Pos;//AF1
	
	GPIOC -> MODER |= GPIO_MODER_MODER6_0;
	GPIOC -> MODER |= GPIO_MODER_MODER7_0;
	GPIOC -> MODER |= GPIO_MODER_MODER8_0;
	GPIOC -> MODER |= GPIO_MODER_MODER9_0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	char* error = "there was an error";
	char* hellomessage = "   Your USART3 is working \n";
	char testChar = 'a';
	char* cmdRequest = " Command: ";
	char* green0 = " - Green 0 Received";
	char* green1 = " - Green 1 Received";
	char* green2 = " - Green 2 Received";
	char* red0 = " - Red 0 Received";
	char* red1 = " - Red 1 Received";
	char* red2 = " - Red 2 Received";
	char* orange0 = " - Orange 0 Received";
	char* orange1 = " - Orange 1 Received";
	char* orange2 = " - Orange 2 Received";
	char* blue0 = " - Blue 0 Received";
	char* blue1 = " - Blue 1 Received";
	char* blue2 = " - Blue 2 Received";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	volatile char rxBuffer = 0;
	
	sendCharUSART3(testChar);
	sendUSART3(hellomessage);

  while (1)
  {
		//Part 1 Main based input --------------------
		/*
		//BLOCKING CODE FOR USART Receiving of data.
		if((USART3 -> ISR &= USART_ISR_RXNE_Msk) != 0)
		{
			rxBuffer = USART3 -> RDR;
			USART3 -> CR1 |= USART_CR1_RE; //Data removed, enable next data read.
			switch(rxBuffer)
			{
				case 'r':
					GPIOC -> ODR ^= GPIO_ODR_6;
					break;
				case 'o':
					GPIOC -> ODR ^= GPIO_ODR_8;
					break;
				case 'g':
					GPIOC -> ODR ^= GPIO_ODR_9;
					break;
				case 'b':
					GPIOC -> ODR ^= GPIO_ODR_7;
					break;
				default:
					sendUSART3(error);
					break;
			}
		}
		else
		{    }
		*/
		
		//PART 2 Interrupt based input----------------------
		sendUSART3(cmdRequest);
		globalinput = 0;
		while(globalinput == 0){}
		sendCharUSART3(globalinput);
		switch(globalinput)
			{
				case 'r':
					globalinput = 0;
					while(globalinput == 0){}
					sendCharUSART3(globalinput);
					switch(globalinput)
					{
						case '0':
							GPIOC -> ODR &= ~GPIO_ODR_6;
							sendUSART3(red0);
							globalinput = 0;
						break;
						case '1':
							GPIOC -> ODR |= GPIO_ODR_6;
							sendUSART3(red1);
							globalinput = 0;
						break;
						case '2':
							GPIOC -> ODR ^= GPIO_ODR_6;
							sendUSART3(red2);
							globalinput = 0;
						break;
						default:
							sendUSART3(error);
							globalinput = 0;
						break;
					}
					break;
				case 'o':
					globalinput = 0;
					while(globalinput == 0){}
					sendCharUSART3(globalinput);
					switch(globalinput)
					{
						case '0':
							GPIOC -> ODR &= ~GPIO_ODR_8;
							sendUSART3(orange0);
							globalinput = 0;
						break;
						case '1':
							GPIOC -> ODR |= GPIO_ODR_8;
							sendUSART3(orange1);
							globalinput = 0;
						break;
						case '2':
							GPIOC -> ODR ^= GPIO_ODR_8;
							sendUSART3(orange2);
							globalinput = 0;
						break;
						default:
							sendUSART3(error);
							globalinput = 0;
						break;
					}
					break;
				case 'g':
					globalinput = 0;
					while(globalinput == 0){}
					sendCharUSART3(globalinput);
					switch(globalinput)
					{
						case '0':
							GPIOC -> ODR &= ~GPIO_ODR_9;
							sendUSART3(green0);
							globalinput = 0;
						break;
						case '1':
							GPIOC -> ODR |= GPIO_ODR_9;
							sendUSART3(green1);
							globalinput = 0;
						break;
						case '2':
							GPIOC -> ODR ^= GPIO_ODR_9;
							sendUSART3(green2);
							globalinput = 0;
						break;
						default:
							sendUSART3(error);
							globalinput = 0;
						break;
					}
					break;
				case 'b':
					globalinput = 0;
					while(globalinput == 0){}
					sendCharUSART3(globalinput);
					switch(globalinput)
					{
						case '0':
							GPIOC -> ODR &= ~GPIO_ODR_7;
							sendUSART3(blue0);
							globalinput = 0;
						break;
						case '1':
							GPIOC -> ODR |= GPIO_ODR_7;
							sendUSART3(blue1);
							globalinput = 0;
						break;
						case '2':
							GPIOC -> ODR ^= GPIO_ODR_7;
							sendUSART3(blue2);
							globalinput = 0;
						break;
						default:
							sendUSART3(error);
							sendCharUSART3(globalinput);
							globalinput = 0;
						break;
					}
					break;
				default:
					sendUSART3(error);
					globalinput = 0;
					break;
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

void sendCharUSART3(char singleChar)
{
	while((USART3 -> ISR &= USART_ISR_TXE_Msk) == 0)
	{/*Transmit Data register must be empty before beginning transmission*/}
	
	//USART3 -> CR1 |= USART_CR1_TE;//Enable transmission of data
	USART3 -> TDR = singleChar;
}

void sendUSART3(char* stringinput)
{
	while((USART3 -> ISR &= USART_ISR_TXE_Msk) == 0)
	{/*Transmit Data register must be empty before beginning transmission*/}
	int index = 0;

	while(stringinput[index] != 0){
		
		sendCharUSART3(stringinput[index]);
		index++;
		while((USART3 -> ISR &= USART_ISR_TXE_Msk) == 0)
		{/*Transmit Data register must be empty before beginning transmission*/}
		
	}
	
	return;
}

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
