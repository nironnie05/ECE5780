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
#define gyroID 0x69
#define WHOAMI 0x0F
#define xLowID 0x28
#define xHighID 0x29
#define yLowID 0x2A
#define yHighID 0x2B

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sendRegAddr(uint16_t devID, uint8_t regID);
void writeReg(uint16_t devID, uint8_t regID, uint8_t data);
uint8_t readReg(uint16_t devID);

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
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	//RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	//I2C2 TIMINGR Setup
	I2C2->TIMINGR |= (0x1 << 28); // PRESC
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
	
	//LED OUTPUT SETUP---------------------------------------------------------------------------------
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
	
	//Peripheral Enable I2C2
	I2C2->CR1 |= I2C_CR1_PE; 
	
  /* USER CODE END SysInit */
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
		GYRO NOTES:
		0x29 = high 8 bits of x axis
		0x28 = low  8 bits of x axis
		0x2B = high 8 bits of y axis
		0x2A = low  8 bits of y axis
		
		*/
		//1/2 sec LED test
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
		
		//PART 1:
		
		//TRANSMIT WHO AM I STEP -----------------------------------------------------------------------------
		sendRegAddr(gyroID, WHOAMI);

		//RECEIVE WHO AM I DEVICE ID -------------------------------------------------------------------------
		uint8_t data = readReg(gyroID);
		if(data == 0xD3){//I2C w/ Gyro works
			GPIOC -> ODR ^= GPIO_ODR_6;
			GPIOC -> ODR ^= GPIO_ODR_7;
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
			GPIOC -> ODR ^= GPIO_ODR_6;
			GPIOC -> ODR ^= GPIO_ODR_7;
		}
		else{//I2C w/ Gyro Failed
			GPIOC -> ODR ^= GPIO_ODR_7;
			HAL_Delay(250);
			GPIOC -> ODR ^= GPIO_ODR_7;
			HAL_Delay(250);
			GPIOC -> ODR ^= GPIO_ODR_7;
			HAL_Delay(250);
			GPIOC -> ODR ^= GPIO_ODR_7;
			HAL_Delay(250);
		}
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//PART 2 ------------------------------------------------------------------------------------
	int8_t normal_mode = 0xB;
	int8_t ctrl_reg1_addr = 0x20;
	writeReg(gyroID, ctrl_reg1_addr, normal_mode);
	
	int32_t thresh = 1000;
  while (1)
  {
		//get xlow data
		sendRegAddr(gyroID, 0x28);
		int8_t xlowdat = readReg(gyroID);
		
		//get xhigh data
		sendRegAddr(gyroID, 0x29);
		int8_t xhighdat = readReg(gyroID);
		
		//get ylow data
		sendRegAddr(gyroID, 0x2A);
		int8_t ylowdat = readReg(gyroID);
		
		//get yhigh data
		sendRegAddr(gyroID, 0x2B);
		int8_t yhighdat = readReg(gyroID);
		
		//calculating x and y value, high = upper 8 bits, low = lower 8 bits
		int16_t xdat = ((int16_t)xhighdat << 8) | (uint8_t)xlowdat;
		int16_t ydat = ((int16_t)yhighdat << 8) | (uint8_t)ylowdat;
		GPIOC->ODR &= ~(GPIO_ODR_7 | GPIO_ODR_6 | GPIO_ODR_8 | GPIO_ODR_9); //turn off all LEDS
		
		if (ydat > thresh) {
				GPIOC -> ODR |= GPIO_ODR_6; // Red LED for positive Y 
		} else if (ydat < -thresh) {
				GPIOC -> ODR |= GPIO_ODR_7; // Blue LED for negative Y 
		}

		if (xdat > thresh) {
				GPIOC -> ODR |= GPIO_ODR_9; // Green LED for positive X
		} else if (xdat < -thresh) {
				GPIOC -> ODR |= GPIO_ODR_8; // Orange LED for negative X
		}
		
		HAL_Delay(100);
		
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

void sendRegAddr(uint16_t devID, uint8_t regID)
{
	I2C2 -> CR2 = 0;//reset cr2 register
	I2C2 -> CR2 |= (devID << 1);//store devID into SADD[7:1]
	I2C2 -> CR2 |= (I2C_CR2_RD_WRN_Msk & (0x0 << I2C_CR2_RD_WRN_Pos));//set rd_wrn bit = 0, write operation
	I2C2 -> CR2 |= (0x1 << I2C_CR2_NBYTES_Pos);//store 0x1 into NBYTES [7:0] ie. send 1 byte
	I2C2 -> CR2 |= (0x1 << I2C_CR2_START_Pos);//set START bit of CR2 to be 1
	
	
	while (!(I2C2 -> ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {}
	// Once RXNE flag set continue
	
	// Check if NACK set
	if (I2C2->ISR & I2C_ISR_NACKF)
	{
		//GPIOC->ODR |= GPIO_ODR_8; // orange led turned on
	}
	
	I2C2 -> TXDR = regID;

	while (!(I2C2 -> ISR & I2C_ISR_TC)) {}
	
}

uint8_t readReg(uint16_t devID)
{
	uint8_t data;
	I2C2 -> CR2 = 0;//reset cr2 register
	I2C2 -> CR2 |= (devID << 1);//store devID into SADD[7:1]
	I2C2 -> CR2 |= (I2C_CR2_RD_WRN_Msk & (0x1 << I2C_CR2_RD_WRN_Pos));//set rd_wrn bit = 1, read operation
	I2C2 -> CR2 |= (0x1 << I2C_CR2_NBYTES_Pos);//store 0x1 into NBYTES [7:0] ie. send 1 byte
	I2C2 -> CR2 |= (0x1 << I2C_CR2_START_Pos);//set START bit of CR2 to be 1
	
	
	while (!(I2C2 -> ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF))) {}
	// Once RXNE flag set continue
	
	// Check if NACK set
	if (I2C2 -> ISR & I2C_ISR_NACKF)
	{
		//GPIOC->ODR |= GPIO_ODR_8; // orange led turned on
	}
	while (!(I2C2 -> ISR & I2C_ISR_TC)) {}//TC flag indicates that the read register has been filled with data
	data = I2C2 -> RXDR;
	return data;
}


//IGNORE FOR NOW
void writeReg(uint16_t devID, uint8_t regID, uint8_t data)
{
	
	I2C2 -> CR2 = 0;//reset cr2 register
	I2C2 -> CR2 |= (devID << 1);//store devID into SADD[7:1]
	I2C2 -> CR2 &= ~(1 << 10);//set rd_wrn bit = 0, write operation
	I2C2 -> CR2 |= (0x2 << I2C_CR2_NBYTES_Pos);//store 0x2 into NBYTES [7:0] ie. send 2 byte
	I2C2 -> CR2 |= (0x1 << I2C_CR2_START_Pos);//set START bit of CR2 to be 1
	
	while (!(I2C2 -> ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {}
	// Once RXNE flag set continue
	
	// Check if NACK set
	if (I2C2 -> ISR & I2C_ISR_NACKF)
	{
		//GPIOC->ODR |= GPIO_ODR_8; // orange led turned on
	}
	
	I2C2 -> TXDR = regID;//we want to transmit the desired register to modify
	
	while (!(I2C2->ISR & I2C_ISR_TXIS)) {}//wait until that transfer is complete
	
	I2C2 -> TXDR = data;//tells the slave device to store the "data" 8 bits of data into the previously provided register address/ID
		
	while (!(I2C2->ISR & I2C_ISR_TC)) {}// wait until transfer is complete to proceed

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
