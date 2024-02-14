/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern volatile int tim1_int_count;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
volatile int tim1_int_count;
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	tim1_int_count += 1;
	if(tim1_int_count >= 200){
			tim1_int_count = 0;
			GPIOC -> ODR ^= GPIO_ODR_7;
	}
	
	
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
//Declaring the interrupt handler of EXTI0_1 (Push Button falling edge)
//NOTE: THIS CODE DOES NOT DIRECTLY REPLICATE LAB INSTRUCTIONS:
//      because the sys_timer is of higher priority, I used the tim_1_interrupt counter
//      to more accurately track the time that we want the EXTI0_1 interrupt to prevent 
//      the main control loop from resuming.
volatile int tim1_overflowCount;
void EXTI0_1_IRQHandler(void)
{
		char check_int_latch = 0b0000;
		char exitLoop = 0b0000;
		tim1_overflowCount = 0;
		GPIOC -> ODR ^= GPIO_ODR_8;
		GPIOC -> ODR ^= GPIO_ODR_9;
		
		while(exitLoop == 0){
			if(tim1_int_count == 0){//tim1_int_count resets to 0 every 200 ms
				if(check_int_latch != 0) {tim1_overflowCount ++;}
				check_int_latch = 0b0000;
			}
			if(tim1_int_count != 0){//tim1_int_count resets to 0 every 200 ms
				check_int_latch = 0b0001;
			}
			if(tim1_overflowCount == 15)//200 ms * 15 = 3 sec delay before exiting this interrupt.
			{
				exitLoop = 0b0001;
			}
		}
	
		EXTI->PR |= 1;	
	
		GPIOC -> ODR ^= GPIO_ODR_8;
		GPIOC -> ODR ^= GPIO_ODR_9;
}

/* USER CODE END 1 */
