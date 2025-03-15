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
#include "miros.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Mutex_t mutex1;
Mutex_t mutex2;
Mutex_t mutex3;

Event_Group_t  event_group_1;
#define BIT_1 (1U << 1)
#define BIT_2 (1U << 2)
#define BIT_3 (1U << 3)

OS_Queue_t mrtos_queue;
uint32_t buff[10];

Stream_Buffer_t  stream_buffer_1;
char tx_string[]= "fine";

uint8_t timer_test=0;
soft_timer_t soft_timer_1;
void Timer_Handler_1()
{
  timer_test ++;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void OS_onIdle()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
}
void OS_onStartup(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);

    /* set the SysTick interrupt priority (highest) */
    NVIC_SetPriority(SysTick_IRQn, 0U);
}
uint32_t stack_blinky1[40];
OSThread blinky1;
void main_blinky1() {
//    OS_SoftwareTimer_Start(&soft_timer_1);
    while (1) {
//       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
//       OS_Mutex_Get(&mutex1,30,1000);
//       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
//       OS_delay(1000);
//       OS_Mutex_Release(&mutex1);

//		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
//		 OS_delay(1000);
//		 OS_Queue_Push(&mrtos_queue, 15,waitForever);
//		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
//		 OS_delay(1000);


//       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
//		 OS_delay(1000);
//		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
//		 OS_EventGroup_SetBits(&event_group_1, BIT_1 | BIT_2 );
//		 OS_delay(1000);
//		 OS_EventGroup_SetBits(&event_group_1, BIT_3);


//      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
//      OS_delay(1000);
//      //OS_Stream_Buffer_Send(&stream_buffer_1,tx_string,strlen(tx_string),waitForever);
//      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
//      OS_delay(1000);
    }
}

uint32_t stack_blinky2[40];
OSThread blinky2;
uint32_t temp1;
uint8_t temp2;
char rx_string[100];
void main_blinky2() {
    while (1) {

//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
//		OS_delay(1000);
//		OS_Mutex_Get(&mutex1,30,5000);
//		OS_delay(1000);
//		OS_Mutex_Release(&mutex1);

//		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
//		 OS_Queue_Pop(&mrtos_queue, &temp1,100);
//		 OS_delay(100);
//		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
//		 OS_delay(1000);

//     	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
//		 OS_delay(1000);
//		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
//		 temp2=OS_EventGroup_WaitBits(&event_group_1, BIT_1 | BIT_2 |BIT_3,1,1,waitForever);
//		 OS_delay(1000);

//      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
//      OS_delay(1000);
//      OS_Stream_Buffer_Receive(&stream_buffer_1,rx_string,6,0,1500);
//      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
//      OS_delay(1000);
    }
}
uint32_t stack_idleThread[40];


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  	MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  	OS_init(stack_idleThread, sizeof(stack_idleThread));

      /* start blinky1 thread */
      OSThread_start(&blinky1,
                     2U, /* priority */
                     &main_blinky1,
                     stack_blinky1, sizeof(stack_blinky1));

      /* start blinky2 thread */
      OSThread_start(&blinky2,
                     5U, /* priority */
                     &main_blinky2,
                     stack_blinky2, sizeof(stack_blinky2));


//      OS_Binary_Semaphore_Init();

      OS_Mutex_Init(&mutex1);


      OS_Queue_Init(&mrtos_queue, buff, 10);

//      OS_SoftwareTimer_Init(&soft_timer_1,3000,Timer_Handler_1);

        OS_EventGroup_Init(&event_group_1);
//
//        OS_Stream_Buffer_Init(&stream_buffer_1,30,6);
      /* transfer control to the RTOS to run the threads */
      OS_run();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
