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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "visEffect.h"
#include "ws2812b.h"
#include "i2c.h"
#include "MPU6050.h"
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
 ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2_blinky */
osThreadId_t Task2_blinkyHandle;
const osThreadAttr_t Task2_blinky_attributes = {
  .name = "Task2_blinky",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task3_PWM */
osThreadId_t Task3_PWMHandle;
const osThreadAttr_t Task3_PWM_attributes = {
  .name = "Task3_PWM",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task4 */
osThreadId_t Task4Handle;
const osThreadAttr_t Task4_attributes = {
  .name = "Task4",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
WS2812_Struct ws2812b;

MPU_ConfigTypeDef myConfig;
ScaledData_Def meAccel;
char buffer[50];
char charbuffer[50];
uint8_t Cmd_End[3] = {0xFF, 0xFF, 0xFF};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void Task2_init(void *argument);
void Task3_init(void *argument);
void Task4_init(void *argument);

/* USER CODE BEGIN PFP */
  // command end sequence

void NEXTION_SendString (char *ID, char *string)
{
	memset(charbuffer,'\0',50);
    sprintf (charbuffer, "%s.txt=\"%s\"", ID, string);
	HAL_UART_Transmit (&huart1,(uint8_t *) charbuffer, strlen(charbuffer), HAL_MAX_DELAY);
	HAL_UART_Transmit (&huart1, Cmd_End, 3, HAL_MAX_DELAY);
	HAL_UART_Transmit (&huart1,(uint8_t *) buffer, strlen(buffer), HAL_MAX_DELAY);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void send_task2 (void)
{

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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit (&huart1, (uint8_t*)"rest", 4, HAL_MAX_DELAY);
  HAL_UART_Transmit (&huart1, Cmd_End, 3, HAL_MAX_DELAY);

  i2c_init();

  myConfig.Accel_Full_Scale = AFS_SEL_4g;
  myConfig.ClockSource = Internal_8MHz;
  myConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
  myConfig.Sleep_Mode_Bit = 0;
  myConfig.Gyro_Full_Scale = FS_SEL_500;

  MPU6050_Config(&myConfig);

  visInit("GRB");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task2_blinky */
  Task2_blinkyHandle = osThreadNew(Task2_init, NULL, &Task2_blinky_attributes);

  /* creation of Task3_PWM */
  Task3_PWMHandle = osThreadNew(Task3_init, NULL, &Task3_PWM_attributes);

  /* creation of Task4 */
  Task4Handle = osThreadNew(Task4_init, NULL, &Task4_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

 /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Battery_Monitoring_Pin|LD2_Pin|Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(battery_power_GPIO_Port, battery_power_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Battery_Monitoring_Pin LD2_Pin Buzzer_Pin Fan_Pin */
  GPIO_InitStruct.Pin = Battery_Monitoring_Pin|LD2_Pin|Buzzer_Pin|Fan_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : battery_power_Pin */
  GPIO_InitStruct.Pin = battery_power_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(battery_power_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	 visHandle();
  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task2_init */
/**
* @brief Function implementing the Task2_blinky thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task2_init */
void Task2_init(void *argument)
{
  /* USER CODE BEGIN Task2_init */
  /* Infinite loop */
  for(;;)
  {
	  NEXTION_SendString("t0", "12:00");
	  osDelay(50);

	  NEXTION_SendString("t2", "ID: AMR@123");
	  osDelay(70);
	  // Prepare the message for fan status
      memset(buffer,'\0',50);
	  sprintf(buffer, "Fan Status: %d \n\r",HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7));
//	  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	  NEXTION_SendString("t1", buffer);
	  osDelay(50);

	  NEXTION_SendString("t4", "LED strip running");
      osDelay(50);

      MPU6050_Get_Accel_Scale(&meAccel);
      memset(buffer,'\0',50);
      sprintf(buffer, "ACC:x=%.2f,y=%.2f,z=%.2f", meAccel.x, meAccel.y, meAccel.z);
//    "ACC:x=%.2f,y=%.2f,z=%.2f\n\r"
//    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
      NEXTION_SendString("t5", buffer);
      osDelay(70);
  }
  /* USER CODE END Task2_init */
}

/* USER CODE BEGIN Header_Task3_init */
/**
* @brief Function implementing the Task3_PWM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task3_init */
void Task3_init(void *argument)
{
  /* USER CODE BEGIN Task3_init */
  /* Infinite loop */
  for(;;)
  {
  	 //Toggle the green LED
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	  osDelay(200);
  }
  /* USER CODE END Task3_init */
}

/* USER CODE BEGIN Header_Task4_init */
/**
* @brief Function implementing the Task4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task4_init */
void Task4_init(void *argument)
{
  /* USER CODE BEGIN Task4_init */

  /* Infinite loop */
  for(;;)
  {
	  uint16_t adcValue;
	  float voltage;
	  char msg[100];
	  char fanStatus = '0'; // Initialize fan status to '0'
	  char rxData;



	      if (HAL_UART_Receive(&huart2, (uint8_t*)&rxData, 1, 10) == HAL_OK)
	      {
	          if (rxData == '1' || rxData == '0')
	          {
	               fanStatus = rxData;  // Update fan status based on received data

	               // Control the relay based on fanStatus
	               if (fanStatus == '1')
	               {
	                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);    // Turn on the fan (activate relay)
	               }
	               else
	               {
	                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  // Turn off the fan (deactivate relay)
	               }
	          }
	      }

	      //Start ADC Conversion
	      HAL_ADC_Start(&hadc1);


	  	  // Poll for conversion completion
	      if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
	  	  {
	  		  // Get ADC value
	  		  adcValue = HAL_ADC_GetValue(&hadc1);

	  		  // Convert ADC value to voltage
	  		  voltage = ((float)(adcValue * 3.3 )/ 4095.0);

	  		  // Check if voltage is below the threshold
	  		  if (voltage < 1.5)
	  	      {
	  			    // Activate the buzzer and print a message
	  			    // Turn on the buzzer
	                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	                memset(buffer,'\0',50);
	                sprintf(buffer, "battery Low: %.2f V", voltage);
	                NEXTION_SendString("t3", buffer);
	                osDelay(500);
	                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	               	memset(buffer,'\0',50);
	                NEXTION_SendString("t3","Battery Charged");
	                osDelay(50);
	  		  }
	  		  else
	  		  {
	  			    // Turn off the buzzer
	                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	                memset(buffer,'\0',50);
	                NEXTION_SendString("t3","Battery Charged");
	                osDelay(50);
	          }


	  		// Prepare the message with ADC value, voltage, and fan status
	  		sprintf(msg, "ADC Value: %d, Voltage: %.2f V, Fan Status: %c, ACC:x=%.2f,y=%.2f,z=%.2f\r\n",
	  	    adcValue, voltage, fanStatus, meAccel.x, meAccel.y, meAccel.z);

	  		// Send the message via UART
	  		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	  	  }
		 //Stop ADC Conversion
		 HAL_ADC_Stop(&hadc1);

		 osDelay(500);
  }
  /* USER CODE END Task4_init */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

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
