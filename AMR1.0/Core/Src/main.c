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
#include <stdio.h>
#include <string.h>
#include "mpu6050.h"
#include "visEffect.h"
#include "ws2812b.h"


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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for NexDisplay */
osThreadId_t NexDisplayHandle;
const osThreadAttr_t NexDisplay_attributes = {
  .name = "NexDisplay",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AccelGyro */
osThreadId_t AccelGyroHandle;
const osThreadAttr_t AccelGyro_attributes = {
  .name = "AccelGyro",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for VisEffect */
osThreadId_t VisEffectHandle;
const osThreadAttr_t VisEffect_attributes = {
  .name = "VisEffect",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
WS2812_Struct ws2812b;

uint8_t Fan_bit = 0;

uint8_t RxData[10] = { };
uint8_t Cmd_End[3] = {0xFF,0xFF,0xFF};
uint8_t validCommands[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x40, 0x50}; // Valid command bytes array
float voltage = 0.0;

uint8_t switchvar = 0;

MPU6050_t MPU6050;



HAL_StatusTypeDef status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartNextion(void *argument);
void StartAccel(void *argument);
void StartVisEffect(void *argument);

/* USER CODE BEGIN PFP */

//void SendInvalidMessage(UART_HandleTypeDef *huart);
//void ProcessPacket(uint8_t *packet, uint8_t length);


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    Fan_bit = RxData[0] - 48;

    HAL_UART_Receive_IT(&huart2, RxData, 1);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void NEXTION_SendString (char *ID, char *string)
{
  uint8_t charbuffer[50] = { };
  sprintf((char *)charbuffer, "%s.txt=\"%s\"", ID, string);
  HAL_UART_Transmit (&huart1,(uint8_t *) charbuffer, strlen((const char *)charbuffer), HAL_MAX_DELAY);
  HAL_UART_Transmit (&huart1, Cmd_End, 3, HAL_MAX_DELAY);
}

//void SendInvalidMessage(UART_HandleTypeDef *huart)
//{
//    char *msg = "Invalid Command\r\n";
//    HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//}
//
//
//void ProcessPacket(uint8_t *packet, uint8_t length)
//{
//    // Custom function to process the packet if all checks are passed
//    // Example: simply print the packet's contents to UART2 for verification
//    char msg[100];
//    snprintf(msg, sizeof(msg), "Valid Packet Received: ");
//    for (uint8_t i = 0; i < length; i++)
//    {
//        char byteStr[5];
//        snprintf(byteStr, sizeof(byteStr), "%02X ", packet[i]);
//        strncat(msg, byteStr, sizeof(msg) - strlen(msg) - 1);
//    }
//    strncat(msg, "\r\n", sizeof(msg) - strlen(msg) - 1);
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//}


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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, RxData, 1);

  while (MPU6050_Init(&hi2c1) == 1);

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

  /* creation of NexDisplay */
  NexDisplayHandle = osThreadNew(StartNextion, NULL, &NexDisplay_attributes);

  /* creation of AccelGyro */
  AccelGyroHandle = osThreadNew(StartAccel, NULL, &AccelGyro_attributes);

  /* creation of VisEffect */
  VisEffectHandle = osThreadNew(StartVisEffect, NULL, &VisEffect_attributes);

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Buzzer_Pin|Fan_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Buzzer_Pin Fan_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Buzzer_Pin|Fan_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	  if(voltage <= 1.5)
	  {
		  HAL_GPIO_TogglePin(GPIOA, Buzzer_Pin);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
      }

	  if(Fan_bit)
	  {
	      //Turn ON Fan
	      HAL_GPIO_WritePin(GPIOA, Fan_Pin, GPIO_PIN_SET);
	  }
	  else
	  {
	      //Turn OFF Fan
	      HAL_GPIO_WritePin(GPIOA, Fan_Pin, GPIO_PIN_RESET);
      }
      osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartNextion */
/**
* @brief Function implementing the NexDisplay thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNextion */
void StartNextion(void *argument)
{
  /* USER CODE BEGIN StartNextion */
  /* Infinite loop */
  for(;;)
  {
	      switchvar++;
	      switch(switchvar)
	      {
	          case 1:
	              NEXTION_SendString("t0", "12:00");
	              break;
	          case 2:
	              if(Fan_bit)
	              {
	                  NEXTION_SendString("t1", "Fan ON");
	              }
	              else
	              {
	                  NEXTION_SendString("t1", "Fan OFF");
	              }
	              break;
	          case 3:
	              NEXTION_SendString("t2", "AMR@123");
	              break;
	          case 4:
	              if(voltage >= 1.5)
	              {
	                  NEXTION_SendString("t4", "Battery Charged");

	              }
	              else
	              {
	                  NEXTION_SendString("t4", "Battery Low");
	              }
	              break;
	          case 5:
	              uint8_t Abuffer[50] = { };
	              sprintf((char *)Abuffer, "ACC:x=%.2f,y=%.2f,z=%.2f", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
	              NEXTION_SendString("t5",(char *) Abuffer);
	              break;
	          case 6:
	              uint8_t Gbuffer[50] = { };
	              sprintf((char *)Gbuffer, "GYRO:x=%.2f,y=%.2f,z=%.2f", MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);
	              NEXTION_SendString("t6",(char *) Gbuffer);
	              break;
	          case 7:
	              break;
	          default:
	              switchvar = 0;
	              break;
         }
         osDelay(100);
  }
  /* USER CODE END StartNextion */
}

/* USER CODE BEGIN Header_StartAccel */
/**
* @brief Function implementing the AccelGyro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAccel */
void StartAccel(void *argument)
{
  /* USER CODE BEGIN StartAccel */
  /* Infinite loop */
  for(;;)
  {
	  MPU6050_Read_All(&hi2c1, &MPU6050);
	  osDelay(100);
	  //Start ADC Conversion
	  HAL_ADC_Start(&hadc1);
	  // Poll for conversion completion
	  if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
	  {
	     // Get ADC value
	     uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
	     // Convert ADC value to voltage
	     voltage = ((float)(adcValue * 3.3 )/ 4095.0);
	  }
	  //Stop ADC Conversion
	  HAL_ADC_Stop(&hadc1);

  }
  /* USER CODE END StartAccel */
}

/* USER CODE BEGIN Header_StartVisEffect */
/**
* @brief Function implementing the VisEffect thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVisEffect */
void StartVisEffect(void *argument)
{
  /* USER CODE BEGIN StartVisEffect */
  /* Infinite loop */
  for(;;)
  {
	visHandle();
	osDelay(10);
  }
  /* USER CODE END StartVisEffect */
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
//
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
//
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
