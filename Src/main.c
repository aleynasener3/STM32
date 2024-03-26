/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "led.h"
#include "ultrasonic2.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <mpu6050.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
t_TriggerHandler trig;
t_TriggerHandler trig2;
t_LedHandler led;
t_mpu6050Handler gyro;
Results res;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char str1[100];
char str2[100];
char str3[100];
char str4[100];
char str5[100];
char str6[100];
char str7[100];
char str8[100];
char str9[100];
char str10[100];

char rxBuffer[5]; // Buffer to store received data
int rxIndex = 0; // Index for buffer position


int receivedNumber = 0;



void printer(const char* sentence){
	HAL_UART_Transmit(&huart2,(uint8_t*)sentence, strlen(sentence), HAL_MAX_DELAY);
	}
//



int receive_number(){
	memset(rxBuffer, 0, sizeof(rxBuffer));
	int index = 0;
	while (index< sizeof(rxBuffer)-1){
		HAL_UART_Receive(&huart2, (uint8_t*)(rxBuffer+index), 1, HAL_MAX_DELAY);
		rxBuffer[sizeof(rxBuffer) - 1] = '\0'; // Null-terminate the received data

		if(rxBuffer[index]=='\r'){
			break;
		}
		index++;

	}
	receivedNumber  = atoi(rxBuffer); // Convert received data to integer
	//HAL_UART_Receive_IT(&huart2, (uint8_t*)rxBuffer, 1);
	return receivedNumber;


}

const char* sentence2;
char getter(){
	HAL_UART_Receive(&huart2,(uint8_t*)sentence2, strlen(sentence2), HAL_MAX_DELAY );
	return *sentence2;
}

void UART_Callback_1(){

	while(1){
		if (rxIndex < sizeof(rxBuffer) - 1){

			if (rxBuffer[rxIndex] != '\r' ){

				rxIndex++;
			}else{
				rxBuffer[rxIndex] = '\0';
				// Process the received data
				receivedNumber = atoi(rxBuffer); // Convert received data to integer
				int result = receivedNumber * 2;

				// Send the result back
				sprintf(str1, "Result: %d\n\r", result);
				printer(str1);

				// Reset buffer index
				rxIndex = 0;

				// Re-enable UART RX interrupt
				HAL_UART_Receive_IT(&huart2, (uint8_t *)rxBuffer, sizeof(rxBuffer)-1);
				memset(rxBuffer, 0, sizeof(rxBuffer));
				break;

			}

		}else {
	        // Buffer overflow, handle error or clear buffer
	        rxIndex = 0;
	        break;


	    }

		 HAL_UART_Receive_IT(&huart2, (uint8_t *)rxBuffer, sizeof(rxBuffer)-1);

	}
}



char rxData[2];




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  //HAL_I2C_DeInit(&hi2c1);

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  Ultrasonic_Init(&trig,GPIOA,GPIO_PIN_4, &htim2, TIM_CHANNEL_1,HAL_TIM_ACTIVE_CHANNEL_1);
  //Ultrasonic_Init(&trig2,GPIOA,GPIO_PIN_5, &htim2, TIM_CHANNEL_2,HAL_TIM_ACTIVE_CHANNEL_2);
  Led_Init(&led,GPIOA,GPIO_PIN_5);

  //first_read(&gyro);

  printer("started\n\r");


  int result3 = mpu6050_init(&gyro,&hi2c1, 0b1101000 , 1, 100, 1 );





  __HAL_UART_CLEAR_OREFLAG(&huart2);

  __HAL_UART_CLEAR_NEFLAG(&huart2);





  //HAL_UART_Receive_IT(&huart2, (uint8_t*)rxBuffer, sizeof(rxBuffer) - 1); // Enable UART RX interrupt
  __enable_irq();   // Enable global interrupts
  //rxBuffer[sizeof(rxBuffer) - 1] = '\0'; // Null-terminate the received data
  memset(rxBuffer, 0, sizeof(rxBuffer));

  memset(rxData, 0, sizeof(rxData));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */








	  float acc_x = mpu6050_read_acc_x(&gyro);
	  float acc_y = mpu6050_read_acc_y(&gyro);
	  float acc_z = mpu6050_read_acc_z(&gyro);
	  acc_x = ((float)(acc_x))*10.0f; //m/s2
	  acc_y = ((float)(acc_y))*10.0f;
	  acc_z = ((float)(acc_z))*10.0f;

	  sprintf(str1,"%f\n%f\n%f\n a",acc_x,acc_y,acc_z);

	  printer(str1);
	  HAL_Delay(300);
	  /*
	  float temp = mpu6050_read_temp(&gyro);

	  float gyro_x = mpu6050_read_gyro_x(&gyro);
	  float gyro_y = mpu6050_read_gyro_y(&gyro);
	  float gyro_z = mpu6050_read_gyro_z(&gyro);

	  sprintf(str1,"%f",acc_x);
	  sprintf(str2,"%f",acc_y);
	  sprintf(str3,"%f",acc_z);

	  sprintf(str4,"%f",temp);

	  sprintf(str5,"%f",gyro_x);
	  sprintf(str6,"%f",gyro_y);
	  sprintf(str7,"%f",gyro_z);

	  printer("acc x: ");
	  printer(str1);
	  printer("     acc y: ");
	  printer(str2);
	  printer("     acc z: ");
	  printer(str3);
	  printer("\n\r");
	  printer("temp: ");
	  printer(str4);
	  printer("\n\r");
	  printer("gyro x: ");
	  printer(str5);
	  printer("      gyro y: ");
	  printer(str6);
	  printer("      gyro z: ");
	  printer(str7);
	  printer("\n\r\n\r\n\r");

	  HAL_Delay(1000);

	  mpu6050_read_gyro(&gyro,&res);

	  float x = gyro_x(&res);
	  float y = gyro_y(&res);
	  float z = gyro_z(&res);

	  sprintf(str1,"%f",x);
	  sprintf(str2,"%f",y);
	  sprintf(str3,"%f",z);

	  printer("x: ");
	  printer(str1);
	  printer("     y: ");
	  printer(str2);
	  printer("     z: ");
	  printer(str3);
	  printer("\n\r");
	  HAL_Delay(1000);
	  */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	//HAL_TIM_IC_CaptureCallback2(&trig, htim);
	//HAL_TIM_IC_CaptureCallback2(&trig2,htim);


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	/*HAL_UART_Receive_IT(&huart2, (uint8_t *)rxData, sizeof(rxData)-1);

	//HAL_UART_Receive(&huart2, (uint8_t*)rxData, sizeof(rxData), 100);

		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_Delay(1000);

	*/

	//UART_Callback_1();

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
