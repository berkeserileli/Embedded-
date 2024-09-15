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
#define MPU6050_ADDRESS  0x68<<1
#define MPU6050_READ 0xE1
#define MPU6050_Write 0xE0
#define who_am_i 0x75
#define power_config_reg_address 0x6B
#define sample_data_reg_address 0x19
#define config_reg_address 0x1A
#define gyro_config_reg_address 0x1B
#define accel_config_reg_address 0x1C
#define accel_xout_h_address 0x3B
#define gyro_xout_h_address 0x43
#define temp_out_h 0x41
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float a_x, a_y,a_z;
float g_x, g_y,g_z;
float temp;
uint8_t accel_data_x[6]; //Throughout x axis
uint8_t gyro_data_x[6]; //Throughout x axis
uint8_t var;
uint8_t temp_out[2];



void MPU6050_init(){
	uint8_t power_data = 0x00;
	uint8_t sample_data_rate = 0x07;
	uint8_t dlpf_data = 0x00;
	uint8_t gyro_data = 0x08;
	uint8_t accel_config_data = 0x08;

	  HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS,who_am_i,1,&var,1,10);


	  HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDRESS,power_config_reg_address,1,&power_data,1,100);   //Power Config Reg 0x68, Internal 8MHz oscillator
	  HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDRESS,sample_data_reg_address,1,&sample_data_rate,1,100); //Sample Data Rate 1kHz, [Gyro_Data_Rate(8Khz) / (1 + SMPRT_DIV)]
      HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDRESS,config_reg_address,1, &dlpf_data,1,100); //Config Reg 0x1A, default 8kHz ODR
      HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDRESS,gyro_config_reg_address,1,&gyro_data,1,100); //Gyro Config Reg 0x1B, 500'/s
      HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDRESS,accel_config_reg_address,1,&accel_config_data,1,100); //Accel Config Reg 0x1C, +-4g

}

void MPU6050_I2C_Read(){
	  HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS,accel_xout_h_address,1,accel_data_x,6,100);
	  HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS,gyro_xout_h_address,1,gyro_data_x,6,100);
	  HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS,temp_out_h,1,temp_out,2,100);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	     MPU6050_I2C_Read();
         uint16_t accel_x_val = (int16_t)(accel_data_x[0]<<8 | accel_data_x[1]); //fill 16-bit reg with raw values
         uint16_t accel_y_val = (int16_t)(accel_data_x[2]<<8 | accel_data_x[3]);
         uint16_t accel_z_val = (int16_t)(accel_data_x[4]<<8 | accel_data_x[5]);

         	  a_x = (float) accel_x_val / 8192.0 ;   //for +-4g LSB values
         	  a_y = (float) accel_y_val / 8192.0 ;
         	  a_z = (float) accel_z_val / 8192.0 ;

         uint16_t gyro_x_val = (int16_t)(gyro_data_x[0]<<8 | gyro_data_x[1]); //fill 16-bit reg with raw values
         uint16_t gyro_y_val = (int16_t)(gyro_data_x[2]<<8 | gyro_data_x[3]);
         uint16_t gyro_z_val = (int16_t)(gyro_data_x[4]<<8 | gyro_data_x[5]);

         	  g_x = (float) gyro_x_val / 65.5 ;   //for +-65.5 LSB' values
         	  g_y = (float) gyro_y_val / 65.5 ;
         	  g_z = (float) gyro_z_val / 65.5 ;

         uint16_t temp_out_val = (uint16_t)(temp_out[0]<<8 | temp_out[1]);
         	  temp = ((float) temp_out_val /340.0) + 36.53;
	       //char message_acc[50];
	       //sprintf(message_acc,"Acceleration of x:%.2f, Acceleration of y:%.2f, Acceleration of z:%.2f",a_x,a_y,a_z);
	       //HAL_UART_Transmit(&huart1,(uint8_t*)message_acc,sizeof(message_acc),10);

	       //char message_gyro[50];
	       //sprintf(message_gyro,"Angular speed of x:%.2f, Angular speed of y:%.2f, Angular speed of z:%.2f",g_x,g_y,g_z);
	       //HAL_UART_Transmit(&huart1,(uint8_t*)message_gyro,sizeof(message_gyro),10);

	       //printf("Acceleration of x:%.2f, Acceleration of y:%.2f, Acceleration of z:%.2f",a_x,a_y,a_z);
	       //printf("Angular speed of x:%.2f, Angular speed of y:%.2f, Angular speed of z:%.2f",g_x,g_y,g_z);


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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
