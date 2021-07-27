/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "i2c-lcd.h"
#include <stdio.h>
#include <string.h>
#include "UartRingbuffer.h"
#include "ESP_DATA_HANDLER.h"

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
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MPU6050_ADDR 0xD0 	//0x68 is the slave address. (0x68 << 1 = 0xD0)
							// it is done because slave address is 7 bit address
							// and for including RW bit for i2c communication we do left shift by 1
							// https://www.circuitbasics.com/basics-of-the-i2c-communication-protocol/ --- to get details of I2C communication
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48


// signed int is used because the sensor output is in 16bit 2's complement
// meaning 1 in MSB indicate -ve value, 0 in MSB indicate +ve value
int16_t acc_x_raw = 0;
int16_t acc_y_raw = 0;
int16_t acc_z_raw = 0;

int16_t gyro_x_raw = 0;
int16_t gyro_y_raw = 0;
int16_t gyro_z_raw = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

uint8_t aData[2];
uint8_t bData[2];
uint8_t cData[2];

// Initialisation
void MPU6050_Init(void)
{
	uint8_t check, Data;
	//  identitification of the device
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

	if(check == 104)	// if device present
	{
		// power management register initialisation
		Data = 0x00;	// Data is 0 to wakeup sensor
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, 1000);
	}

		//Data Rate set to 1kHz using SMPLRT_DIV register
		Data = 0x07;	//ie.,Enabling DLPF (register 26 - MPU6050 datasheet)
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, 1000);

		//configuring accelerometer
		//XA_ST=0 (means no self test), YA_ST=0, ZA_ST=0, AFS_SEL=0(means± 2g)
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, 1000);

		//configuring gyroscope
		//XG_ST=0 (means no self test), YG_ST=0, ZG_ST=0, FS_SEL=0(means ± 250 °/s)
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, 1000);
}


// Reading accelerometer raw values
void AccelerationRead(void)
{
//	uint8_t aData[2];
//	uint8_t bData[2];
//	uint8_t cData[2];

	// X, Y, Z values Memory read
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H, 1, aData, 2, 1000);
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, ACCEL_YOUT_H, 1, bData, 2, 1000);
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, ACCEL_ZOUT_H, 1, cData, 2, 1000);

	// sensor data is received one after the other
	// so the higher byte is left shifted by 8 places
	// the the higher and lower byte are combined to a single 16 bit sensor value

	acc_x_raw = (int16_t)(aData[0]<<8 | aData[1]);
	acc_y_raw = (int16_t)(bData[0]<<8 | bData[1]);
	acc_z_raw = (int16_t)(cData[0]<<8 | cData[1]);

	// Converting acceleration raw values in terms of 'g'
	Ax = acc_x_raw/16384.0;	// LSB Sensitivity = 16384 for full scale range of 2g. also 16384.0 is used for float values
	Ay = acc_y_raw/16384.0;
	Az = acc_z_raw/16384.0;

//	// Sending via Serial
	HAL_Delay(1);
//	uint8_t test= 65;
	HAL_UART_Transmit(&huart6, aData, 2, 1000);

}

// Reading  Gyroscope raw values
void GyroscopeRead(void)
{
	uint8_t gData[2];
	uint8_t hData[2];
	uint8_t iData[2];
//	int16_t gyro_x_raw;
//	int16_t gyro_y_raw;
//	int16_t gyro_z_raw;

	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, GYRO_XOUT_H, 1, gData, 2, 1000);
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, GYRO_YOUT_H, 1, hData, 2, 1000);
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, GYRO_ZOUT_H, 1, iData, 2, 1000);

	gyro_x_raw = (int16_t)(gData[0]<<8  | gData[1]);
	gyro_y_raw = (int16_t)(hData[0]<<8	| hData[1]);
	gyro_z_raw = (int16_t)(iData[0]<<8	| iData[1]);

	// converting raw gyroscope values to degree/sec
	Gx = gyro_x_raw/131.0;	// LSB Sensitivity = 131 LSB/°/s for full scale range of ± 250 °/s also 131.0 is used for float values
	Gy = gyro_y_raw/131.0;
	Gz = gyro_z_raw/131.0;
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
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  //ESP
//  ESP_Init("JioFiber 1", "Dexiatoungjio", "192.168.0.102");


 //LCD commands
  lcd_init();
  MPU6050_Init();
  lcd_send_string("Initialized");
  HAL_Delay(1);

  lcd_clear();
  lcd_send_cmd(0x80 | 0x5A);
  lcd_send_string("MPU6050");

  // ESP commands
// ESP_Init("JioFiber 1", "Dexiatoungjio", "192.168.29.212/24");

  char buf[4];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  AccelerationRead();
//	  GyroscopeRead();


	  // Print on LCD
	  lcd_send_cmd(0x80 | 0x00); //Row,Col ---> 1,1
	  lcd_send_string("Ax=");
	  sprintf(buf, "%.2f", Ax);
	  lcd_send_string(buf);
	  lcd_send_string("g ");


	  lcd_send_cmd(0x80 | 0x40); //2,1
	  lcd_send_string("Ay=");
	  sprintf(buf, "%.2f", Ay);
	  lcd_send_string(buf);
	  lcd_send_string("g ");

	  lcd_send_cmd(0x80 | 0x14); //3,1
	  lcd_send_string("Az=");
	  sprintf(buf, "%.2f", Az);
	  lcd_send_string(buf);
	  lcd_send_string("g ");

//	  lcd_send_cmd(0x80 | 0x0A); //1,11
//	  lcd_send_string("Gx=");
//	  sprintf(buf, "%.2f", Gx);
//	  lcd_send_string(buf);
//
//	  lcd_send_cmd(0x80 | 0x4A); //2,11
//	  lcd_send_string("Gy=");
//	  sprintf(buf, "%.2f", Gy);
//	  lcd_send_string(buf);
//
//	  lcd_send_cmd(0x80 | 0x1E); //3,11
//	  lcd_send_string("Gz=");
//	  sprintf(buf, "%.2f", Gz);
//	  lcd_send_string(buf);

	  HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
