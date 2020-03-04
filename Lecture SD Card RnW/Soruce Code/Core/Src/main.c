/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LSM6DS33_DEV_ADD	0x6A
#define I2C_MEMADD_SIZE_8BIT	0x00000001U
#define LINEAR_ACCEL_RANGE 2
#define GYRO_RANGE 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
unsigned char LSMwrite[6];
unsigned char temperatureData[2];
unsigned char linearAccelData[6];
unsigned char angularRateData[6];

HAL_StatusTypeDef i2cStats;
float temperature;
float linearAccel[3];
float angularRate[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float calcLinearAccel(int16_t, int);
float calcGyro(int16_t twosCompAdjustedInput, int gyroRange);
int16_t twosComp2ByteToSignedInt(unsigned char *);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_StateTypeDef state;
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  LSMwrite[0] = 0b01010000; //CTRL1_XL 2g, 400Hz, normal mode
  i2cStats = HAL_I2C_Mem_Write(&hi2c1, LSM6DS33_DEV_ADD << 1, 0x10, I2C_MEMADD_SIZE_8BIT, LSMwrite, 1, 100); // reset FUNC_CFG_ACCESS
  HAL_Delay(20);

  LSMwrite[0] = 0b01011100; //CTRL2_XL 2000dps, gyro full scale off, normal mode
  i2cStats = HAL_I2C_Mem_Write(&hi2c1, LSM6DS33_DEV_ADD << 1, 0x11, I2C_MEMADD_SIZE_8BIT, LSMwrite, 1, 100); // reset FUNC_CFG_ACCESS
  HAL_Delay(20);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(200);
	CDC_Transmit_FS(buff, sizeof(buff));

	state=HAL_I2C_GetState(&hi2c1);
	if(state==HAL_I2C_STATE_READY||state==HAL_I2C_STATE_BUSY_RX){
		//Read from temperature register
		i2cStats = HAL_I2C_Mem_Read(&hi2c1, LSM6DS33_DEV_ADD << 1, 0x20, I2C_MEMADD_SIZE_8BIT, temperatureData, 2, 100); //reading two byte from temperature register
		temperature = twosComp2ByteToSignedInt(temperatureData);
		temperature = temperature / 16.0 + 25.0;
		HAL_Delay(20); //just to be safe

		//Read from Acceleration register
		i2cStats = HAL_I2C_Mem_Read(&hi2c1, LSM6DS33_DEV_ADD << 1, 0x28, I2C_MEMADD_SIZE_8BIT, linearAccelData, 6, 100); //reading from x accel LSB
		linearAccel[0] = calcLinearAccel(twosComp2ByteToSignedInt(linearAccelData), LINEAR_ACCEL_RANGE);
		linearAccel[1] = calcLinearAccel(twosComp2ByteToSignedInt(&linearAccelData[2]), LINEAR_ACCEL_RANGE);
		linearAccel[2] = calcLinearAccel(twosComp2ByteToSignedInt(&linearAccelData[4]), LINEAR_ACCEL_RANGE);
		HAL_Delay(20); //just to be safe

		i2cStats = HAL_I2C_Mem_Read(&hi2c1, LSM6DS33_DEV_ADD << 1, 0x22, I2C_MEMADD_SIZE_8BIT, angularRateData, 6, 100); //reading from x accel LSB
		angularRate[0] = calcGyro(twosComp2ByteToSignedInt(angularRateData), GYRO_RANGE);
		angularRate[1] = calcGyro(twosComp2ByteToSignedInt(&angularRateData[2]), GYRO_RANGE);
		angularRate[2] = calcGyro(twosComp2ByteToSignedInt(&angularRateData[4]), GYRO_RANGE);
	}


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
float calcLinearAccel(int16_t twosCompAdjustedInput, int accelConvertionRatio){
	return (float)twosCompAdjustedInput * 0.061 * (accelConvertionRatio >> 1) / 1000;
}

float calcGyro(int16_t twosCompAdjustedInput, int gyroRange){
	uint8_t gyroRangeDivisor = gyroRange / 125;
	if ( gyroRange == 245 ) {
		gyroRangeDivisor = 2;
	}

	float output = (float)twosCompAdjustedInput * 4.375 * (gyroRangeDivisor) / 1000;
	return output;
}

//Expect two byte char array
int16_t twosComp2ByteToSignedInt(unsigned char * input){
	if (input == NULL) return -1;
	return (int16_t) ((signed char) input[1] << 8) | input[0];
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
