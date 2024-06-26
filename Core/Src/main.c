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
#include <math.h>
#include "OLED_SSD1306.h"
#include "IMU_6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// MPU6050_IMU structure
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)
#define SENSE 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
//HAL_StatusTypeDef Oled_State, IMU_State;
MPU6050_t My_MPU;

//float AccRoll = 0, AccPitch = 0;
//float GyroRoll = 0, GyroPitch = 0;
float FltrRoll = 0, FltrPitch = 0;

uint32_t start_time = 0;
//uint32_t end_time = 0;
float delta_time = 0;
float calibrated_Ax = 0, calibrated_Ay = 0, calibrated_Az = 0;
float calibrated_Gx = 0, calibrated_Gy = 0, calibrated_Gz = 0;
float Cal_GyroX = 0, Cal_GyroY = 0, Cal_GyroZ = 0;
float Cal_AccX = 0, Cal_AccY = 0, Cal_AccZ = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void IMU_Calibrate(I2C_HandleTypeDef *i2c, MPU6050_t *IMU);
void Draw_Diamond(void);
void Draw_Square(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t i = 0;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	HAL_Delay(100);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADD, 1, 1000) == HAL_OK
			&& HAL_I2C_IsDeviceReady(&hi2c1, IMU_ADD, 1, 1000) == HAL_OK) {

		OLED_Setup(&hi2c1);
		IMU_Setup(&hi2c1);
		OLED_Print(&hi2c1, "hold_still", 2, 0);
		OLED_Print(&hi2c1, "wait...", 4, 20);
		IMU_Calibrate(&hi2c1, &My_MPU);
		while (1) {

			delta_time = HAL_GetTick() - start_time;
			delta_time /= 1000; //in seconds
			start_time = HAL_GetTick();

			IMU_ReadAll(&hi2c1, &My_MPU);

			calibrated_Ax = My_MPU.Ax - Cal_AccX;
			calibrated_Ay = My_MPU.Ay - Cal_AccY;
			calibrated_Az = My_MPU.Az - Cal_AccZ;

			calibrated_Gx = My_MPU.Gx - Cal_GyroX;
			calibrated_Gy = My_MPU.Gy - Cal_GyroY;
			calibrated_Gz = My_MPU.Gz - Cal_GyroZ;

			/*
			 * calculation in detail
			 *
			 *AccPitch = -(atan(calibrated_Ax / calibrated_Az)) * RAD_TO_DEG;
			 *AccRoll = (atan(calibrated_Ay / calibrated_Az)) * RAD_TO_DEG;
			 *GyroRoll = GyroRoll + (calibrated_Gx * delta_time);
			 *GyroPitch = GyroPitch + (calibrated_Gy * delta_time);
			 *
			 */

			/*Performing Complementery sensor fusion of data from Gyroscope and Accelerometer*/
			FltrRoll = (((atan(calibrated_Ay / calibrated_Az)) * RAD_TO_DEG)
					* 0.15) + ((FltrRoll + calibrated_Gx * delta_time) * 0.85);
			FltrPitch = ((-(atan(calibrated_Ax / calibrated_Az)) * RAD_TO_DEG)
					* 0.15) + ((FltrPitch + calibrated_Gy * delta_time) * 0.85);

			if (((uint8_t) FltrRoll) == 0 && ((uint8_t) FltrPitch) == 0) {
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
			} else {
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
			}
			OLED_ClearRAMBuffer();
			Draw_Square();
			Draw_Diamond();
			OLED_UpdateScreen(&hi2c1);
			HAL_Delay(50);

			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
		}
	}

	else {
		//IMU or OLED not connected
		Error_Handler();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x1094102C;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void IMU_Calibrate(I2C_HandleTypeDef *i2c, MPU6050_t *IMU) {

	for (int i = 0; i < 2000; i++) {
		IMU_ReadAll(i2c, IMU);
		Cal_AccX += IMU->Ax;
		Cal_AccY += IMU->Ay;
		Cal_AccZ += IMU->Az;
		Cal_GyroX += IMU->Gx;
		Cal_GyroY += IMU->Gy;
		Cal_GyroZ += IMU->Gz;
	}

	//averaging all the readings
	Cal_AccX /= 2000;
	Cal_AccY /= 2000;
	Cal_AccZ /= 2000;
	Cal_AccZ -= 1; // subtracting 1g because G in Z axis should be 1 unlike X and Y which should be 0

	Cal_GyroX /= 2000;
	Cal_GyroY /= 2000;
	Cal_GyroZ /= 2000;
}

void Draw_Diamond(void) {
	OLED_SinglePixel(&hi2c1, -(uint8_t) (FltrRoll * SENSE) + 63,
			(uint8_t) (FltrPitch * SENSE) + 31, WHITE);	//center point
	OLED_SinglePixel(&hi2c1, -(uint8_t) (FltrRoll * SENSE) + 63,
			(uint8_t) (FltrPitch * SENSE) + 30, WHITE);
	OLED_SinglePixel(&hi2c1, -(uint8_t) (FltrRoll * SENSE) + 63,
			(uint8_t) (FltrPitch * SENSE) + 32, WHITE);
	OLED_SinglePixel(&hi2c1, -(uint8_t) (FltrRoll * SENSE) + 62,
			(uint8_t) (FltrPitch * SENSE) + 31, WHITE);
	OLED_SinglePixel(&hi2c1, -(uint8_t) (FltrRoll * SENSE) + 64,
			(uint8_t) (FltrPitch * SENSE) + 31, WHITE);
}

void Draw_Square(void) {
	OLED_SinglePixel(&hi2c1, 66, 28, WHITE);
	OLED_SinglePixel(&hi2c1, 60, 28, WHITE);
	OLED_SinglePixel(&hi2c1, 66, 34, WHITE);
	OLED_SinglePixel(&hi2c1, 60, 34, WHITE);
	OLED_SinglePixel(&hi2c1, 66, 31, WHITE);
	OLED_SinglePixel(&hi2c1, 60, 31, WHITE);
	OLED_SinglePixel(&hi2c1, 63, 34, WHITE);
	OLED_SinglePixel(&hi2c1, 63, 28, WHITE);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
