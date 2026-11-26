/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "har_config.h"
#include "uart_io.h"
#include "mpu6050.h"
#include "math.h"
#include "preprocessing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GYRO_CALIBRATE_SAMPLES_CNT		200U
#define IMU_DROP_LIMIT_CNT 						5U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t imu_tick = 0,
									imu_served = 0,
									imu_drop_cnt = 0;
mpu6050_t imu;
static float gyro_bias[MPU6050_AXES] = {0.0f},
						 last_sample[RAW_CHANNELS] = {0};
uint32_t imu_err_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void IMU_Service(void);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
	uint8_t who = 0;
	
	#ifdef DEBUG_PRINT_RAW
		uart_printf("Mode: IMU_DEBUG\r\n");
	#endif
	
	while(MPU6050_Init(&hi2c1, &who) != HAL_OK){
		uart_printf("MPU not found\r\n");
		HAL_Delay(500);
	}
	uart_printf("WHO_AM_I = 0x%02X\r\n", who);
	
	uart_printf("Calibrating gyro - put the sensor STILL on the table...\r\n");
	if(MPU6050_CalibrateGyro(&hi2c1, GYRO_CALIBRATE_SAMPLES_CNT, gyro_bias) != HAL_OK)
			uart_printf("Gyro calibration FAILED.\r\n");
	
	uart_printf("gyro_bias bias [dps]: %.3f %.3f %.3f\r\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
	
	imu_served = imu_tick;
	HAL_TIM_Base_Start_IT(&htim14);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		IMU_Service();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void IMU_Service(void){
	uint32_t now = imu_tick;
	uint32_t pending = now - imu_served;
	
	if(pending == 0U) return;
	imu_served = now;
	if(pending > IMU_DROP_LIMIT_CNT){
		imu_drop_cnt += pending - IMU_DROP_LIMIT_CNT;
		pending = IMU_DROP_LIMIT_CNT;
	}
	
	if(MPU6050_ReadAll(&hi2c1, &imu) == HAL_OK){
		const float *acc[MPU6050_AXES] = {&imu.ax, &imu.ay, &imu.az};
		const float *gyro[MPU6050_AXES] = {&imu.gx, &imu.gy, &imu.gz};
		
		for(uint32_t i = 0; i < MPU6050_AXES; i++){
			last_sample[i] = *acc[i];
			last_sample[MPU6050_AXES + i] = *gyro[i] - gyro_bias[i];
		}
	}
	else{
		imu_err_cnt++;
	}
	
	for(uint32_t i = 0; i < pending; i++) PP_PushSample(last_sample);
	
	#ifdef DEBUG_PRINT_RAW
		static uint32_t dbg = 0;
		if(++dbg >= 50U){
			dbg = 0;
			uart_printf(
				"RAW, %.3f, %.3f, %.3f, %.2f, %.2f, %.2f, err=%lu, drop=%lu\r\n",
				last_sample[0], last_sample[1], last_sample[2],
				last_sample[3], last_sample[4], last_sample[5],
				(unsigned long)imu_err_cnt, (unsigned long)imu_drop_cnt
			);
		}
	#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM14) imu_tick++;
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
#ifdef USE_FULL_ASSERT
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
