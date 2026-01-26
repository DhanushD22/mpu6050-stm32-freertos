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
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 		0x68 // AD0 tied to GND
#define WHO_AM_I_REG 		0x75
#define PWR_MGMT_1_REG 		0x6B
#define ACCEL_XOUT_H 		0x3B
#define GYRO_XOUT_H 		0x43
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

QueueHandle_t imuQueue;
QueueHandle_t orientQueue;
SemaphoreHandle_t uartMutex;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

// IMU Task - Read raw sensor data
void IMU_Task(void *argument)
{
    imu_data_t imu;
    uint8_t accel_raw[6], gyro_raw[6];

    for (;;)
    {
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1,
                             ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                             accel_raw, 6, 100) != HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1,
                             GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                             gyro_raw, 6, 100) != HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        imu.ax_g = ((int16_t)(accel_raw[0] << 8 | accel_raw[1])) / 16384.0f;
        imu.ay_g = ((int16_t)(accel_raw[2] << 8 | accel_raw[3])) / 16384.0f;
        imu.az_g = ((int16_t)(accel_raw[4] << 8 | accel_raw[5])) / 16384.0f;

        imu.gx_dps = ((int16_t)(gyro_raw[0] << 8 | gyro_raw[1])) / 131.0f;
        imu.gy_dps = ((int16_t)(gyro_raw[2] << 8 | gyro_raw[3])) / 131.0f;
        imu.gz_dps = ((int16_t)(gyro_raw[4] << 8 | gyro_raw[5])) / 131.0f;

        xQueueSend(imuQueue, &imu, 0);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Orinetation task - calculating roll and pitch

void Orientation_Task(void *argument)
{
    imu_data_t imu;
    orientation_t orient;

    for (;;)
    {
        if (xQueueReceive(imuQueue, &imu, portMAX_DELAY) == pdTRUE)
        {
            float roll  = atan2f(imu.ay_g, imu.az_g);
            float pitch = atan2f(-imu.ax_g,
                                  sqrtf(imu.ay_g * imu.ay_g +
                                        imu.az_g * imu.az_g));

            orient.roll_deg  = roll  * 57.2957795f;
            orient.pitch_deg = pitch * 57.2957795f;

            xQueueSend(orientQueue, &orient, 0);
        }
    }
}

// Logger task - print on serial terminal

void Logger_Task(void *argument)
{
    orientation_t orient;

    for (;;)
    {
        if (xQueueReceive(orientQueue, &orient, portMAX_DELAY) == pdTRUE)
        {
        	if (xSemaphoreTake(uartMutex, portMAX_DELAY))
        	{
        	    printf("ROLL=%.2f deg | PITCH=%.2f deg\r\n",
        	           orient.roll_deg,
        	           orient.pitch_deg);
        	    xSemaphoreGive(uartMutex);
        	}
        }
    }
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Scanning I2C bus...\r\n");

  for (uint8_t addr = 1; addr < 127; addr++)
  {
	  if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK)
	  {
		  printf("Found device at 0x%02X\r\n", addr);
	  }
  }

  printf("UART ALIVE\r\n");

  uint8_t data = 0x00;
  uint8_t who_am_i = 0;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1,
                    PWR_MGMT_1_REG,
                    I2C_MEMADD_SIZE_8BIT,
                    &data, 1, 100);
  HAL_Delay(100);

  HAL_I2C_Mem_Read(
       &hi2c1,
       MPU6050_ADDR << 1,
       WHO_AM_I_REG,
       1,
       &who_am_i,
       1,
       HAL_MAX_DELAY
   );
  printf("WHO_AM_I = 0x%02X\r\n", who_am_i);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

//  /* Initialize led */
//  BSP_LED_Init(LED_GREEN);
//
//  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
//  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
//
//  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
//  BspCOMInit.BaudRate   = 115200;
//  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
//  BspCOMInit.StopBits   = COM_STOPBITS_1;
//  BspCOMInit.Parity     = COM_PARITY_NONE;
//  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
//  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
//  {
//    Error_Handler();
//  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
