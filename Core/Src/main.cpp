/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MPU6050_ADDR 0x68 << 1
#define CONVERT_G_TO_MS2 9.80665f

void MPU6050_Init(void) {
    uint8_t check, data;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 100);
    if (check == 0x68) {
        data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);  // Wake up
    }
}

void MPU6050_Read_Accel(float *ax, float *ay, float *az) {
    uint8_t rec[6];
    int16_t raw_x, raw_y, raw_z;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, rec, 6, 100);
    raw_x = (int16_t)(rec[0] << 8 | rec[1]);
    raw_y = (int16_t)(rec[2] << 8 | rec[3]);
    raw_z = (int16_t)(rec[4] << 8 | rec[5]);

    *ax = ((float)raw_x / 16384.0f) * CONVERT_G_TO_MS2;
    *ay = ((float)raw_y / 16384.0f) * CONVERT_G_TO_MS2;
    *az = ((float)raw_z / 16384.0f) * CONVERT_G_TO_MS2;
}

extern "C" int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

extern "C" void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string, fmt, argp)) // build string
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}

void ei_printf(const char *format, ...) {
    va_list myargs;
    va_start(myargs, format);
    vprint(format, myargs);
    va_end(myargs);
}

//// paste the raw features here
//static const float features[] = {
//		2.5400, 3.4500, 1.6100, 0.9800, 7.6900, 0.4700, 0.6900, 15.3600, -1.9900, -4.2600, 19.6100, -1.4800, -14.6100, 19.6100, -0.5400, -6.5900, 19.6100, 0.6100, 2.0400, 19.6100, -3.6200, 8.6600, 10.7000, -7.3500, 11.9600, -1.8700, -7.0200, 13.3200, -19.5600, -7.7900, 10.6400, -19.6100, -9.8800, 4.3600, -19.6100, -7.0600, 3.6500, -19.6100, -0.4600, 4.3100, -19.6100, -7.3800, 4.1700, -19.6100, -12.8900, 3.8100, -19.6100, -10.2000, 4.4500, -15.5600, -3.8600, 4.0300, -7.4400, 1.8700, 2.3900, 2.4700, 3.7200, 0.6900, 9.0200, 2.4700, -0.8100, 19.6100, -2.1400, -11.3100, 19.6100, 1.7400, -14.5700, 19.6100, 0.9700, -1.6300, 19.6100, -1.0200, 8.8300, 15.0200, -5.4700, 13.2800, -0.0700, -8.3200, 16.5400, -16.9600, -8.9000, 13.1600, -19.6100, -12.9200, 5.9800, -19.6100, -7.1200, 4.0400, -19.6100, 1.4100, 4.3300, -19.6100, -5.9500, 4.2500, -19.6100, -11.8900, 3.2900, -17.0800, -9.4000, 2.9700, -9.4100, -3.1800, 2.1100, -0.1600, 1.7900, 0.6500, 6.0600, 2.4900, -2.0400, 17.6900, 0.5600, -8.1500, 19.6100, -0.4700, -17.0200, 19.6100, 3.1800, -7.7700, 19.6100, 4.8800, 1.4000, 14.9200, -1.7200, 6.9700, 4.6300, -6.6900, 11.0800, -11.4100, -8.0400, 13.1300, -19.6100, -13.6800, 5.0700, -19.6100, -12.2600, 2.6400, -19.6100, 3.4900, 3.8100, -19.6100, -3.0000, 4.8400, -19.6100, -10.2900, 5.7200, -17.8500, -6.9800, 5.4000, -5.8600, 0.6000
//};

#define AXIS_COUNT 3
#define SAMPLE_FREQ_HZ 50
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_FREQ_HZ)
#define FRAME_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE  // 150
#define STRIDE_MS 181

float features[FRAME_SIZE];
uint32_t last_inference_time = 0;
size_t collected_samples = 0;


int get_feature_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
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
  MPU6050_Init();
  printf("Started\r\n");
  setvbuf(stdout, NULL, _IONBF, 0);
  float ax, ay, az;
  uint32_t t0 = HAL_GetTick();
  last_inference_time = HAL_GetTick();

  signal_t signal;
  signal.total_length = sizeof(features) / sizeof(features[0]);
  signal.get_data = &get_feature_data;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    // 1. Отримати нові дані
	    MPU6050_Read_Accel(&ax, &ay, &az);

	    // 2. Зрушити масив вліво (на 3 елементи)
	    memmove(&features[0], &features[AXIS_COUNT], sizeof(float) * (FRAME_SIZE - AXIS_COUNT));

	    // 3. Додати новий семпл в кінець
	    features[FRAME_SIZE - 3] = ax;
	    features[FRAME_SIZE - 2] = ay;
	    features[FRAME_SIZE - 1] = az;

	    // 4. Очікувати наступний семпл (20 мс = 50 Гц)
	    HAL_Delay(SAMPLE_PERIOD_MS);

	    // 5. Запуск класифікації кожні STRIDE_MS
	    if (HAL_GetTick() - last_inference_time >= STRIDE_MS) {
	        last_inference_time = HAL_GetTick();

	        signal.total_length = FRAME_SIZE;
	        signal.get_data = [](size_t offset, size_t length, float *out_ptr) {
	            memcpy(out_ptr, features + offset, length * sizeof(float));
	            return 0;
	        };

	        ei_impulse_result_t result = { 0 };
	        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);
	        ei_printf("run_classifier returned: %d\n", res);

	        ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
	            result.timing.dsp, result.timing.classification, result.timing.anomaly);
	        ei_printf("[");
	        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
	            ei_printf_float(result.classification[ix].value);
	            if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) ei_printf(", ");
	        }
	        #if EI_CLASSIFIER_HAS_ANOMALY == 1
	            ei_printf(", ");
	            ei_printf_float(result.anomaly);
	        #endif
	        ei_printf("]\n\n");
      }

//      HAL_Delay(980);

//	MPU6050_Read_Accel(&ax, &ay, &az);
//
//	// зрушення і вставка
//	memmove(&features[0], &features[AXIS_COUNT], sizeof(float) * (FRAME_SIZE - AXIS_COUNT));
//	features[FRAME_SIZE - 3] = ax;
//	features[FRAME_SIZE - 2] = ay;
//	features[FRAME_SIZE - 1] = az;
//
//	HAL_Delay(SAMPLE_PERIOD_MS);
//
//	// лічильник семплів (до 50)
//	if (collected_samples < FRAME_SIZE) {
//		collected_samples += AXIS_COUNT;
//		continue;  // ще не готові
//	}
//
//	if (HAL_GetTick() - last_inference_time >= STRIDE_MS) {
//		last_inference_time = HAL_GetTick();
//
//		signal.total_length = FRAME_SIZE;
//		signal.get_data = [](size_t offset, size_t length, float *out_ptr) {
//			memcpy(out_ptr, features + offset, length * sizeof(float));
//			return 0;
//		};
//
//		ei_impulse_result_t result = { 0 };
//		EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);
//
//		ei_printf("run_classifier returned: %d\n", res);
//		ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
//			result.timing.dsp, result.timing.classification, result.timing.anomaly);
//		ei_printf("[");
//		for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
//			ei_printf_float(result.classification[ix].value);
//			if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) ei_printf(", ");
//		}
//		#if EI_CLASSIFIER_HAS_ANOMALY == 1
//			ei_printf(", ");
//			ei_printf_float(result.anomaly);
//		#endif
//		ei_printf("]\n\n");
//	}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
