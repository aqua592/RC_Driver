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
#include "adc.h"
#include "hrtim.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
float dutyCycle = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //  InitParameter(&INV,
  //  			//		SPM
  //  			0.2034f,		// Rs
  //  			2.941e-3,	// Ld
  //  			2.941e-3,	// Lq
  //  			0.07796f,	// Lamf
  //  			4.f,		// PP
  //  			0.748e-3,	// Jm
  //  			1.e-4,	// Bm
  //  			1.f,		// Idsr_align
  //  			4.5f,		// Is_rated  // 12.445
  //  			3.0f,		// Is_limit
  //  			3000.f,	// Wrpm_rated
  //  			0.776f	// Te_rated
  //  );

    //InitParameter(&INV, 5.87e-3, 0.61e-6, 0.61e-6, 728e-6, 4.f, 1e-6, 1e-6, 2.f, 10.f, 20.f, 35000.f, 1e-2); // Velineon 3500
    InitParameter(&INV, 19e-3, 3.2e-6, 3.2e-6, 2e-3, 1.f, 1e-6, 1e-6, 2.f, 50.f, 50.f, 10000.f, 3); // XERUN 13.5T
  //  InitParameter(&INV, 7.85e-3, 0.35e-6, 0.35e-6, 0.711e-3, 4.f, 1e-6, 1e-6, 2.f, 40.f, 40.f, 10000.f, 3); // CASTLE 13.5T

    InitSpeedController(&INV, PI2*25.f, 0.707f);		//PI2*2.	0.707
  //  InitFluxWeakeningController(&INV, 0.0001f, 1.f);
  //  InitTorqueController(&INV);
    InitCurrentController(&INV, PI2*1000.f);	//PI2*200.

    Init_Spd_PLL(&INV, PI2*20.f);
    initExtended_Sensorless_Synchronous_Frame(&EXT_1, PI2*200.f, INV.Rs, INV.Ld, INV.Lq);


    //HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // Idc

    HAL_ADC_Start_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    //HAL_ADC_Start_DMA(&hadc1, adc1Val, 4);
  //  HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);

    HAL_TIMEx_HallSensor_Start_IT(&htim3);

    FLAG.READY = 1;
    FLAG.FAULT = 0;       // 에러 상태 해제
    FLAG.DUTY_TEST = 1;   // DUTY_TEST 모드 진입 (50% 듀티 출력)

    HAL_TIM_Base_Start_IT(&htim2); // TIM2 10kHz 제어 루프 시작 (이 안에서 Control() 호출)

  /* USER CODE END 2 */

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
