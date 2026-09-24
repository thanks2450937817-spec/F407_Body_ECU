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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "body_control.h"
#include "can_app.h"
#include "DTC_manager.h"
#include "Fault_injection.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 灯故障诊断每10 ms执行一次 */
#define DTC_CHECK_PERIOD_MS  10U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 保存上一次执行灯故障诊断的系统毫秒时间 */
static uint32_t last_dtc_check_ms = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * 当前工程时钟树（由下面的 SystemClock_Config() 配置）：
 *
 *   HSI 16 MHz
 *      -> PLLM = 8，得到 2 MHz
 *      -> PLLN = 80，得到 160 MHz
 *      -> PLLP = 2，得到 SYSCLK = 80 MHz
 *
 *   AHB  不分频：HCLK  = 80 MHz（CPU和存储器总线）
 *   APB1 2分频 ：PCLK1 = 40 MHz（CAN1使用该外设时钟）
 *   APB2 不分频：PCLK2 = 80 MHz
 *
 * CAN位时序的500 kbit/s计算依赖PCLK1=40 MHz；如果以后修改
 * 系统时钟或APB1分频，必须同时重新计算CAN的Prescaler/BS1/BS2。
 */

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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  /*
   * 应用初始化顺序：
   * 1. GPIO和CAN1已由CubeMX生成的函数完成底层初始化；
   * 2. BodyControl_Init()建立执行器的安全初始状态；
   * 3. DTC_Manager_Init()初始化故障记录；
   * 4. CAN_App_Init()再启动CAN和接收中断。
   */

  /* 初始化车身执行器，上电默认关闭PA7车灯（PA7输出高电平） */
  BodyControl_Init();

  /* 初始化DTC状态和100样本统计窗口 */
  DTC_Manager_Init();
  last_dtc_check_ms = HAL_GetTick();

  /* 初始化CAN应用层：过滤器、CAN启动、帧头和接收中断通知 */
  if (CAN_App_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 读取KEY1并更新故障注入状态；按下时模拟车灯无法点亮 */
    FaultInjection_ProcessButton();
    uint32_t current_time_ms = HAL_GetTick();

    if ((uint32_t)(current_time_ms - last_dtc_check_ms) >=
        DTC_CHECK_PERIOD_MS)
    {
      /* 只记录当前真正读取到的一次结果，不补算已经错过的采样 */
      last_dtc_check_ms = current_time_ms;
      DTC_Manager_ReportTestResult(BodyControl_CheckLampFault());
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * 主循环只处理非紧急任务：执行中断暂存的命令、发送周期心跳。
     * CAN接收中断只负责快速取走并保存数据，避免在中断中操作GPIO
     * 或等待发送邮箱，从而缩短中断占用时间。
     */
    if (CAN_App_Process() != HAL_OK)
    {
      Error_Handler();
    }

    /* 主循环约每1 ms运行一次，避免空转占满CPU */
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
  RCC_OscInitStruct.PLL.PLLN = 80;
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
