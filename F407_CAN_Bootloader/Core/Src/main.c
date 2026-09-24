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
#include "Bootloader_can.h"
#include "Bootloader_uds.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 应用程序入口函数类型：无参数、无返回值 */
typedef void (*ApplicationEntryFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APPLICATION_START_ADDRESS    0x08010000U
#define APPLICATION_END_ADDRESS      0x08060000U
#define APPLICATION_RAM_START        0x20000000U
#define APPLICATION_RAM_END          0x20020000U
/* 写入RTC备份寄存器的升级请求标志，十六进制对应ASCII字符“BOOT” */
#define BOOT_REQUEST_MAGIC           0x424F4F54U

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
static uint8_t Bootloader_IsApplicationValid(void);
static void Bootloader_JumpToApplication(void);
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
  __HAL_RCC_PWR_CLK_ENABLE();//开启PWR外设时钟
  HAL_PWR_EnableBkUpAccess();//关闭备份域写保护，之后才允许写入备份寄存器
  uint8_t update_requested = 0U;
  if (RTC->BKP0R == BOOT_REQUEST_MAGIC) {//读取备份寄存器
    RTC->BKP0R = 0U;
    update_requested = 1U;
  }
  HAL_PWR_DisableBkUpAccess();//恢复备份域写保护，之后不允许写入备份寄存器
  /* 如果应用程序有效，就将控制权交给应用程序 */
  if (update_requested == 0U) {
     if (Bootloader_IsApplicationValid() != 0U)
    {
      Bootloader_JumpToApplication();
    }
  }


  /*
   * 如果应用程序无效，跳转函数不会执行，
   * 程序会继续向下初始化Bootloader。
   */
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
  if (Bootloader_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Bootloader_CAN_Process();
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
static void Bootloader_JumpToApplication(void)
{
  /* 读取应用程序向量表中的初始栈地址 */
  uint32_t initial_stack_pointer =
      *(const uint32_t *)APPLICATION_START_ADDRESS;

  /* 读取应用程序向量表中的Reset_Handler地址 */
  uint32_t reset_handler_address =
      *(const uint32_t *)(APPLICATION_START_ADDRESS + 4U);

  /* 把整数地址转换为可以调用的函数指针 */
  ApplicationEntryFunction application_entry =
      (ApplicationEntryFunction)reset_handler_address;
  /* 将中断向量表切换到应用程序所在地址 */
  /*
  *SCB：CMSIS 提供的宏，指向 Cortex-M4 的系统控制寄存器。
   VTOR：中断向量表地址寄存器。
   ->：通过指针访问结构体成员，SCB->VTOR 等价于 (*SCB).VTOR。
   */
  SCB->VTOR = APPLICATION_START_ADDRESS;
  /* 等待前面的寄存器和内存操作全部完成 */
  __DSB();//__DSB()：Data Synchronization Barrier，数据同步屏障。等待前面的内存和寄存器访问完成。

  /* 清空CPU已经预取的指令，按照新的系统状态继续执行 */
  __ISB();//Instruction Synchronization Barrier，指令同步屏障。丢弃已经预取的旧指令，让 CPU 按修改后的系统状态继续取指。
  /* 切换到应用程序自己的主栈 */
  __set_MSP(initial_stack_pointer);

  /* 进入应用程序的Reset_Handler */
  application_entry();

  /* 正常情况下Reset_Handler不会返回 */
  while (1)
  {
  }
}
static uint8_t Bootloader_IsApplicationValid(void)//检测是否存在有效应用程序
{
  //1.获取初始栈地址
  uint32_t initial_stack_pointer =
      *(const uint32_t *)APPLICATION_START_ADDRESS;
  //2.判断初始栈地址是否位于RAM内
  if (((initial_stack_pointer >= APPLICATION_RAM_START) &&
      (initial_stack_pointer <= APPLICATION_RAM_END)) != 1)
  {
    return 0U;
  }
  //3.读取Reset_Handler 地址
  uint32_t reset_Handler_Address =
    *(const uint32_t *)(APPLICATION_START_ADDRESS + 4U);
  //4.判断Reset_Handler最后一位是否为1
  if ((reset_Handler_Address & 1U) != 1) {
    return 0U;
  }
  //5.获取真正的代码地址
  uint32_t real_code_address = reset_Handler_Address & ~1U;
  //6.检查代码地址是否在应用程序范围之内
  if ((real_code_address < APPLICATION_END_ADDRESS &&
    real_code_address >= APPLICATION_START_ADDRESS) != 1 ) {
    return 0U;
  }
  //7.全部通过返回1
  return 1U;
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
