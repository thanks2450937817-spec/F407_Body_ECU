/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */

/*
 * CAN1硬件连接（STM32F407VET6 + SN65HVD230 + CANable）：
 *
 *   STM32 PB9 / CAN1_TX  -> SN65HVD230 D（也可能标为TXD/CTX）
 *   STM32 PB8 / CAN1_RX  <- SN65HVD230 R（也可能标为RXD/CRX）
 *   STM32 3.3 V          -> SN65HVD230 VCC
 *   STM32 GND            <-> SN65HVD230 GND <-> CANable GND
 *   SN65HVD230 CANH      <-> CANable CANH
 *   SN65HVD230 CANL      <-> CANable CANL
 *
 * CAN总线两端各接一个120欧终端电阻。断电后在CANH与CANL之间测量，
 * 两个120欧并联应约为60欧。CANable当前使用SLCAN、COM17、500 kbit/s。
 * 注意：CAN_TX/RX是单端逻辑信号，CANH/CANL才是差分总线信号，
 * STM32不能跳过收发器直接连接CANH/CANL。
 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /*
   * CAN1位时序依据APB1外设时钟PCLK1 = 40 MHz：
   *
   *   时间量子频率 = 40 MHz / Prescaler(5) = 8 MHz
   *   1 TQ时间     = 1 / 8 MHz = 125 ns
   *   每位TQ数     = SyncSeg(1) + BS1(13) + BS2(2) = 16 TQ
   *   每位时间     = 16 * 125 ns = 2 us
   *   CAN波特率    = 1 / 2 us = 500 kbit/s
   *   采样点       = (1 + 13) / 16 = 87.5%
   *
   * CAN_MODE_NORMAL表示真实总线正常模式，需要其他节点（CANable）ACK；
   * AutoRetransmission开启后，未收到ACK的帧会由硬件自动重发。
   */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /*
   * MSP初始化负责CAN1所依赖的底层资源：
   * - 打开CAN1和GPIOB外设时钟；
   * - 将PB8/PB9切换为复用功能AF9，而不是普通GPIO；
   * - 打开CAN1_RX0中断，使FIFO0来帧后CPU能及时读取。
   * 实际收发方向：PB9发送到收发器D，PB8从收发器R接收。
   */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
