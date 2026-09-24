#ifndef APP_CAN_APP_H
#define APP_CAN_APP_H

#include "stm32f4xx_hal.h"

/*
 * CAN应用层公开接口。
 * 底层CAN1的时钟、PB8/PB9和NVIC配置位于Core/Src/can.c；
 * 本模块负责过滤器、收发协议、命令分发和周期心跳。
 */

/**
  * @brief  初始化CAN应用层：过滤器、帧头、CAN启动和接收通知
  * @param  hcan CAN外设句柄
  * @retval HAL_OK表示初始化成功，其他值表示失败
  */
HAL_StatusTypeDef CAN_App_Init(CAN_HandleTypeDef *hcan);

/**
  * @brief  处理接收命令，并按周期发送ECU心跳
  * @retval HAL_OK表示处理正常，其他值表示应答发送失败
  */
HAL_StatusTypeDef CAN_App_Process(void);



#endif /* APP_CAN_APP_H */
