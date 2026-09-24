//
// Created by admin on 2026/9/17.
//

#ifndef F407_CAN_BOOTLOADER_BOOTLOADER_CAN_H
#define F407_CAN_BOOTLOADER_BOOTLOADER_CAN_H

#include "stm32f4xx_hal.h"


HAL_StatusTypeDef Bootloader_CAN_Init(
    CAN_HandleTypeDef *hcan);

HAL_StatusTypeDef Bootloader_CAN_Process(void);

HAL_StatusTypeDef Bootloader_CAN_SendSingleFrame(
    const uint8_t *uds_data);

HAL_StatusTypeDef Bootloader_CAN_SendNegativeResponse(uint8_t service_id,uint8_t NRC);









#endif //F407_CAN_BOOTLOADER_BOOTLOADER_CAN_H