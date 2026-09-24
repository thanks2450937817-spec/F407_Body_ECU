//
// Created by admin on 2026/9/18.
//

#ifndef F407_CAN_BOOTLOADER_BOOTLOADER_UDS_H
#define F407_CAN_BOOTLOADER_BOOTLOADER_UDS_H

#include "stm32f4xx_hal.h"
#include "Bootloader_can.h"
#define UDS_REQUEST_DOWNLOAD         0x34U//下载请求
#define UDS_TRANSFER_DATA            0x36U//发送数据
#define UDS_REQUEST_TRANSFER_EXIT    0x37U//结束下载
#define UDS_SERVICE_ECU_RESET                  0x11U
#define UDS_ECU_RESET_HARD_RESET               0x01U//11子服务

HAL_StatusTypeDef Bootloader_UDS_Process(
    const uint8_t *message,
    uint16_t message_length);















#endif //F407_CAN_BOOTLOADER_BOOTLOADER_UDS_H