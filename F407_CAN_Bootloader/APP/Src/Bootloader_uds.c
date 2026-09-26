//
// Created by admin on 2026/9/18.


// 从项目根目录运行PC工具：python C:\Users\admin\Desktop\Embedded\CAN\tools\can_test.py
#include "Bootloader_uds.h"

#define UDS_REQUEST_DOWNLOAD                   0x34U//下载请求
#define UDS_TRANSFER_DATA                      0x36U//发送数据
#define UDS_REQUEST_TRANSFER_EXIT              0x37U//结束下载
#define UDS_SERVICE_ECU_RESET                  0x11U
#define UDS_ECU_RESET_HARD_RESET               0x01U//11子服务表示请求 ECU 执行完整的软件复位
#define UDS_POSITIVE_RESPONSE_OFFSET           0x40U
#define UDS_DATA_FORMAT_NONE                   0x00U//不压缩不加密
#define UDS_ADDRESS_AND_LENGTH_FORMAT_4_BYTES  0x44U//地址占4字节，长度占4字节
#define UDS_WRITE_START_ADDRESS                0x08010000U
#define UDS_DATA_LENGTH                        0x00040000U

#define UDS_NRC_GENERAL_PROGRAMMING_FAILURE    0x72U//一般编程错误

// typedef HAL_StatusTypeDef(*bootloader_function)(const uint8_t *,uint16_t);
//
// typedef struct Bootloader_UDS {
//     uint8_t service_id;
//     bootloader_function function;
// }bootloader_uds;
//
// bootloader_uds bootloader_uds_service_table[] =
//     {
//
//
// };

static uint8_t download_in_progress = 0U;
static uint32_t download_start_address = 0U;
static uint32_t download_total_size = 0U;
static uint32_t download_received_size = 0U;
static uint8_t expected_block_sequence_counter = 1U;

static HAL_StatusTypeDef UDS_34_Process(const uint8_t *message,
    uint16_t message_length);
static HAL_StatusTypeDef UDS_36_Process(const uint8_t *message,
    uint16_t message_length);
static HAL_StatusTypeDef UDS_37_Process(const uint8_t *message,
    uint16_t message_length);
static uint32_t Bootloader_CalculateCRC32(
    uint32_t start_address,
    uint32_t data_length);
static HAL_StatusTypeDef UDS_11_Process(const uint8_t *message,
    uint16_t message_length);

static HAL_StatusTypeDef Bootloader_Flash_EraseApplication();

static HAL_StatusTypeDef Bootloader_Flash_WriteData(
    uint32_t current_download_address,
    const uint8_t *firmware_data,
    uint8_t firmware_data_length);

static uint32_t Bootloader_CalculateCRC32(
    uint32_t start_address,
    uint32_t data_length);

HAL_StatusTypeDef Bootloader_UDS_Process(
    const uint8_t *message,
    uint16_t message_length) {
    if (message == NULL) {
        return HAL_ERROR;
    }
    if (message_length < 1U) {
        return HAL_ERROR;
    }
    if (message[0] == UDS_REQUEST_DOWNLOAD) {
        return UDS_34_Process(message,message_length);
    }
    if (message[0] == UDS_TRANSFER_DATA) {
        return UDS_36_Process(message,message_length);
    }
    if (message[0] == UDS_REQUEST_TRANSFER_EXIT) {
        return UDS_37_Process(message,message_length);
    }
    if (message[0] == UDS_SERVICE_ECU_RESET) {
        return UDS_11_Process(message,message_length);
    }

    return HAL_ERROR;

}

HAL_StatusTypeDef UDS_34_Process(const uint8_t *message,
    uint16_t message_length) {
    if (message_length != 11U) {
        return HAL_ERROR;
    }
    if (message[1] != UDS_DATA_FORMAT_NONE) {
        return HAL_ERROR;
    }
    if (message[2] != UDS_ADDRESS_AND_LENGTH_FORMAT_4_BYTES) {
        return HAL_ERROR;
    }
    const uint32_t download_address = (message[3] << 24) + (message[4] << 16) + (message[5] << 8) + message[6];
    const uint32_t download_size = (message[7] << 24) + (message[8] << 16) + (message[9] << 8) + message[10];
    if (download_address != UDS_WRITE_START_ADDRESS) {
        return HAL_ERROR;
    }
    if (download_size <= 0  || download_size > 0x00050000) {
        return HAL_ERROR;
    }
    if (Bootloader_Flash_EraseApplication() != HAL_OK) {
        download_in_progress = 0U;
        return HAL_ERROR;
    }
    download_received_size = 0U;
    download_in_progress = 1;
    download_start_address = download_address;
    download_total_size = download_size;
    expected_block_sequence_counter = 1;

    return Bootloader_CAN_SendSingleFrame(message);
}


HAL_StatusTypeDef UDS_36_Process(const uint8_t *message,
    uint16_t message_length) {
    uint32_t current_download_address =
    download_start_address + download_received_size;
    uint8_t block_sequence_counter = 0U;
    if(download_in_progress != 1) {
        return HAL_ERROR;
    }
    if (message_length < 3) {
        return HAL_ERROR;
    }
    uint8_t firmware_data_length = message_length - 2;
    block_sequence_counter = message[1];
    const uint8_t *firmware_data = &message[2];
    if (block_sequence_counter != expected_block_sequence_counter) {
        return HAL_ERROR;
    }
    if (firmware_data_length > download_total_size - download_received_size) {
        return HAL_ERROR;
    }

    if (Bootloader_Flash_WriteData(current_download_address, firmware_data,firmware_data_length) != HAL_OK) {
        download_in_progress = 0;
        return HAL_ERROR;
    }
    expected_block_sequence_counter++;
    download_received_size += firmware_data_length;
    return Bootloader_CAN_SendSingleFrame(message);
}

HAL_StatusTypeDef UDS_37_Process(const uint8_t *message,
    uint16_t message_length) {

    if(download_in_progress != 1U) {
        return HAL_ERROR;
    }
    if (message_length != 5U) {
        return HAL_ERROR;
    }
    if (download_received_size != download_total_size) {
        return HAL_ERROR;
    }
    uint32_t expected_crc = ((uint32_t)message[1] << 24) +
                            ((uint32_t)message[2] << 16) +
                            ((uint32_t)message[3] << 8) +
                            (uint32_t)message[4];
    uint32_t actual_crc =
        Bootloader_CalculateCRC32(
            download_start_address,
            download_total_size);
    if (actual_crc != expected_crc) {
        download_in_progress = 0U;
        return Bootloader_CAN_SendNegativeResponse(message[0],UDS_NRC_GENERAL_PROGRAMMING_FAILURE);
    }
    if (Bootloader_CAN_SendSingleFrame(message) != HAL_OK) {
        return HAL_ERROR;
    }
    download_in_progress = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef UDS_11_Process(const uint8_t *message,
    uint16_t message_length) {

  if (message_length != 2) {
      return HAL_ERROR;
  }
  if (message[1] != UDS_ECU_RESET_HARD_RESET) {
      return HAL_ERROR;
  }
    return Bootloader_CAN_SendSingleFrame(message);

}




HAL_StatusTypeDef Bootloader_Flash_EraseApplication() {

    if (HAL_FLASH_Unlock() != HAL_OK) {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }
    uint32_t fault_sector = 0U;
    FLASH_EraseInitTypeDef EraseInit;
    EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInit.Sector = FLASH_SECTOR_4;
    EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInit.NbSectors = 3U;

    if (HAL_FLASHEx_Erase(&EraseInit,&fault_sector) != HAL_OK) {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }
    HAL_FLASH_Lock();
    return HAL_OK;
}

HAL_StatusTypeDef Bootloader_Flash_WriteData(
    uint32_t current_download_address,
    const uint8_t *firmware_data,
    uint8_t firmware_data_length) {

    if(HAL_FLASH_Unlock() != HAL_OK) {
        return HAL_ERROR;
    }

    for (uint32_t i = 0U; i < firmware_data_length; i++) {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
            current_download_address + i,
            firmware_data[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;


}


/*
 * Calculate CRC-32/IEEE over bytes already stored in Flash.
 *
 * Parameters used by both the Bootloader and the future PC downloader:
 *   initial value : 0xFFFFFFFF
 *   polynomial    : 0xEDB88320 (reflected form)
 *   final XOR     : 0xFFFFFFFF
 */
static uint32_t Bootloader_CalculateCRC32(
    uint32_t start_address,
    uint32_t data_length) {
    const uint8_t *flash_data =
        (const uint8_t *)(uintptr_t)start_address;
    uint32_t crc = 0xFFFFFFFFU;

    for (uint32_t byte_index = 0U;
         byte_index < data_length;
         byte_index++) {
        crc ^= (uint32_t)flash_data[byte_index];

        for (uint8_t bit_index = 0U;
             bit_index < 8U;
             bit_index++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            }
            else {
                crc >>= 1U;
            }
        }
    }

    return crc ^ 0xFFFFFFFFU;
}


