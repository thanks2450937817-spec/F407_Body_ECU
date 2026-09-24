#ifndef F407_BODY_ECU_DTC_MANAGER_H
#define F407_BODY_ECU_DTC_MANAGER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
/*
 * DTC管理模块的公开接口。
 * 具体DTC编号、状态和计数保留在DTC_manager.c内部，
 * CAN/UDS模块以后通过这里声明的接口访问，避免直接操作内部变量。
 */
typedef struct
{
    uint16_t id;
    uint8_t ftb;
    uint8_t status;
    uint16_t window_sample_count;
    uint16_t window_failure_count;
} DTC_Record;

extern DTC_Record lamp_output_dtc;
/**
 * @brief  初始化DTC运行状态和统计窗口计数
 * @retval None
 */
void DTC_Manager_Init(void);

/**
 * @brief  上报一次周期检测结果，并更新testFailed和确认统计
 * @param  test_failed 0表示本次检测正常，非0表示本次检测失败
 * @retval None
 */
void DTC_Manager_ReportTestResult(uint8_t test_failed);

HAL_StatusTypeDef DTC_Manager_ClearAll(void);


#endif /* F407_BODY_ECU_DTC_MANAGER_H */
