//
// Created by admin on 2026/9/17.
//

#include "Bootloader_can.h"

#include "Bootloader_uds.h"

#define UDS_NEGATIVE_RESPONSE_ID           0x7FU
#define BOOTLOADER_REQUEST_ID              0x7E0U
#define BOOTLOADER_RESPONSE_ID             0x7E8U

#define ISOTP_SINGLE_FRAME                 0x00U
#define ISOTP_FIRST_FRAME                  0x10U
#define ISOTP_CONSECUTIVE_FRAME            0x20U
#define ISOTP_FLOW_CONTROL                 0x30U
#define ISOTP_BLOCK_SIZE                   0x00U
#define FLOW_CONTROL_STmin                 0x00U
#define UDS_POSITIVE_RESPONSE_OFFSET       0x40U

#define MAX_BLOCK_SIZE                     0x20U
#define MAX_TRANSFER_DATA_LENGTH           0x0100U

static uint8_t ecu_reset_pending = 0U;
static uint32_t ecu_reset_mailbox = 0U;


static CAN_TxHeaderTypeDef Bootloader_response_header;

static CAN_HandleTypeDef *bootloader_can_handle = NULL;

volatile static uint8_t bootloader_rx_pending = 0U;

volatile static uint8_t bootloader_rx_dlc = 0U;

volatile static uint8_t bootloader_rx_data[8] = {0U};

volatile static CAN_RxHeaderTypeDef bootloader_rx_header;

static uint8_t isotp_rx_buffer[256] = {0};//接收缓冲区，暂时将我们升级收到的数据存在这儿

static uint16_t isotp_expected_length = 0U;//完整消息长度

static uint16_t isotp_received_length = 0U;//当前收到了多少消息

static uint32_t response_mailbox = 0U;

static uint8_t isotp_message_ready = 0U;

HAL_StatusTypeDef Bootloader_ConfigCommandFilter() {
    /* CAN_FilterTypeDef是HAL库定义的bxCAN硬件过滤器配置结构体 */
    CAN_FilterTypeDef can_filter = {0U};

    /* 把通过的帧放入接收FIFO0 */
    can_filter.FilterBank = 0U; /* 配置第0号过滤器组 */
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    /* IDMASK模式：用“过滤ID + 掩码”决定哪些位必须相等 */
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    /* 32位尺度可在同一个过滤条件中同时比较ID、IDE和RTR */
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;

    /* bxCAN的11位标准ID在32位过滤值中需要左移5位 */
    can_filter.FilterIdHigh = (BOOTLOADER_REQUEST_ID << 5U);
    can_filter.FilterIdLow = 0x0000U;

    /*
     * 掩码的11个ID位全为1，表示每一位都必须与目标ID相等；
     * 低半字同时屏蔽检查IDE和RTR，只接收标准数据帧。
     */
    can_filter.FilterMaskIdHigh = (0x7FFU << 5U);
    can_filter.FilterMaskIdLow = (CAN_ID_EXT | CAN_RTR_REMOTE);
    can_filter.FilterActivation = ENABLE;

    /* 双CAN芯片中，过滤器组0至13分给CAN1，14至27分给CAN2 */
    can_filter.SlaveStartFilterBank = 14U;

    /* 过滤器组0：接收0x7E0 */
    if (HAL_CAN_ConfigFilter(bootloader_can_handle, &can_filter) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef Bootloader_CAN_Init(
    CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }
    bootloader_can_handle = hcan;

    if (Bootloader_ConfigCommandFilter() != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_CAN_Start(bootloader_can_handle) != HAL_OK) {
        return HAL_ERROR;
    }


    Bootloader_response_header.StdId = BOOTLOADER_RESPONSE_ID;
    Bootloader_response_header.ExtId = 0U;
    Bootloader_response_header.IDE = CAN_ID_STD;
    Bootloader_response_header.RTR = CAN_RTR_DATA;
    Bootloader_response_header.DLC = 8U;
    Bootloader_response_header.TransmitGlobalTime = DISABLE;

    /* FIFO0收到新消息时，让CAN外设向CPU发出中断请求 */
    //指定CAN外设中的哪些事件允许产生中断请求。
    //CAN_IT_RX_FIFO0_MSG_PENDING:当FIFO0中出现尚未读取的新报文时，通知CPU处理。


    return HAL_CAN_ActivateNotification(
        bootloader_can_handle,
        CAN_IT_RX_FIFO0_MSG_PENDING);


}



HAL_StatusTypeDef Bootloader_CAN_Process(void)
{
    static uint8_t last_receive_count = 0U;
    static uint8_t first_frame_received = 0U;
    if (ecu_reset_pending == 1) {
        if (HAL_CAN_IsTxMessagePending(
    bootloader_can_handle,
    ecu_reset_mailbox) == 0) {
            ecu_reset_pending = 0U;
            NVIC_SystemReset();
        }
    }
    if (isotp_message_ready == 1) {

        isotp_message_ready = 0U;
        return Bootloader_UDS_Process(isotp_rx_buffer,isotp_received_length);

    }

    if (bootloader_rx_pending != 1U) {
        return HAL_OK;
    }

    uint8_t received_data[8];
    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();//这里就是在关闭普通中断
    received_data[0] = bootloader_rx_data[0];
    received_data[1] = bootloader_rx_data[1];
    received_data[2] = bootloader_rx_data[2];
    received_data[3] = bootloader_rx_data[3];
    received_data[4] = bootloader_rx_data[4];
    received_data[5] = bootloader_rx_data[5];
    received_data[6] = bootloader_rx_data[6];
    received_data[7] = bootloader_rx_data[7];
    CAN_RxHeaderTypeDef rx_header = bootloader_rx_header;

    bootloader_rx_pending = 0U;

    /* 只有原本允许中断时，退出临界区后才重新开启中断 */
    if (previous_primask == 0U)
    {
        __enable_irq();
    }
    if (rx_header.DLC < 1U) {
        return HAL_ERROR;
    }
    uint8_t frame_type = (received_data[0]>>4);

    if (frame_type == ISOTP_SINGLE_FRAME>>4) {
        uint8_t single_frame_length =
    received_data[0] & 0x0FU;
        if (single_frame_length < 1 || single_frame_length > 7) {
            return HAL_ERROR;
        }
        if (single_frame_length > rx_header.DLC - 1) {
            return HAL_ERROR;
        }
        for (uint8_t i = 0U; i < single_frame_length; i++) {
            isotp_rx_buffer[i] = received_data[i+1];
        }
        isotp_expected_length = single_frame_length;
        isotp_received_length = single_frame_length;
        isotp_message_ready   = 1;
        first_frame_received = 0U;
        last_receive_count = 0U;

        return HAL_OK;
    }
    if (frame_type == ISOTP_FIRST_FRAME>>4 && rx_header.DLC == 8) {
        isotp_expected_length = ((received_data[0]&0x0FU)<<8) + received_data[1];
        if (isotp_expected_length <= 7U || isotp_expected_length > 256U) {
            return HAL_ERROR;
        }
        for (int i = 0U; i < 6U; i++) {
            isotp_rx_buffer[i] = received_data[i+2];
        }
        isotp_received_length = 6;
        first_frame_received = 1;
        last_receive_count = 0;

        uint8_t response_data[8] = {0U};
        response_data[0] = ISOTP_FLOW_CONTROL;
        response_data[1] = ISOTP_BLOCK_SIZE;
        response_data[2] = FLOW_CONTROL_STmin;
        return HAL_CAN_AddTxMessage(
                bootloader_can_handle,
                &Bootloader_response_header,
                response_data,
                &response_mailbox);
    }

    if (frame_type == ISOTP_CONSECUTIVE_FRAME>>4) {
        if (first_frame_received != 1) {
            return HAL_ERROR;
        }
        if (isotp_received_length > isotp_expected_length) {
            return HAL_ERROR;
        }
        if (isotp_received_length > 256) {
            return HAL_ERROR;
        }


        uint8_t receive_count = received_data[0] & 0x0F;

        uint16_t remaining_length = isotp_expected_length - isotp_received_length;
        uint16_t current_frame_data_length =
            rx_header.DLC - 1U;

        uint16_t copy_length;

        if (remaining_length < current_frame_data_length)
        {
            copy_length = remaining_length;
        }
        else
        {
            copy_length = current_frame_data_length;
        }


        if (last_receive_count == 0x0FU)
        {
            /* 0x0F之后只能接收0x00 */
            if (receive_count != 0U)
            {
                return HAL_ERROR;
            }
        }
        else
        {
            /* 普通情况：当前序号应当等于上一序号加1 */
            if (receive_count != last_receive_count + 1U)
            {
                return HAL_ERROR;
            }
        }
        for (uint8_t i = 0U; i < copy_length; i++) {
            isotp_rx_buffer[isotp_received_length] = received_data[i+1];
            isotp_received_length++;
        }
        last_receive_count = receive_count;
        if (isotp_received_length == isotp_expected_length)
        {
            first_frame_received = 0U;
            last_receive_count = 0U;
            isotp_message_ready = 1U;
        }
        return HAL_OK;

    }
    if (frame_type == ISOTP_FLOW_CONTROL>>4) {
        return HAL_OK;
    }
    return HAL_ERROR;
}


HAL_StatusTypeDef Bootloader_CAN_SendSingleFrame(
    const uint8_t *uds_data) {
    uint8_t response_data[8] = {0U};
//response_mailbox
    if (uds_data[0] == UDS_REQUEST_DOWNLOAD) {
        response_data[0] = 0x04U;
        response_data[1] = uds_data[0] + UDS_POSITIVE_RESPONSE_OFFSET;
        response_data[2] = MAX_BLOCK_SIZE;
        response_data[3] = MAX_TRANSFER_DATA_LENGTH >> 8;
        response_data[4] = MAX_TRANSFER_DATA_LENGTH & 0x0F;

        return HAL_CAN_AddTxMessage(bootloader_can_handle,
                                    &Bootloader_response_header,
                                    response_data,
                                    &response_mailbox);
    }
    if (uds_data[0] == UDS_TRANSFER_DATA) {
        response_data[0] = 0x02U;
        response_data[1] = uds_data[0] + UDS_POSITIVE_RESPONSE_OFFSET;
        response_data[2] = uds_data[1] ;


        return HAL_CAN_AddTxMessage(bootloader_can_handle,
                                    &Bootloader_response_header,
                                    response_data,
                                    &response_mailbox);
    }
    if (uds_data[0] == UDS_REQUEST_TRANSFER_EXIT) {
        response_data[0] = 0x01U;
        response_data[1] = uds_data[0] + UDS_POSITIVE_RESPONSE_OFFSET;

        return HAL_CAN_AddTxMessage(bootloader_can_handle,
                                    &Bootloader_response_header,
                                    response_data,
                                    &response_mailbox);
    }
    if (uds_data[0] == UDS_SERVICE_ECU_RESET) {
        response_data[0] = 0x02U;
        response_data[1] = uds_data[0] + UDS_POSITIVE_RESPONSE_OFFSET;
        response_data[2] = uds_data[1] ;
        if (HAL_CAN_AddTxMessage(bootloader_can_handle,
                            &Bootloader_response_header,
                            response_data,
                            &ecu_reset_mailbox) == HAL_OK) {
            ecu_reset_pending = 1U;
            return HAL_OK;
        }
        return HAL_ERROR;
    }
return HAL_ERROR;

}



HAL_StatusTypeDef Bootloader_CAN_SendNegativeResponse(uint8_t service_id,uint8_t NRC) {
    uint8_t response_data[8] = {0U};
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = service_id;
    response_data[3] = NRC;

    return HAL_CAN_AddTxMessage(bootloader_can_handle,
                                &Bootloader_response_header,
                                response_data,
                                &response_mailbox);
}





void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if ((bootloader_can_handle == NULL) || (hcan != bootloader_can_handle))
    {
        return;
    }

    if (HAL_CAN_GetRxMessage(
            hcan,
            CAN_RX_FIFO0,
            &rx_header,
            rx_data) != HAL_OK)
    {
        return;
    }

    if (rx_header.IDE == CAN_ID_EXT) {
        return;
    }
    if (rx_header.RTR != CAN_RTR_DATA) {
        return;
    }
    if (rx_header.StdId != BOOTLOADER_REQUEST_ID) {
        return;
    }
    if (rx_header.DLC > 8U) {
        return;
    }

    bootloader_rx_data[0] = rx_data[0];
    bootloader_rx_data[1] = rx_data[1];
    bootloader_rx_data[2] = rx_data[2];
    bootloader_rx_data[3] = rx_data[3];
    bootloader_rx_data[4] = rx_data[4];
    bootloader_rx_data[5] = rx_data[5];
    bootloader_rx_data[6] = rx_data[6];
    bootloader_rx_data[7] = rx_data[7];
    bootloader_rx_header = rx_header;
    bootloader_rx_dlc = rx_header.DLC;
    bootloader_rx_pending = 1U;






}













