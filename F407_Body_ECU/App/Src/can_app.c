/*
 * CAN应用层模块，不由STM32CubeMX生成。
 *
 * 当前网络角色：
 * - STM32F407：车身ECU
 * - CANable + CANgaroo：PC端测试节点/诊断仪
 * - SN65HVD230：把STM32的TX/RX逻辑电平转换为CANH/CANL差分信号
 *
 * 当前经典CAN协议（均为11位标准数据帧）：
 * - 0x123，ECU -> PC：100 ms周期心跳，DLC=8
 * - 0x200，PC  -> ECU：自定义控制命令
 * - 0x201，ECU -> PC：自定义命令应答
 * - 0x7E0，PC  -> ECU：UDS物理寻址请求（ISO-TP承载）
 * - 0x7E8，ECU -> PC：UDS物理寻址响应（ISO-TP承载）
 *
 * 当前只支持诊断会话控制的默认会话请求10 01，并返回包含P2/P2*的
 * 标准正响应50 01；其他服务返回NRC 0x11，0x10长度错误返回NRC 0x13。
 * 其余否定响应以及ISO-TP多帧传输将在后续逐步实现。
 */
#include "can_app.h"
#include "body_control.h"
#include "DTC_manager.h"
/* STM32周期发送的心跳帧ID；U后缀表示无符号整数常量 */
#define CAN_HEARTBEAT_ID          0x123U

/* CANgaroo发送给STM32的命令帧ID */
#define CAN_COMMAND_ID            0x200U

/* STM32返回给CANgaroo的应答帧ID */
#define CAN_RESPONSE_ID           0x201U

/* PC诊断仪发送给ECU的UDS请求帧ID */
#define CAN_UDS_REQUEST_ID        0x7E0U

/* ECU返回给PC诊断仪的UDS响应帧ID */
#define CAN_UDS_RESPONSE_ID       0x7E8U

/*
 * ISO-TP单帧首字节：高4位表示帧类型，低4位表示UDS有效载荷长度。
 * 例如02 10 01中：0x02高4位为0=Single Frame，低4位2=后续2字节。
 */
#define ISOTP_FRAME_TYPE_MASK     0xF0U
#define ISOTP_FRAME_LENGTH_MASK   0x0FU
#define ISOTP_SINGLE_FRAME_TYPE   0x00U

/* 诊断会话控制与子功能 */
#define UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL 0x10U
#define UDS_SESSION_DEFAULT                    0x01U
#define UDS_SESSION_PROGRAMMING                0x02U
#define UDS_SESSION_EXTENDED                   0x03U
#define BOOT_REQUEST_MAGIC                     0x424F4F54U//升级请求标志位
#define RESET_WAIT_TIMEOUT_TIME                100U//复位等待超时时间
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION   0x7EU//当前会话不支持的子功能
/* 诊断仪在线与子功能 */
#define UDS_SERVICE_TESTER_PRESENT             0x3EU
#define UDS_TESTER_PRESENT_ZERO_SUBFUNCTION    0x00U //诊断仪仍然在线，请ECU维持当前诊断会话
/* 请求读取DID */
#define UDS_READ_DATA_BY_IDENTIFIER            0x22U
#define UDS_Active_Diagnostic_Session          0xF186U
#define UDS_NRC_REQUEST_OUT_OF_RANGE           0x31U//不支持的DID回复报文
/* 请求读取DTC */
#define UDS_READ_DTC                           0x19U
#define UDS_ReportDTCByStatusMask              0x02U
#define UDS_SUPPORTED_STATUS_MASK              0x09U
//清除DTC
#define UDS_ClearDiagnosticInformation         0x14U
#define UDS_CLEAR_ALL_DTC                      0xFFFFFFU
/* UDS正响应服务ID = 请求服务ID + 0x40，例如0x10变为0x50 */
#define UDS_POSITIVE_RESPONSE_OFFSET           0x40U

/* UDS否定响应格式中的固定服务ID和“服务不支持”响应码 */
#define UDS_NEGATIVE_RESPONSE_ID               0x7FU
#define UDS_NRC_SERVICE_NOT_SUPPORTED          0x11U
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED      0x12U
#define UDS_NRC_INCORRECT_MESSAGE_LENGTH       0x13U
#define UDS_NRC_GENERAL_PROGRAMMING_FAILURE    0x72U//0x72表示执行Flash擦除、写入等操作失败。
/*
 * 默认会话的服务器响应时间参数：
 * P2Server_max以1 ms为单位，50表示普通响应最长50 ms；
 * P2*Server_max以10 ms为单位，500表示等待响应时最长5000 ms。
 */
#define UDS_P2_SERVER_MAX_MS                   50U
#define UDS_P2_STAR_SERVER_MAX_10MS            500U

/* 当前支持的应用层命令 */
#define CAN_COMMAND_PING          0x01U
#define CAN_COMMAND_SET_LAMP      0x02U

/* 应答状态码 */
#define CAN_RESPONSE_OK           0x00U
#define CAN_RESPONSE_UNSUPPORTED  0x01U
#define CAN_RESPONSE_INVALID_DATA 0x02U

/* 心跳发送周期，单位为毫秒 */
#define CAN_HEARTBEAT_PERIOD_MS   100U

/*
 * 为S3超时设定变量，分别保存实时时间和最新更新时间
 */
static uint32_t previous_time = 0U;
static uint32_t last_time = 0U;
/*
 * 为22读数据设定变量
 */
volatile static uint8_t Data_identifier[8];

/* 提前声明0x10服务处理函数，供下面的函数指针表使用 */
static HAL_StatusTypeDef UDS_SERVICE_ID_10_Process(void);
static HAL_StatusTypeDef UDS_SERVICE_ID_3E_Process(void);
static HAL_StatusTypeDef UDS_SERVICE_ID_22_Process(void);
static HAL_StatusTypeDef UDS_SERVICE_ID_19_Process(void);
static HAL_StatusTypeDef UDS_SERVICE_ID_14_Process(void);
/*定义函数指针类型
HAL_StatusTypeDef：被指向函数的返回类型。
*：说明这是指针。
UDS_ServiceHandler：我们给这种函数指针类型起的名字。
(void)：被指向的函数没有参数。
*/
typedef HAL_StatusTypeDef (*UDS_ServiceHandler)(void);

//定义函数指针表结构，下面会创建指针函数表
typedef struct
{
  uint8_t service_id;
  UDS_ServiceHandler process;
} UDS_ServiceEntry;
//创建指针函数表
static const UDS_ServiceEntry uds_service_table[] =
{

  {UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL,UDS_SERVICE_ID_10_Process},

  {UDS_SERVICE_TESTER_PRESENT, UDS_SERVICE_ID_3E_Process},

  {UDS_READ_DATA_BY_IDENTIFIER, UDS_SERVICE_ID_22_Process},

  {UDS_READ_DTC, UDS_SERVICE_ID_19_Process},

  {UDS_ClearDiagnosticInformation, UDS_SERVICE_ID_14_Process}


};

/*
 * 保存CAN_App_Init()传入的CAN句柄，供后续处理使用。
 * CAN_HandleTypeDef是HAL库定义的CAN外设“管理对象”，其中包含
 * CAN1寄存器入口、初始化参数、运行状态和错误信息等。
 */
static CAN_HandleTypeDef *can_app_handle = NULL;

/*
 * CAN_TxHeaderTypeDef是HAL库定义的“发送帧头”结构体，记录ID、
 * 标准帧/扩展帧、数据帧/远程帧和DLC；真正的8字节数据单独存放。
 */
static CAN_TxHeaderTypeDef can_heartbeat_header;
static CAN_TxHeaderTypeDef can_response_header;
static CAN_TxHeaderTypeDef can_uds_response_header;
static uint8_t can_heartbeat_data[8];
/* HAL_CAN_AddTxMessage()通过该变量返回本次使用的发送邮箱编号 */
static uint32_t can_tx_mailbox;
static uint32_t can_tx_sequence = 0U;
static uint32_t last_heartbeat_tick = 0U;
static uint8_t programming_reset_pending = 0U;//等待复位
static uint32_t programming_response_mailbox = 0U;//接受回复邮箱返回的信息
static uint32_t start_time_of_reset_pending = 0U;//复位等待开始时间
static uint32_t response_mailbox   = 0U;//使用了那个邮箱
/*
 * CAN命令由接收中断写入、主循环读取，因此使用volatile，提醒编译器
 * 每次都从内存读取，不能假设这些值在主循环之外不会改变。
 * 当前版本使用单命令槽，适用于低频控制和手动测试。
 */
static volatile uint8_t can_command_pending = 0U;
static volatile uint8_t can_command_code = 0U;
static volatile uint8_t can_command_token = 0U;
static volatile uint8_t can_command_value = 0U;
static volatile uint8_t can_command_dlc = 0U;
static volatile uint8_t can_current_session = UDS_SESSION_DEFAULT;//当前会话状态
/*
 * 暂存一条已经通过格式检查的ISO-TP单帧：有效长度、UDS服务ID、
 * 子功能和“有新请求”标志都由中断写入、由主循环读取。
 */
static volatile uint8_t uds_request_pending = 0U;
static volatile uint8_t uds_payload_length = 0U;
static volatile uint8_t uds_service_id = 0U;
static volatile uint8_t uds_sub_function = 0U;
//DTC
static volatile uint8_t uds_requested_status_mask = 0U;//19
static volatile uint8_t uds_groupOfDTC[3] = {0};//14



/**
  * @brief  配置CAN接收过滤器，接收应用命令0x200和UDS请求0x7E0
  * @retval HAL库执行结果
  */
static HAL_StatusTypeDef CAN_App_ConfigCommandFilter(void)
{
  /* CAN_FilterTypeDef是HAL库定义的bxCAN硬件过滤器配置结构体 */
  CAN_FilterTypeDef can_filter = {0};

  /* 两个过滤器都把通过的帧放入接收FIFO0 */
  can_filter.FilterBank = 0; /* 配置第0号过滤器组 */
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  /* IDMASK模式：用“过滤ID + 掩码”决定哪些位必须相等 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  /* 32位尺度可在同一个过滤条件中同时比较ID、IDE和RTR */
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;

  /* bxCAN的11位标准ID在32位过滤值中需要左移5位 */
  can_filter.FilterIdHigh = (CAN_COMMAND_ID << 5);
  can_filter.FilterIdLow = 0x0000U;

  /*
   * 掩码的11个ID位全为1，表示每一位都必须与目标ID相等；
   * 低半字同时屏蔽检查IDE和RTR，只接收标准数据帧。
   */
  can_filter.FilterMaskIdHigh = (0x7FFU << 5);
  can_filter.FilterMaskIdLow = (CAN_ID_EXT | CAN_RTR_REMOTE);
  can_filter.FilterActivation = ENABLE;

  /* 双CAN芯片中，过滤器组0至13分给CAN1，14至27分给CAN2 */
  can_filter.SlaveStartFilterBank = 14;

  /* 过滤器组0：接收现有应用命令ID 0x200 */
  if (HAL_CAN_ConfigFilter(can_app_handle, &can_filter) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /*
   * 过滤器组1：保持相同的帧类型和掩码，
   * 只把目标ID改为UDS物理寻址请求ID 0x7E0。
   */
  can_filter.FilterBank = 1;
  can_filter.FilterIdHigh = (CAN_UDS_REQUEST_ID << 5);

  /* 第二次调用把修改后的临时配置写入硬件过滤器组1 */
  return HAL_CAN_ConfigFilter(can_app_handle, &can_filter);
}

/**
  * @brief  处理一条由接收中断保存的待处理命令
  * @retval HAL库执行结果
  */
static HAL_StatusTypeDef CAN_App_ProcessPendingCommand(void)//在主循环中不断执行
{
  uint8_t command_code;//保存命令码，表示 PC 要让 ECU 做什么。
  uint8_t command_token;//token 可以翻译为“令牌”或“请求编号”。用于区分连续的两次应答
  uint8_t command_value;//保存命令携带的具体参数，对于车灯命令0：关灯 1：开灯
  uint8_t command_dlc;//表示这帧 CAN 报文携带多少字节数据
  uint8_t response_data[4];//这是准备发送给 PC 的应答数据数组。response_data[0]：命令码，response_data[1]：令牌，response_data[2]：执行结果，response_data[3]：当前车灯状态
  uint32_t previous_primask;//保存的是 Cortex-M4 的全局可屏蔽中断总开关状态

  if (can_command_pending == 0U)//表示目前没有等待中的CAN命令要处理
  {
    return HAL_OK;
  }

  /* 没有空闲邮箱时保留命令，下一次主循环继续尝试 */
  if (HAL_CAN_GetTxMailboxesFreeLevel(can_app_handle) == 0U)
  {
    return HAL_OK;
  }

  /*
   * 复制中断与主循环共享的数据时短暂关闭中断，
   * 避免命令码、令牌和参数分别来自不同的CAN帧。
   */
  previous_primask = __get_PRIMASK();
  __disable_irq();//这里就是在关闭普通中断


  command_code = can_command_code;
  command_token = can_command_token;
  command_value = can_command_value;
  command_dlc = can_command_dlc;
  can_command_pending = 0U;

  /* 只有原本允许中断时，退出临界区后才重新开启中断 */
  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  /*
   * 0x201应答格式：
   * byte0=命令码，byte1=令牌，byte2=状态码；
   * 灯命令还用byte3返回当前实际逻辑灯状态。
   * “令牌”由PC给出并原样返回，用于匹配哪次请求产生了该应答。
   */
  response_data[0] = command_code;
  response_data[1] = command_token;
  can_response_header.DLC = 3U;

  if (command_code == CAN_COMMAND_PING)
  {
    response_data[2] = CAN_RESPONSE_OK;
  }
  else if (command_code == CAN_COMMAND_SET_LAMP)
  {
    if ((command_dlc >= 3U) && (command_value <= 1U))
    {
      BodyControl_SetLamp(command_value);
      response_data[2] = CAN_RESPONSE_OK;
    }
    else
    {
      /* 长度不足或状态不是0/1时，不改变当前车灯状态 */
      response_data[2] = CAN_RESPONSE_INVALID_DATA;
    }

    /* 灯光命令额外返回当前实际采用的逻辑灯状态 */
    response_data[3] = BodyControl_GetLampState();
    can_response_header.DLC = 4U;
  }
  else
  {
    response_data[2] = CAN_RESPONSE_UNSUPPORTED;
  }

  return HAL_CAN_AddTxMessage(
      can_app_handle,
      &can_response_header,
      response_data,
      &response_mailbox);
}

/**
  * @brief  处理一条待处理的UDS单帧请求
  * @retval HAL_OK表示本次处理完成或没有请求，HAL_ERROR表示发送失败
  */
static HAL_StatusTypeDef CAN_App_ProcessPendingUdsRequest(void)
{
  uint8_t service_id;
  uint8_t response_data[8] = {0U};
  uint32_t previous_primask;

  if (uds_request_pending == 0U)
  {
    return HAL_OK;
  }

  /*
   * 当前支持的请求需要立即发送应答。没有空闲发送邮箱时先不取走请求，
   * 留到下一次主循环再处理，避免请求已经清除却无法发送响应。
   */
  if (HAL_CAN_GetTxMailboxesFreeLevel(can_app_handle) == 0U)
  {
    return HAL_OK;
  }


  /* 当前先直接读取服务号，用于查找对应的UDS服务处理函数 */
  service_id = uds_service_id;

  /* 计算函数指针表中包含多少个服务 */
  uint32_t service_count =
      sizeof(uds_service_table) / sizeof(uds_service_table[0]);

  /* 按照服务号遍历函数指针表 */
  for (uint32_t index = 0U; index < service_count; index++)
  {
    if (uds_service_table[index].service_id == service_id)
    {
      return uds_service_table[index].process();
    }
  }

  previous_primask = __get_PRIMASK();
  __disable_irq();

  uds_request_pending = 0U;

  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  response_data[0] = 0x03U;
  response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
  response_data[2] = service_id;
  response_data[3] = UDS_NRC_SERVICE_NOT_SUPPORTED;

  return HAL_CAN_AddTxMessage(
      can_app_handle,
      &can_uds_response_header,
      response_data,
      &response_mailbox);
}

/**
  * @brief  服务处理函数
  * @retval HAL_OK表示本次处理完成或没有请求，HAL_ERROR表示发送失败
  */
static HAL_StatusTypeDef UDS_SERVICE_ID_10_Process(void) {

  uint8_t payload_length;
  uint8_t service_id;
  uint8_t sub_function;
  uint8_t response_data[8] = {0U};
  uint32_t previous_primask;

  /*
 * 中断可能随时写入下一条UDS请求，所以复制共享数据时短暂关闭中断。
 * 复制完成后清除pending，表示主循环已经取走这一条请求。
 */
  previous_primask = __get_PRIMASK();
  __disable_irq();

  payload_length = uds_payload_length;//uds_payload_length等等都是在中断中被赋值的
  service_id = uds_service_id;
  sub_function = uds_sub_function;
  uds_request_pending = 0U;

  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  /*
 * DiagnosticSessionControl请求必须恰好包含两个UDS字节：
 * 服务ID和会话子功能。长度不是2时返回NRC 0x13，表示
 * IncorrectMessageLengthOrInvalidFormat（消息长度或格式错误）。
 */
  if (payload_length != 2U)
  {
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = service_id;
    response_data[3] = UDS_NRC_INCORRECT_MESSAGE_LENGTH;

    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,
        response_data,
        &response_mailbox);
  }

  /*
   * 长度正确后再判断子功能。
   * 10 = DiagnosticSessionControl（诊断会话控制）
   * 01 = defaultSession（默认会话）
   * 02 = programmerSession(编程会话)
   * 03 = extendedSession（扩展会话）
   * 其他子功能暂时忽略，留到下一小步返回NRC 0x12。
   */
  if ((sub_function != UDS_SESSION_DEFAULT) &&
      (sub_function != UDS_SESSION_EXTENDED) &&
      (sub_function != UDS_SESSION_PROGRAMMING))
  {
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = service_id;
    response_data[3] = UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,//CAN帧头，保存CAN报文的基本属性，比如是不是标准帧，有没有扩展ID，数据位有多少等等
        response_data,
        &response_mailbox);
  }
  uint8_t requested_session = 0U;
  if (sub_function == UDS_SESSION_DEFAULT) {
    requested_session = UDS_SESSION_DEFAULT;
  }

  if (sub_function == UDS_SESSION_EXTENDED) {
    requested_session = UDS_SESSION_EXTENDED;
  }

  if (sub_function == UDS_SESSION_PROGRAMMING &&
    can_current_session != UDS_SESSION_EXTENDED) {
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = service_id;
    response_data[3] = UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION;
    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,//CAN帧头，保存CAN报文的基本属性，比如是不是标准帧，有没有扩展ID，数据位有多少等等
        response_data,
        &response_mailbox);
  }
  if (sub_function == UDS_SESSION_PROGRAMMING &&
    can_current_session == UDS_SESSION_EXTENDED) {

    requested_session = UDS_SESSION_PROGRAMMING;





  }
  /*
   * 构造标准的DiagnosticSessionControl单帧正响应：
   * byte0 = 0x06，表示单帧内有6字节UDS有效载荷
   * byte1 = 0x50，即请求服务0x10 + 正响应偏移0x40
   * byte2 = requested_session，返回接受的目标会话0x01或0x02或0x03
   * byte3..4 = P2Server_max，0x0032表示50 ms
   * byte5..6 = P2*Server_max，0x01F4表示500 * 10 ms = 5000 ms
   * byte7 = 0，用作经典CAN的填充字节
   */
  HAL_StatusTypeDef send_result = 0U;
  response_data[0] = 0x06U;
  response_data[1] =
      UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL +
      UDS_POSITIVE_RESPONSE_OFFSET;
  response_data[2] = requested_session;
  response_data[3] = (uint8_t)(UDS_P2_SERVER_MAX_MS >> 8);//右移8位后再转换为uint8_t → 取得高字节
  response_data[4] = (uint8_t)UDS_P2_SERVER_MAX_MS;//直接转换为uint8_t → 取得低字节
  response_data[5] =
      (uint8_t)(UDS_P2_STAR_SERVER_MAX_10MS >> 8);
  response_data[6] =
      (uint8_t)UDS_P2_STAR_SERVER_MAX_10MS;

  send_result = HAL_CAN_AddTxMessage(
                                          can_app_handle,
                                          &can_uds_response_header,
                                          response_data,
                                          &response_mailbox);
  if (send_result != HAL_OK) {
    return send_result;
  }
  if (requested_session == UDS_SESSION_PROGRAMMING) {
    programming_response_mailbox = response_mailbox;
    start_time_of_reset_pending = HAL_GetTick();
    programming_reset_pending = 1U;
  }

  can_current_session = requested_session;

  if (can_current_session == UDS_SESSION_EXTENDED  ||
      can_current_session == UDS_SESSION_PROGRAMMING) {
    previous_time = HAL_GetTick();
  }
  return send_result;


}

static HAL_StatusTypeDef UDS_SERVICE_ID_3E_Process(void) {


  /*
 * 中断可能随时写入下一条UDS请求，所以复制共享数据时短暂关闭中断。
 * 复制完成后清除pending，表示主循环已经取走这一条请求。
 */
  uint32_t previous_primask = __get_PRIMASK();
  __disable_irq();

  uint8_t response_data[8] = {0U};
  uint8_t payload_length = uds_payload_length;//uds_payload_length等等都是在中断中被赋值的
  uint8_t service_id = uds_service_id;
  uint8_t sub_function = uds_sub_function;
  uds_request_pending = 0U;

  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  /*
 * DiagnosticSessionControl请求必须恰好包含两个UDS字节：
 * 服务ID和会话子功能。长度不是2时返回NRC 0x13，表示
 * IncorrectMessageLengthOrInvalidFormat（消息长度或格式错误）。
 */
  if (payload_length != 2U)
  {
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = service_id;
    response_data[3] = UDS_NRC_INCORRECT_MESSAGE_LENGTH;

    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,
        response_data,
        &response_mailbox);
  }

  /*
    3E的作用只有一个，延长扩展会话时间
   */
  if (sub_function != UDS_TESTER_PRESENT_ZERO_SUBFUNCTION)
  {
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = service_id;
    response_data[3] = UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,//CAN帧头，保存CAN报文的基本属性，比如是不是标准帧，有没有扩展ID，数据位有多少等等
        response_data,
        &response_mailbox);
  }
/*
  byte0 = 0x02，两个UDS有效字节
  byte1 = 0x7E，0x3E + 0x40
  byte2 = 0x00，原样返回子功能
  byte3～7 = 0，填充
*/

  response_data[0] = 0x02U;
  response_data[1] =
      UDS_SERVICE_TESTER_PRESENT +
      UDS_POSITIVE_RESPONSE_OFFSET;
  response_data[2] = sub_function;
  HAL_StatusTypeDef send_result = HAL_CAN_AddTxMessage(
                                          can_app_handle,
                                          &can_uds_response_header,
                                          response_data,
                                          &response_mailbox);
  if (send_result != HAL_OK) {
    return send_result;
  }
  previous_time = HAL_GetTick();//获取更新时间
  return send_result;
}

static HAL_StatusTypeDef UDS_SERVICE_ID_22_Process(void) {


uint8_t payload[8];
  /*
 * 中断可能随时写入下一条UDS请求，所以复制共享数据时短暂关闭中断。
 * 复制完成后清除pending，表示主循环已经取走这一条请求。
 */
  uint32_t previous_primask = __get_PRIMASK();
  __disable_irq();

  uint8_t response_data[8] = {0U};
  uint8_t payload_length = uds_payload_length;//uds_payload_length等等都是在中断中被赋值的
  payload[0] = uds_service_id;
  payload[1] = Data_identifier[0];
  payload[2] = Data_identifier[1];
  uds_request_pending = 0U;

  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  /*
 * DiagnosticSessionControl请求必须恰好包含两个UDS字节：
 * 服务ID和会话子功能。长度不是2时返回NRC 0x13，表示
 * IncorrectMessageLengthOrInvalidFormat（消息长度或格式错误）。
 */
  if (payload_length != 3U)
  {
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = payload[0];//服务ID
    response_data[3] = UDS_NRC_INCORRECT_MESSAGE_LENGTH;

    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,
        response_data,
        &response_mailbox);
  }

  if (payload[1] == (uint8_t)(UDS_Active_Diagnostic_Session>>8) &&
      payload[2] == (uint8_t)(UDS_Active_Diagnostic_Session)) {
    /*
  byte0 = 0x02，两个UDS有效字节
  byte1 = 0x7E，0x3E + 0x40
  byte2 = 0x00，原样返回子功能
  byte3～7 = 0，填充
*/
    //当前只允许请求一个DID
    response_data[0] = 0x04U;
    response_data[1] =
        UDS_READ_DATA_BY_IDENTIFIER +
        UDS_POSITIVE_RESPONSE_OFFSET;
    response_data[2] = payload[1];//当前的0xF1
    response_data[3] = payload[2];//当前的0x86
    response_data[4] = can_current_session;
    HAL_StatusTypeDef send_result = HAL_CAN_AddTxMessage(
                                            can_app_handle,
                                            &can_uds_response_header,
                                            response_data,
                                            &response_mailbox);
    if (send_result != HAL_OK) {
      return send_result;
    }
    return send_result;
  }
  response_data[0] = 0x03U;
  response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
  response_data[2] = payload[0];//服务ID
  response_data[3] = UDS_NRC_REQUEST_OUT_OF_RANGE;

  return HAL_CAN_AddTxMessage(
      can_app_handle,
      &can_uds_response_header,
      response_data,
      &response_mailbox);

}

static HAL_StatusTypeDef UDS_SERVICE_ID_19_Process(void) {


uint8_t payload[9] = {0};
  /*
 * 中断可能随时写入下一条UDS请求，所以复制共享数据时短暂关闭中断。
 * 复制完成后清除pending，表示主循环已经取走这一条请求。
 */
  uint32_t previous_primask = __get_PRIMASK();
  __disable_irq();

  uint8_t response_data[8] = {0U};
  //uds_payload_length等等都是在中断中被赋值的
  payload[0] = uds_payload_length;
  payload[1] = uds_service_id;
  payload[2] = uds_sub_function;
  payload[4] = uds_requested_status_mask ;
  payload[5] = (uint8_t)(lamp_output_dtc.id>>8);//两字节DTC_ID的高字节
  payload[6] = (uint8_t)lamp_output_dtc.id;;//两字节DTC_ID的低字节
  payload[7] = lamp_output_dtc.ftb;
  payload[8] = lamp_output_dtc.status;
  uds_request_pending = 0U;

  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  /*
 * 服务ID和会话子功能。长度不是2时返回NRC 0x13，表示
 * IncorrectMessageLengthOrInvalidFormat（消息长度或格式错误）。
 */
  if (payload[0] != 3U)
  {
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = payload[1];//服务ID
    response_data[3] = UDS_NRC_INCORRECT_MESSAGE_LENGTH;

    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,
        response_data,
        &response_mailbox);
  }

  if (payload[2] != UDS_ReportDTCByStatusMask ) {
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = payload[1];//服务ID
    response_data[3] = UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;

    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,
        response_data,
        &response_mailbox);

  }
  if ((payload[4] & UDS_SUPPORTED_STATUS_MASK & lamp_output_dtc.status) == 0x00 ) {//说明无故障
    response_data[0] = 0x03U;
    response_data[1] =
        UDS_READ_DTC +
        UDS_POSITIVE_RESPONSE_OFFSET;
    response_data[2] = payload[2];
    response_data[3] = UDS_SUPPORTED_STATUS_MASK;
    response_data[4] = 0x00;
    response_data[5] = 0x00;
    response_data[6] = 0x00;
    response_data[7] = 0x00;

    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,
        response_data,
        &response_mailbox);

  }
  /*
byte0 = 0x02，两个UDS有效字节
byte1 = 0x7E，0x3E + 0x40
byte2 = 0x00，原样返回子功能
byte3～7 = 0，填充
*/
  response_data[0] = 0x07U;
  response_data[1] =
      UDS_READ_DTC +
      UDS_POSITIVE_RESPONSE_OFFSET;
  response_data[2] = payload[2];
  response_data[3] = UDS_SUPPORTED_STATUS_MASK;
  response_data[4] = payload[5];//B10高两字节，代表故障所属系统和是标准还是OEM（自定义）
  response_data[5] = payload[6];//00低两字节，代表具体故障部件
  response_data[6] = payload[7];//FTB故障类型
  response_data[7] = payload[8];//DTC信息
  HAL_StatusTypeDef send_result = HAL_CAN_AddTxMessage(
                                          can_app_handle,
                                          &can_uds_response_header,
                                          response_data,
                                          &response_mailbox);
  if (send_result != HAL_OK) {
    return send_result;
  }
  return send_result;
}

static HAL_StatusTypeDef UDS_SERVICE_ID_14_Process(void) {


uint8_t payload[9] = {0};
  /*
 * 中断可能随时写入下一条UDS请求，所以复制共享数据时短暂关闭中断。
 * 复制完成后清除pending，表示主循环已经取走这一条请求。
 */
  uint32_t previous_primask = __get_PRIMASK();
  __disable_irq();

  uint8_t response_data[8] = {0U};
  //uds_payload_length等等都是在中断中被赋值的
  payload[0] = uds_payload_length;
  payload[1] = uds_service_id;
  payload[2] = uds_groupOfDTC[0];
  payload[3] = uds_groupOfDTC[1];
  payload[4] = uds_groupOfDTC[2];

  uds_request_pending = 0U;

  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  /*
 * DiagnosticSessionControl请求必须恰好包含两个UDS字节：
 * 服务ID和会话子功能。长度不是2时返回NRC 0x13，表示
 * IncorrectMessageLengthOrInvalidFormat（消息长度或格式错误）。
 */
  if (payload[0] != 4U)
  {
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = payload[1];//服务ID
    response_data[3] = UDS_NRC_INCORRECT_MESSAGE_LENGTH;

    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,
        response_data,
        &response_mailbox);
  }

  if (((payload[2] == (uint8_t)(UDS_CLEAR_ALL_DTC>>16)) && //匹配0xFFFFFFU
       (payload[3] == (uint8_t)(UDS_CLEAR_ALL_DTC>>8)) &&
       (payload[4] == (uint8_t)(UDS_CLEAR_ALL_DTC))) != 1) {
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = payload[1];//服务ID
    response_data[3] = UDS_NRC_REQUEST_OUT_OF_RANGE;

    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,
        response_data,
        &response_mailbox);

  }


  const HAL_StatusTypeDef clear_result = DTC_Manager_ClearAll();

  if (clear_result != HAL_OK) {
    response_data[0] = 0x03U;
    response_data[1] = UDS_NEGATIVE_RESPONSE_ID;
    response_data[2] = payload[1];//服务ID
    response_data[3] = UDS_NRC_GENERAL_PROGRAMMING_FAILURE;

    return HAL_CAN_AddTxMessage(
        can_app_handle,
        &can_uds_response_header,
        response_data,
        &response_mailbox);
  }

/*
byte0 = 0x01，两个UDS有效字节
byte1 = 0x7E，0x14 + 0x40
byte2 = 0x00，原样返回子功能
byte3～7 = 0，填充
*/
  response_data[0] = 0x01U;
  response_data[1] =
      UDS_ClearDiagnosticInformation +
      UDS_POSITIVE_RESPONSE_OFFSET;

  const HAL_StatusTypeDef send_result = HAL_CAN_AddTxMessage(
                                          can_app_handle,
                                          &can_uds_response_header,
                                          response_data,
                                          &response_mailbox);
  if (send_result != HAL_OK) {
    return send_result;
  }
  return send_result;
}
/**
  * @brief  到达周期后发送一帧ECU心跳
  * @retval None
  */

static void CAN_App_SendHeartbeat(void)
{
  uint32_t current_tick = HAL_GetTick();

  if ((current_tick - last_heartbeat_tick) < CAN_HEARTBEAT_PERIOD_MS)
  {
    return;
  }

  /* 发生延迟时从当前时刻重新计时，不集中补发过去的心跳 */
  last_heartbeat_tick = current_tick;

  /*
   * 0x123心跳数据格式：
   * byte0..3 = 32位递增序号（小端，低字节先发送）
   * byte4    = 固定标记0xA5
   * byte5    = 固定标记0x5A
   * byte6    = 车灯逻辑状态（0关/1开）
   * byte7    = 预留，当前固定为0
   */
  can_heartbeat_data[0] = (uint8_t)(can_tx_sequence);
  can_heartbeat_data[1] = (uint8_t)(can_tx_sequence >> 8);
  can_heartbeat_data[2] = (uint8_t)(can_tx_sequence >> 16);
  can_heartbeat_data[3] = (uint8_t)(can_tx_sequence >> 24);
  can_heartbeat_data[4] = 0xA5U;
  can_heartbeat_data[5] = 0x5AU;
  can_heartbeat_data[6] = BodyControl_GetLampState();
  can_heartbeat_data[7] = 0x00U;

  if (HAL_CAN_GetTxMailboxesFreeLevel(can_app_handle) == 0U)
  {
    return;
  }

  if (HAL_CAN_AddTxMessage(
          can_app_handle,
          &can_heartbeat_header,
          can_heartbeat_data,
          &can_tx_mailbox) == HAL_OK)
  {
    /* 成功放入发送邮箱后才增加序号 */
    can_tx_sequence++;
  }
}

HAL_StatusTypeDef CAN_App_Init(CAN_HandleTypeDef *hcan)
{
  if (hcan == NULL)
  {
    return HAL_ERROR;
  }
  can_current_session = UDS_SESSION_DEFAULT;
  can_app_handle = hcan;
  can_command_pending = 0U;
  uds_request_pending = 0U;
  can_tx_sequence = 0U;

  if (CAN_App_ConfigCommandFilter() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 过滤器必须在CAN启动前配置；随后CAN才开始参与总线通信 */
  if (HAL_CAN_Start(can_app_handle) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 配置周期心跳帧：标准数据帧ID 0x123，长度8字节 */
  can_heartbeat_header.StdId = CAN_HEARTBEAT_ID;
  can_heartbeat_header.ExtId = 0U;//把没有用到的值配置为0，避免他保留不确定值
  can_heartbeat_header.IDE = CAN_ID_STD;
  can_heartbeat_header.RTR = CAN_RTR_DATA;
  can_heartbeat_header.DLC = 8U;
  can_heartbeat_header.TransmitGlobalTime = DISABLE;

  /* 配置命令应答帧：标准数据帧ID 0x201 */
  can_response_header.StdId = CAN_RESPONSE_ID;
  can_response_header.ExtId = 0U;
  can_response_header.IDE = CAN_ID_STD;/* 使用11位标准ID */  //还有扩展帧，写法是：CAN_ID_EXT
  can_response_header.RTR = CAN_RTR_DATA;/* 发送的是数据帧 */ // 远程帧是CAN_RTR_REMOTE
  can_response_header.DLC = 3U;
  can_response_header.TransmitGlobalTime = DISABLE;

  /* 配置UDS应答帧：标准数据帧ID 0x7E8，固定发送8字节 */
  can_uds_response_header.StdId = CAN_UDS_RESPONSE_ID;
  can_uds_response_header.ExtId = 0U;
  can_uds_response_header.IDE = CAN_ID_STD;
  can_uds_response_header.RTR = CAN_RTR_DATA;
  can_uds_response_header.DLC = 8U;
  can_uds_response_header.TransmitGlobalTime = DISABLE;

  last_heartbeat_tick = HAL_GetTick();

  /* FIFO0收到新消息时，让CAN外设向CPU发出中断请求 */
  return HAL_CAN_ActivateNotification(
      can_app_handle,
      CAN_IT_RX_FIFO0_MSG_PENDING);
}//初始化

HAL_StatusTypeDef Check_Timeout(void) {
  last_time = HAL_GetTick();
  uint32_t Time_difference = last_time - previous_time;
  if (can_current_session == UDS_SESSION_EXTENDED && Time_difference >= 5000) {

    return HAL_TIMEOUT;

  }
  return HAL_OK;
}

HAL_StatusTypeDef CAN_App_Process(void)
{
  if (can_app_handle == NULL)
  {
    return HAL_ERROR;
  }

  /* 先处理PC命令，再检查心跳周期；该函数由main()反复调用 */
  if (CAN_App_ProcessPendingCommand() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 处理诊断仪发来的0x7E0 UDS请求，并在支持时发送0x7E8响应 */
  if (CAN_App_ProcessPendingUdsRequest() != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (Check_Timeout() != HAL_OK) {
    can_current_session = UDS_SESSION_DEFAULT;
  }
  if (programming_reset_pending == 1) {
    if (HAL_CAN_IsTxMessagePending(can_app_handle,programming_response_mailbox) == 0) {

      __HAL_RCC_PWR_CLK_ENABLE();//开启PWR外设时钟

      HAL_PWR_EnableBkUpAccess();//关闭备份域写保护，之后才允许写入备份寄存器

      RTC->BKP0R = BOOT_REQUEST_MAGIC;

      HAL_PWR_DisableBkUpAccess();//恢复备份域写保护，之后不允许写入备份寄存器

      NVIC_SystemReset();
    }
    if (HAL_GetTick() - start_time_of_reset_pending >= RESET_WAIT_TIMEOUT_TIME) {
      HAL_CAN_AbortTxRequest(can_app_handle,programming_response_mailbox);
      programming_reset_pending = 0U;
      programming_response_mailbox = 0U;
      can_current_session = UDS_SESSION_EXTENDED;
      previous_time = HAL_GetTick();
    }
  }
  CAN_App_SendHeartbeat();
  return HAL_OK;
}



/**
  * @brief  CAN1的FIFO0收到新消息时由HAL库调用
  * @param  hcan 触发中断的CAN句柄
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /*
   * CAN_RxHeaderTypeDef保存收到的ID、IDE、RTR、DLC等“帧头”；
   * rx_data保存该帧的0~8字节数据区，两者共同组成一帧经典CAN报文。
   */
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  /* 只处理CAN_App_Init()注册的CAN外设 */
  if ((can_app_handle == NULL) || (hcan != can_app_handle))
  {
    return;
  }

  /*
   * HAL_CAN_GetRxMessage()从硬件FIFO0取出一帧：帧头写入rx_header，
   * 数据写入rx_data；读取完成后硬件释放这个FIFO位置。
   */
  if (HAL_CAN_GetRxMessage(
          hcan,
          CAN_RX_FIFO0,
          &rx_header,
          rx_data) != HAL_OK)
  {
    return;
  }

  /* 先确认这是标准数据帧，再根据CAN ID分流处理 */
  if ((rx_header.IDE == CAN_ID_STD) &&
      (rx_header.RTR == CAN_RTR_DATA))
  {
    if ((rx_header.StdId == CAN_COMMAND_ID) &&
        (rx_header.DLC >= 2U))//可能是PING命令，所以判断是否>=2
    {
      /*
       * 0x200命令格式：byte0=命令码，byte1=令牌，
       * byte2=可选参数（灯命令中0为关、1为开）。
       * 中断只暂存，不在这里执行车灯控制或发送应答。
       */
      can_command_code = rx_data[0];
      can_command_token = rx_data[1];//令牌
      can_command_dlc = (uint8_t)rx_header.DLC;
      can_command_value =
          (rx_header.DLC >= 3U) ? rx_data[2] : 0U;

      /* 最后置位，保证主循环看到标志时其余字段已经写完 */
      can_command_pending = 1U;
    }
    else if ((rx_header.StdId == CAN_UDS_REQUEST_ID) &&
             (rx_header.DLC >= 1U))
    {
      //这两句是在把 rx_data[0] 的高4位和低4位分别取出来
      uint8_t isotp_frame_type =//高四位代表帧类型
          rx_data[0] & ISOTP_FRAME_TYPE_MASK;
      uint8_t payload_length =//第四位代表有效长度
          rx_data[0] & ISOTP_FRAME_LENGTH_MASK;

      /*
       * 当前只接受经典CAN的ISO-TP单帧：
       * 高4位必须为0，长度为1至7，并且不能超过本帧DLC。
       */
      if ((isotp_frame_type == ISOTP_SINGLE_FRAME_TYPE) &&
          (payload_length >= 1U) &&
          (payload_length <= 7U) &&
          (((uint32_t)payload_length + 1U) <= rx_header.DLC))
      {
        uds_payload_length = payload_length;
        uds_service_id = rx_data[1];
        uds_sub_function =
            (payload_length >= 2U) ? rx_data[2] : 0U;
        uds_requested_status_mask  = (payload_length >= 3U) ? rx_data[3] : 0U;
        //清除DTC
        uds_groupOfDTC[0] = (payload_length >= 4U) ? rx_data[2] : 0U;
        uds_groupOfDTC[1] = (payload_length >= 4U) ? rx_data[3] : 0U;
        uds_groupOfDTC[2] = (payload_length >= 4U) ? rx_data[4] : 0U;
        //读取DID
        Data_identifier[0] = (payload_length >= 3U) ? rx_data[2] : 0U;
        Data_identifier[1] = (payload_length >= 3U) ? rx_data[3] : 0U;
        /*
         * 最后置位，表示已经保存了一条合法ISO-TP单帧。
         * 以02 10 01为例：长度=2、服务ID=0x10、子功能=0x01。
         */
        uds_request_pending = 1U;
      }
    }
  }
}
