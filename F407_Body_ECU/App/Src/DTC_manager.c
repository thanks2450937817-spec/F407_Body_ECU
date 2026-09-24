#include "DTC_manager.h"
//x/2wx 0x08060000 查内存的
/*
 * 当前先管理一个由软件模拟的灯驱动故障：B1000-01。
 *
 * B1000编码在前两个字节0x9000中：
 *   最高两位10表示车身系统B，后面的位表示故障编号1000。
 * 第三个字节0x01是故障类型字节FTB。
 * ECU通过CAN发送的是90 00 01，诊断仪负责将其显示成B1000-01。
 */

/* Sector 7用于保存已确认DTC */
#define DTC_FLASH_START_ADDRESS   0x08060000U

/* 用于判断Flash中是否存在有效DTC记录 */
#define DTC_FLASH_VALID_MAGIC     0x44544331U

/* confirmedDTC保存在起始地址后面的第二个32位字中 ，STM32的地址每增加1，表示向后移动1字节*/
#define DTC_FLASH_STATUS_ADDRESS  (DTC_FLASH_START_ADDRESS + 4U)


#define DTC_LAMP_OUTPUT_ID                 0x9000U
#define DTC_LAMP_OUTPUT_FTB                0x01U

/* 当前教学版本的DTC初始状态 */
#define DTC_STATUS_INITIAL                 0x00U

/* 每10 ms采样；累计100个样本后，以5%的失败比例作为确认条件 */
#define DTC_CONFIRMATION_SAMPLE_COUNT      1000U
#define DTC_CONFIRMATION_FAILURE_COUNT     100U

/* 当前阶段准备使用的DTC状态位掩码 */
#define DTC_STATUS_TEST_FAILED             0x01U
#define DTC_STATUS_PENDING_DTC             0x04U
#define DTC_STATUS_CONFIRMED_DTC           0x08U


static HAL_StatusTypeDef DTC_Manager_SaveConfirmedToFlash(void);

/*
 * 将同一个故障的所有信息放在一条记录中。
 * 不再单独保存P/C/B/U；需要时可直接从id最高两位解析。
 */


/* 灯驱动故障的唯一内部记录，暂时只保存在RAM中 */
DTC_Record lamp_output_dtc =
{
  .id = DTC_LAMP_OUTPUT_ID,
  .ftb = DTC_LAMP_OUTPUT_FTB,
  .status = DTC_STATUS_INITIAL,
  .window_sample_count = 0U,
  .window_failure_count = 0U
};

void DTC_Manager_Init(void)
{
  /* 初始化运行状态；DTC编号和FTB在程序运行期间保持不变 */
  lamp_output_dtc.status = DTC_STATUS_INITIAL;
  lamp_output_dtc.window_sample_count = 0U;
  lamp_output_dtc.window_failure_count = 0U;
  uint32_t saved_magic =
  *(const uint32_t *)DTC_FLASH_START_ADDRESS;
  /* 只有有效标记正确，才继续读取保存的状态 */
  if (saved_magic == DTC_FLASH_VALID_MAGIC)
  {
    uint32_t saved_status =
        *(const uint32_t *)DTC_FLASH_STATUS_ADDRESS;

    /* 掉电后只恢复confirmedDTC，不恢复当前testFailed */
    if (saved_status == DTC_STATUS_CONFIRMED_DTC)
    {
      lamp_output_dtc.status = DTC_STATUS_CONFIRMED_DTC;
    }
  }
}

void DTC_Manager_ReportTestResult(uint8_t test_failed)
{
  /* testFailed只表示当前这一次10 ms检测的结果 */
  if (test_failed != 0U)
  {
    lamp_output_dtc.status |= DTC_STATUS_TEST_FAILED;
  }
  else
  {
    /* 只清除testFailed，不能覆盖已经锁存的confirmedDTC */
    lamp_output_dtc.status &=
        (uint8_t)(~DTC_STATUS_TEST_FAILED);//取反 ~
  }

  /* confirmedDTC已经锁存后，仍更新testFailed，但不再重复统计比例 */
  if ((lamp_output_dtc.status & DTC_STATUS_CONFIRMED_DTC) != 0U)
  {
    return;
  }

  lamp_output_dtc.window_sample_count++;
  if (test_failed != 0U)
  {
    lamp_output_dtc.window_failure_count++;
  }

  if (lamp_output_dtc.window_sample_count >=
      DTC_CONFIRMATION_SAMPLE_COUNT)
  {
    /*
     * 固定100样本窗口内至少5次失败，即失败比例达到5%。
     * confirmedDTC使用按位或锁存，当前检测恢复后也不会自动清除。
     */
    if (lamp_output_dtc.window_failure_count >=
        DTC_CONFIRMATION_FAILURE_COUNT)
    {
      lamp_output_dtc.status |= DTC_STATUS_CONFIRMED_DTC;//确定故障
      //存入flash
      if (DTC_Manager_SaveConfirmedToFlash() != HAL_OK){return;}
    }

    /* 开始下一个独立的100样本统计窗口 */
    lamp_output_dtc.window_sample_count = 0U;
    lamp_output_dtc.window_failure_count = 0U;
  }
}
HAL_StatusTypeDef DTC_Manager_ClearAll(void) {

  FLASH_EraseInitTypeDef flash_erase_config = {0};//配置Flash 擦除配置表
  flash_erase_config.TypeErase = FLASH_TYPEERASE_SECTORS;//按扇区擦除
  flash_erase_config.Sector = FLASH_SECTOR_7;//目标是 Sector 7
  flash_erase_config.NbSectors = 1U;//只擦除一个扇区
  flash_erase_config.VoltageRange = FLASH_VOLTAGE_RANGE_3;//适配开发板的3.3V供电

  uint32_t all_sectors_erased = 0xFFFFFFFFU;

  HAL_StatusTypeDef result_unlock = HAL_FLASH_Unlock();//解锁FLASH
  if (result_unlock != HAL_OK) {return result_unlock;}//解锁

  HAL_StatusTypeDef result_Erase = HAL_FLASHEx_Erase(&flash_erase_config,&all_sectors_erased);
  if (result_Erase != HAL_OK) {//这个函数是用来擦除的
    HAL_FLASH_Lock();
    return result_Erase;
  }
  HAL_StatusTypeDef result_lock = HAL_FLASH_Lock();
  if (result_lock != HAL_OK) {return result_lock;}//锁

  lamp_output_dtc.status = DTC_STATUS_INITIAL;
  lamp_output_dtc.window_sample_count = 0U;
  lamp_output_dtc.window_failure_count = 0U;
  return HAL_OK;

}

static HAL_StatusTypeDef  DTC_Manager_SaveConfirmedToFlash(void) {
  uint32_t saved_magic = *(uint32_t *)DTC_FLASH_START_ADDRESS;
  if (saved_magic == DTC_FLASH_VALID_MAGIC) {
      return HAL_OK;
  }
  HAL_StatusTypeDef result_unlock = HAL_FLASH_Unlock();//解锁FLASH
  if (result_unlock != HAL_OK) {return result_unlock;}
  const HAL_StatusTypeDef write_status_result = //写入确定故障
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
              DTC_FLASH_STATUS_ADDRESS,
                DTC_STATUS_CONFIRMED_DTC);
if (write_status_result != HAL_OK) {HAL_FLASH_Lock();return write_status_result;}
  const HAL_StatusTypeDef write_valid_result =//写入DTC存储标志
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
             DTC_FLASH_START_ADDRESS,
               DTC_FLASH_VALID_MAGIC);
  if (write_valid_result != HAL_OK) {HAL_FLASH_Lock();return write_valid_result;}
  result_unlock = HAL_FLASH_Lock();//锁FLASH
  if (result_unlock != HAL_OK) {return result_unlock;}
  return HAL_OK;
}

