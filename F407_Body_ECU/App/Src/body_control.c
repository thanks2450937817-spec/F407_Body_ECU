/*
 * 车身控制应用模块，不由STM32CubeMX生成。
 *
 * 本模块把“车灯开/关”这种业务含义与PA7的真实电平隔离开：
 * CAN应用层只传入逻辑状态0/1，不需要知道硬件是低电平点亮。
 * 以后更换引脚或改成高电平点亮时，只需要修改本文件。
 */
#include "body_control.h"
#include "main.h"

/* 开发板车灯连接到PA7，并且是低电平点亮 */
#define BODY_LAMP_GPIO_PORT  GPIOA
#define BODY_LAMP_GPIO_PIN   GPIO_PIN_7
#define BODY_LAMP_ON_LEVEL   GPIO_PIN_RESET
#define BODY_LAMP_OFF_LEVEL  GPIO_PIN_SET

/* 对外统一使用逻辑状态：0表示关闭，1表示打开 */
static uint8_t body_lamp_state = 0U;

/* 1表示正在模拟灯输出故障；它只影响实际输出，不修改目标状态 */
static uint8_t lamp_fault_injection_active = 0U;

/**
 * @brief  根据目标状态和故障注入状态更新PA7实际输出
 * @retval None
 */
static void BodyControl_ApplyLampOutput(void)
{
    //如果故障注入没有开启且目标状态为1则实际状态改为1，
    //若故障输入开启，则无论目标状态为何均不开灯等
  uint8_t actual_lamp_on =
      ((body_lamp_state != 0U) &&
       (lamp_fault_injection_active == 0U)) ? 1U : 0U;

  HAL_GPIO_WritePin(
      BODY_LAMP_GPIO_PORT,
      BODY_LAMP_GPIO_PIN,
      (actual_lamp_on != 0U) ?
          BODY_LAMP_ON_LEVEL : BODY_LAMP_OFF_LEVEL);
}

void BodyControl_Init(void)
{
  /* 软件记录的状态和GPIO实际电平都初始化为“关闭” */
  body_lamp_state = 0U;
  lamp_fault_injection_active = 0U;
  BodyControl_ApplyLampOutput();
}

void BodyControl_SetLamp(uint8_t lamp_on)
{
  /* 无论调用者传入什么非零值，内部都统一保存为1 */
  body_lamp_state = (lamp_on != 0U) ? 1U : 0U;
  BodyControl_ApplyLampOutput();
}

void BodyControl_SetLampFaultInjection(uint8_t injection_active)
{
  uint8_t normalized_state =
      (injection_active != 0U) ? 1U : 0U;

  /* 状态未变化时无需重复写GPIO */
  if (normalized_state == lamp_fault_injection_active)
  {
    return;
  }

  lamp_fault_injection_active = normalized_state;
  BodyControl_ApplyLampOutput();
}

uint8_t BodyControl_CheckLampFault(void)
{
  /*
   * body_lamp_state保存逻辑目标，PA7保存实际电平。
   * 由于LED低电平点亮，必须先把逻辑目标转换成期望电平再比较。
   */
  GPIO_PinState expected_level =
      (body_lamp_state != 0U) ?
          BODY_LAMP_ON_LEVEL : BODY_LAMP_OFF_LEVEL;
  GPIO_PinState actual_level =
      HAL_GPIO_ReadPin(BODY_LAMP_GPIO_PORT, BODY_LAMP_GPIO_PIN);

  return (actual_level != expected_level) ? 1U : 0U;
}

uint8_t BodyControl_GetLampState(void)
{
  /* 返回业务层逻辑状态，不直接读取PA7的高低电平 */
  return body_lamp_state;
}
