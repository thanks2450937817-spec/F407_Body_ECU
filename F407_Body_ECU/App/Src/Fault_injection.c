#include "Fault_injection.h"
#include "body_control.h"
#include "main.h"

/* KEY1连接到PE3，当前硬件按下时读取为低电平 */
#define FAULT_BUTTON_GPIO_PORT      GPIOE
#define FAULT_BUTTON_GPIO_PIN       GPIO_PIN_3
#define FAULT_BUTTON_PRESSED_LEVEL  GPIO_PIN_RESET

void FaultInjection_ProcessButton(void)
{
  GPIO_PinState button_level =
      HAL_GPIO_ReadPin(FAULT_BUTTON_GPIO_PORT, FAULT_BUTTON_GPIO_PIN);

  /*
   * 按下时只开启故障注入，不直接操作PA7；
   * 松开时关闭注入，由body_control恢复车灯原来的目标状态。
   */
  BodyControl_SetLampFaultInjection(
      (button_level == FAULT_BUTTON_PRESSED_LEVEL) ? 1U : 0U);
}

