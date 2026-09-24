#ifndef APP_BODY_CONTROL_H
#define APP_BODY_CONTROL_H

#include <stdint.h>

/*
 * 车身执行器模块的公开接口。
 * .h文件只声明“外部可以调用什么”，PA7等硬件细节保留在.c文件中。
 */

/**
  * @brief  初始化车身执行器，并让车灯保持关闭
  * @retval None
  */
void BodyControl_Init(void);

/**
  * @brief  设置车灯逻辑状态
  * @param  lamp_on 0表示关灯，非0表示开灯
  * @retval None
  */
void BodyControl_SetLamp(uint8_t lamp_on);

/**
  * @brief  读取当前车灯逻辑状态
  * @retval 0表示关闭，1表示打开
  */
uint8_t BodyControl_GetLampState(void);

/**
  * @brief  开启或关闭车灯故障注入
  * @param  injection_active 0表示关闭注入，非0表示强制实际灯输出关闭
  * @retval None
  */
void BodyControl_SetLampFaultInjection(uint8_t injection_active);

/**
  * @brief  比较车灯目标状态与PA7实际电平
  * @retval 0表示本次检测正常，1表示本次检测失败
  */
uint8_t BodyControl_CheckLampFault(void);

#endif /* APP_BODY_CONTROL_H */
