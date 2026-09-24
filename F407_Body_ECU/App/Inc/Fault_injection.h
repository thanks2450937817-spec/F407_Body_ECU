#ifndef F407_BODY_ECU_FAULT_INJECTION_H
#define F407_BODY_ECU_FAULT_INJECTION_H

/*
 * 读取KEY1并更新灯故障注入状态。
 * 本模块只负责产生测试条件，不直接拥有或操作PA7车灯输出。
 */
void FaultInjection_ProcessButton(void);

#endif /* F407_BODY_ECU_FAULT_INJECTION_H */
