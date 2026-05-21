// hal.h — 硬件抽象层声明
// 外设: ePWM1/2/3, ADCA/C, SPIA(MT6701), GPIO
#ifndef HAL_USER_H
#define HAL_USER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "device.h"
#include "user_config.h"

// 包含拆分后的底层模块
#include "hal_epwm.h"
#include "hal_adc.h"
#include "hal_spi.h"
#include "hal_sci.h"
#include "hal_eqep.h"

// 函数声明

// 完整硬件初始化 (在 Device_init / Device_initGPIO 之后调用)
void HAL_init(void);


void HAL_setupInterrupt(void);



// 野火驱动板使能/禁用 (通过 GPIO 控制 EN 引脚)
void HAL_enableDRV(void);
void HAL_disableDRV(void);



// CPU Timer 用于 1ms 后台任务
void HAL_setupCPUTimer0(void);



static inline bool HAL_getCPUTimerFlag(void)
{
    return CPUTimer_getTimerOverflowStatus(CPUTIMER0_BASE);
}
static inline void HAL_clearCPUTimerFlag(void)
{
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
}

#ifdef __cplusplus
}
#endif

#endif // HAL_USER_H
