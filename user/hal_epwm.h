#ifndef EPWM_USER_H
#define EPWM_USER_H

#include "device.h"

//===========================================================================
// PWM 配置 (ePWM1 / ePWM2 / ePWM3, 中心对齐互补)
//===========================================================================
#define PWM_FREQ_kHz            15.0f           // PWM 开关频率 kHz
#define PWM_FREQ_Hz             (PWM_FREQ_kHz * 1000.0f)
#define PWM_TBPRD               4000U           // 120MHz / (2 * 15kHz) = 4000
#define PWM_DEADBAND_NS         500U            // 死区 500 ns (可调)
#define PWM_DEADBAND_CNT        60U             // 500ns * 120MHz = 60 个时钟

// ePWM base 地址
#define MTR_PWM_A_BASE          EPWM1_BASE      // A 相
#define MTR_PWM_B_BASE          EPWM2_BASE      // B 相
#define MTR_PWM_C_BASE          EPWM3_BASE      // C 相

// PWM GPIO 引脚配置 (ePWM1A/B, ePWM2A/B, ePWM3A/B — 互补输出)
#define MTR_PWM_AH_GPIO         0U
#define MTR_PWM_AH_CFG          GPIO_0_EPWM1_A
#define MTR_PWM_AL_GPIO         1U
#define MTR_PWM_AL_CFG          GPIO_1_EPWM1_B
#define MTR_PWM_BH_GPIO         2U
#define MTR_PWM_BH_CFG          GPIO_2_EPWM2_A
#define MTR_PWM_BL_GPIO         3U
#define MTR_PWM_BL_CFG          GPIO_3_EPWM2_B
#define MTR_PWM_CH_GPIO         4U
#define MTR_PWM_CH_CFG          GPIO_4_EPWM3_A
#define MTR_PWM_CL_GPIO         5U
#define MTR_PWM_CL_CFG          GPIO_5_EPWM3_B

// ePWM 配置: 中心对齐互补模式, 15kHz, 带死区
// 初始化 ePWM 模块（内部调用 HAL_setupOnePWM 三相）
void EPWM_init(void);


// PWM 占空比更新 (duty: 0.0~1.0)
void HAL_writePWM(float dutyA, float dutyB, float dutyC);
// PWM 输出使能/禁用
void HAL_enablePWMoutput(void);
void HAL_disablePWMoutput(void);

#endif // EPWM_USER_H
