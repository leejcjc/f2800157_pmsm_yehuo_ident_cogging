//###########################################################################
// user_config.h — 所有可配置参数集中定义
// 电机: 24V PMSM (5极对, 增量式编码器)
// 编码器: 2500PPR ABZ 增量式编码器
// 驱动: 野火直流无刷电机驱动板 (三电阻采样, 信号隔离, 12~48V)
// 主控: TMS320F2800157 @ 120 MHz
//###########################################################################
#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

//===========================================================================
// 一、系统时钟
//===========================================================================
#define SYS_CLK_FREQ_Hz         120000000UL     // SYSCLK 120 MHz

// 外设配置宏已分散到各模块头文件, 此处 include 以供后续宏引用
#include "hal_epwm.h"   // PWM_FREQ_Hz, PWM_TBPRD 等
#include "hal_spi.h"    
#include "hal_eqep.h"   // EQEP1 引脚配置

//===========================================================================
// 二、野火驱动板使能 (通过 GPIO 控制驱动板 EN 引脚)
//===========================================================================
#define DRV_EN_GPIO                 7U
#define DRV_EN_CFG                  GPIO_7_GPIO7

//===========================================================================
// 三、电机参数 — 24V PMSM
//===========================================================================
#define MOTOR_TYPE                  1               // 1=PMSM
#define MOTOR_NUM_POLE_PAIRS        5U              // 5 极对
#define MOTOR_Rs_OHM                0.048f          // 定子相电阻 0.048 Ω
#define MOTOR_Ls_d_H                0.000038f       // d 轴电感 38 μH
#define MOTOR_Ls_q_H                0.000043f       // q 轴电感 43 μH
#define MOTOR_RATED_VOLTAGE_V       24.0f           // 额定电压 24V  驱动板24v
#define MOTOR_RATED_CURRENT_A       7.0f           // 额定电流 16A   驱动板10A
#define MOTOR_MAX_CURRENT_A         7.0f           // 峰值允许电流
#define MOTOR_OVER_CURRENT_A        7.0f           // 过流保护阈值
#define MOTOR_RATED_SPEED_RPM       3000.0f         // 额定转速
#define MOTOR_RATED_TORQUE_NM       (MOTOR_KT_NM_PER_A * MOTOR_RATED_CURRENT_A)
#define MOTOR_RATED_POWER_W         (MOTOR_RATED_VOLTAGE_V * MOTOR_RATED_CURRENT_A)

// 磁链 λ = 0.005415 Wb
#define MOTOR_FLUX_Wb               0.005415f
#define MOTOR_KT_NM_PER_A           (1.5f * (float)MOTOR_NUM_POLE_PAIRS * MOTOR_FLUX_Wb) //0.0406 N·m/A
#define MOTOR_KV                    0.0f

// 电机电气频率极限
#define MOTOR_FREQ_MAX_Hz           (MOTOR_NUM_POLE_PAIRS * MOTOR_RATED_SPEED_RPM / 60.0f)  // 250 Hz
#define MOTOR_FREQ_MIN_Hz           5.0f

// 开环 V/f 控制参数 (由实测拟合而来) 野火数值
#define OPEN_LOOP_VF_BOOST    1.0f    // V, 低频补偿 (≈ I_target × Rs / k)
#define OPEN_LOOP_VF_SLOPE    0.036f   // V/Hz, 与频率线性增长

// // 开环 V/f 控制参数 (由实测拟合而来) 8311数值
// #define OPEN_LOOP_VF_BOOST    0.5f    // V, 低频补偿 (≈ I_target × Rs / k)
// #define OPEN_LOOP_VF_SLOPE    0.02f   // V/Hz, 与频率线性增长

// 编码器选择 (二选一)
//===========================================================================
#define ENCODER_USE_MT6701_SPI      0U      // 0=禁用 MT6701 SPI 编码器
#define ENCODER_USE_ABZ_INCREMENTAL 1U      // 1=启用 ABZ 增量编码器 (eQEP)
 
#define ENC_ABZ_PPR                 2500U   // 编码器线数
#define ENC_ABZ_QUADRATURE_FACTOR   4U      // 4× 倍频
#define ENC_RESOLUTION_BITS         0U
#define ENC_COUNTS_PER_REV          (ENC_ABZ_PPR * ENC_ABZ_QUADRATURE_FACTOR)   // 10000
#define ENC_DIRECTION_REVERSED      0U      // 0=正向, 1=反向 (调试时若发现方向反则改 1)
 
// 编码器每电周期计数 = 10000 / 5 = 2000    5极对数 
#define ENC_COUNTS_PER_ELEC_REV     (ENC_COUNTS_PER_REV / MOTOR_NUM_POLE_PAIRS)   // 2000

//===========================================================================
// Z 相校准参数(复用 Id_align_ref / ALIGN_DURATION_MS 做强制对齐)
//===========================================================================
#define ZCAL_FIND_FREQ_HZ         10.0f        // 找 Z 的旋转频率（电频率）
#define ZCAL_FIND_TIMEOUT_MS      3000U        // 超时保护（一圈机械=5电周期，10Hz 约 0.5s 一圈）

//===========================================================================
// 四、ISR 频率
//===========================================================================
#define ISR_FREQ_Hz                 PWM_FREQ_Hz     // 15 kHz, 每个 PWM 周期一次 ISR
#define ISR_PERIOD_S                (1.0f / ISR_FREQ_Hz)

//===========================================================================
// 五、控制环增益 (三环级联: 电流环 → 速度环 → 位置环)
//===========================================================================

// ---------------- 电流环 PI (带宽 ~1 kHz, wc = 2π*1000 ≈ 6283 rad/s) ---------------

// D轴: Kp_d = Ld * wc = 38e-6 * 6283 ≈ 0.2388
// Ki (连续) = Rs * wc = 0.048 * 6283 ≈ 301.6
// Ki_discrete = Ki * Ts = 301.6 / 15000 ≈ 0.0201
#define PI_ID_KP                    0.2388f
#define PI_ID_KI                    0.0201f
//#define PI_ID_OUT_MAX               (0.95f * MOTOR_RATED_VOLTAGE_V / 1.732f)  // ≈6.58V
//#define PI_ID_OUT_MIN               (-PI_ID_OUT_MAX)
#define PI_ID_OUT_MAX               5.0f
#define PI_ID_OUT_MIN               (-PI_ID_OUT_MAX)

// Q轴: Kp_q = Lq * wc = 43e-6 * 6283 ≈ 0.2702
#define PI_IQ_KP                    0.2702f
#define PI_IQ_KI                    0.0201f
#define PI_IQ_OUT_MAX               PI_ID_OUT_MAX
#define PI_IQ_OUT_MIN               (-PI_IQ_OUT_MAX)


// ----------- 速度环 PI (带宽 100 Hz, 分频 10:1 = 1.5kHz 执行) ---------------
// 速度环输出 = Iq_ref (A), 输入 = speed_error (rad/s)
// Kp_speed = J*wc/Kt ≈ 5e-6*628/0.05 = 0.063 (偏小,加大到经验初值)


#define SPEED_LOOP_EXEC_RATIO       10U             // 每 10 次 ISR 执行一次速度环
#define SPEED_LOOP_FREQ_Hz          (ISR_FREQ_Hz / (float)SPEED_LOOP_EXEC_RATIO)  // 1500 Hz
#define PI_SPD_KP                   0.015f          // A/(rad/s), 空载保守初值
#define PI_SPD_KI                   0.00012f         // 积分增益 (已含 Ts_spd), 先小后调
#define PI_SPD_OUT_MAX              1.5f      //原来MOTOR_MAX_CURRENT_A (7A)
#define PI_SPD_OUT_MIN              (-PI_SPD_OUT_MAX)

// ------------- 位置环 P (带宽 30 Hz, 与速度环同频执行) --------------------
// 输出 = speed_ref (rad/s), 输入 = position_error (rad)
// Kp_pos ≈ 2π*30 ≈ 188 rad/s per rad
#define POS_LOOP_KP                 50.0f           // (rad/s) / rad, 机器人关节偏保守
#define POS_LOOP_FF_GAIN            0.0f            // 速度前馈增益 (0=关闭)
#define POS_LOOP_SPD_LIMIT          (MOTOR_RATED_SPEED_RPM * 2.0f * 3.14159f / 60.0f)  // rad/s


#define ENC_CALIBRATED_OFFSET       0               // 编码器校准偏移量 (一次性标定值, 对齐后 foc.enc.offset 读数)

// 六、对齐 / 启动参数
#define Id_align_ref                1.5f            // 对齐电流 (d轴) - 调大以克服齿槽转矩，让其死死对准0度
#define ALIGN_DURATION_MS           500U            // 原单步对齐保持时间 ms
//===========================================================================
// 七、保护阈值
//===========================================================================
#define OVER_VOLTAGE_V              30.0f           // 母线过压
#define UNDER_VOLTAGE_V             18.0f           // 母线欠压
#define OVER_CURRENT_A              MOTOR_OVER_CURRENT_A  // 5.0A

//===========================================================================
// 八、数学常数
//===========================================================================
#define MATH_PI                     3.14159265f
#define MATH_TWO_PI                 6.28318530f
#define MATH_ONE_BY_THREE           0.33333333f
#define MATH_TWO_BY_THREE           0.66666666f
#define MATH_ONE_BY_SQRT3           0.57735026f
#define MATH_SQRT2                  1.41421356f
#define MATH_SQRT3                  1.73205080f
#define MATH_SQRT3_BY_2             0.86602540f



#ifdef __cplusplus
}
#endif

#endif // USER_CONFIG_H
