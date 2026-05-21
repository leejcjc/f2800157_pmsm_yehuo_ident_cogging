//###########################################################################
// ident.h — 基于三角函数正交特性的机械参数在线辨识
// 参考: 吴春《基于三角函数正交特性的永磁伺服系统机械参数辨识方法》
//
// 辨识对象: 转动惯量 Jm、粘性摩擦系数 Bm、库仑摩擦力矩 Cm
// 执行频率: 1.5 kHz (与速度环同步, SPEED_LOOP_FREQ_Hz)
// 调用入口: Ident_speedLoop_run(speed_fbk) 返回 Iq_ref (A)
//###########################################################################
#ifndef IDENT_H
#define IDENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "user_config.h"

//===========================================================================
// 状态机
//===========================================================================
typedef enum {
    STATE_ROUGH_J       = 0,    // 粗测 Jm (开环加电流, 测加速度)
    STATE_RETURN_ZERO   = 1,    // 粗测后温柔刹车, 等待静止
    STATE_INT_A1        = 2,    // A1 幅值正弦激励 (积 cos 算 J, 积 sin 算 Bm/Cm)
    STATE_INT_A2        = 3,    // A2 幅值正弦激励 (同上)
    STATE_EVALUATE      = 4,    // 解耦 Bm, Cm, 判断是否继续大迭代
    STATE_ITER_DELAY    = 5,    // 迭代间歇, 让电机静止
    STATE_SUCCESS       = 6,    // 辨识完成
    STATE_TEST_TRACKING = 7,    // 性能测试 (用辨识结果跑速度跟踪)
    STATE_RESTART       = 8     // 复位重跑
} IdentState_e;

//===========================================================================
// 算法输出 (供 VOFA / Expressions 观察)
//===========================================================================
typedef struct {
    float   speed_Ref;      // 算法生成的速度参考 (rad/s)
    float   Jm_Active;      // LPF 后生效的转动惯量 (kg·m²)
    float   Bm_Active;      // 粘性摩擦系数 (N·m·s/rad)
    float   Cm_Active;      // 库仑摩擦力矩 (N·m)
    uint8_t Enable_FF;      // 摩擦前馈使能
    uint8_t Is_Finished;    // 辨识完成标志
} Ident_Output_t;

//===========================================================================
// 暴露给外部 (调试器 / VOFA / main.c) 的全局
//===========================================================================
extern volatile IdentState_e ident_state;
extern Ident_Output_t        ident_res;

// 调试器可调激励幅值
extern float rpm1, rpm2;        // 辨识激励 A1/A2 幅值 (rpm), 默认 300/600
extern float test_rpm1;         // STATE_TEST_TRACKING 速度 (rpm)

//===========================================================================
// 接口
//===========================================================================

// 速度环调用 (1.5 kHz), 返回 Iq 参考 (A)
float Ident_speedLoop_run(float speed_fbk);
// float Ident_speedLoop_run(float speed_fbk, float iq_fbk)

// 退出辨识模式时清状态 (在 RUN→IDLE 路径调用)
void  Ident_reset(void);

#ifdef __cplusplus
}
#endif

#endif // IDENT_H
