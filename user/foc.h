//###########################################################################
// foc.h — FOC 算法数据结构与函数声明
// 包含: PI 控制器, Clarke, Park, IPARK, SVPWM, 编码器处理, 位置环
//###########################################################################
#ifndef FOC_H
#define FOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "user_config.h"
#include "hal_spi.h"

//===========================================================================
// 数据结构
//===========================================================================

// 二维向量
typedef struct {
    float alpha;
    float beta;
} AB_t;

typedef struct {
    float d;
    float q;
} DQ_t;

typedef struct {
    float a;
    float b;
    float c;
} ABC_t;

// PI 控制器
typedef struct {
    float Kp;
    float Ki;
    float outMax;
    float outMin;
    float integral;
    float out;
    float error;
} PI_t;

// FOC 主控制结构体
typedef struct {
    // 采样数据
    ABC_t   iABC_A;             // 三相电流 (A)
    float   vDC_V;              // 母线电压 (V)

    // 坐标变换
    AB_t    iAlBe;              // Clarke 输出 (Iα, Iβ)
    DQ_t    iDQ;                // Park 输出 (Id, Iq)
    DQ_t    vDQ;                // d-q 轴电压指令
    AB_t    vAlBe;              // IPARK 输出 (Vα, Vβ)
    ABC_t   duty;               // SVPWM 占空比 (0~1)

    // PI 控制器
    PI_t    piId;               // d 轴电流环
    PI_t    piIq;               // q 轴电流环
    PI_t    piSpd;              // 速度环
    // 位置环用 P 控制, 不需要完整 PI 结构

    // 参考值
    float   idRef_A;            // d 轴电流参考 (通常=0)
    float   iqRef_A;            // q 轴电流参考 (转矩指令)
    float   spdRef_rads;        // 速度参考 (rad/s)
    float   posRef_rad;         // 位置参考 (rad, 机械)

    // 编码器
    Encoder_t enc;

    // 角度
    float   thetaFOC_rad;       // FOC 使用的电角度

    // 状态
    uint16_t spdLoopCounter;    // 速度环分频计数

} FOC_t;

// 电机控制状态机
typedef enum {
    MTR_STATE_IDLE       = 0,   // 空闲
    MTR_STATE_OFFSET_CAL = 1,   // 偏置校准
    MTR_STATE_ALIGN      = 2,   // 转子对齐 (d轴对准)
    MTR_STATE_Z_CAL      = 3,   // Z相校准
    MTR_STATE_RUN        = 4,   // 闭环运行
    MTR_STATE_FAULT      = 5,   // 故障
    MTR_STATE_STOP       = 6    // 停机
} MotorState_e;

// 控制模式
typedef enum {
    CTRL_MODE_TORQUE   = 0,     // 转矩环 (Iq 直给)
    CTRL_MODE_SPEED    = 1,     // 速度环
    CTRL_MODE_POSITION = 2,     // 位置环
    CTRL_MODE_IDENT    = 3      // 机械参数辨识 (Jm/Bm/Cm)
} CtrlMode_e;

// 故障标志位
typedef union {
    uint16_t all;
    struct {
        uint16_t overVoltage    : 1;
        uint16_t underVoltage   : 1;
        uint16_t overCurrent    : 1;
        uint16_t encoderFault   : 1;
        uint16_t driverFault    : 1;
        uint16_t reserved       : 11;
    } bit;
} FaultFlag_t;

//===========================================================================
// 函数声明
//===========================================================================

// PI 控制器
void    PI_init(PI_t *pi, float Kp, float Ki, float outMax, float outMin);
void    PI_reset(PI_t *pi);
float   PI_run(PI_t *pi, float ref, float fbk);

// Clarke 变换: abc → αβ (等功率变换)
void    Clarke_run(float ia, float ib, AB_t *out);

// Park 变换: αβ → dq
void    Park_run(const AB_t *in, float sinTh, float cosTh, DQ_t *out);

// 逆 Park 变换: dq → αβ
void    IPark_run(const DQ_t *in, float sinTh, float cosTh, AB_t *out);

// SVPWM: αβ 电压 → 三相占空比
void    SVPWM_run(const AB_t *vAlBe, float vDC, ABC_t *duty);

// 位置环 P 控制
float   PosLoop_run(float posRef, float posFbk, float spdLimit);

// FOC 完整初始化
void    FOC_init(FOC_t *foc);

// 快速 sin/cos (查表或 math.h)
void    FOC_sincos(float theta, float *sinVal, float *cosVal);

#ifdef __cplusplus
}
#endif

#endif // FOC_H
