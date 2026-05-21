//###########################################################################
// foc.c — FOC 核心算法实现
// Clarke / Park / IPARK / SVPWM / PI 控制器 / 编码器处理 / 位置环
//###########################################################################
#include "foc.h"
#include <math.h>

//===========================================================================
// PI 控制器
//===========================================================================
void PI_init(PI_t *pi, float Kp, float Ki, float outMax, float outMin)
{
    pi->Kp       = Kp;
    pi->Ki       = Ki;
    pi->outMax   = outMax;
    pi->outMin   = outMin;
    pi->integral = 0.0f;
    pi->out      = 0.0f;
    pi->error    = 0.0f;
}

void PI_reset(PI_t *pi)
{
    pi->integral = 0.0f;
    pi->out      = 0.0f;
    pi->error    = 0.0f;
}

float PI_run(PI_t *pi, float ref, float fbk)
{
    pi->error = ref - fbk;

    // 比例项
    float pTerm = pi->Kp * pi->error;

    // 积分项 (带抗饱和: 仅在输出未饱和时积分)
    pi->integral += pi->Ki * pi->error;

    // 总输出
    float out = pTerm + pi->integral;

    // 限幅并回退积分 (clamping anti-windup)
    if(out > pi->outMax)
    {
        out = pi->outMax;
        pi->integral = out - pTerm;
    }
    else if(out < pi->outMin)
    {
        out = pi->outMin;
        pi->integral = out - pTerm;
    }

    pi->out = out;
    return out;
}

//===========================================================================
// Clarke 变换: abc → αβ
// Iα = Ia
// Iβ = (Ia + 2*Ib) / √3
//===========================================================================
void Clarke_run(float ia, float ib, AB_t *out)
{
    out->alpha = ia;
    out->beta  = (ia + 2.0f * ib) * MATH_ONE_BY_SQRT3;
}

//===========================================================================
// Park 变换: αβ → dq
// Id =  Iα*cos(θ) + Iβ*sin(θ)
// Iq = -Iα*sin(θ) + Iβ*cos(θ)
//===========================================================================
void Park_run(const AB_t *in, float sinTh, float cosTh, DQ_t *out)
{
    out->d =  in->alpha * cosTh + in->beta * sinTh;
    out->q = -in->alpha * sinTh + in->beta * cosTh;
}

//===========================================================================
// 逆 Park 变换: dq → αβ
// Vα = Vd*cos(θ) - Vq*sin(θ)
// Vβ = Vd*sin(θ) + Vq*cos(θ)
//===========================================================================
void IPark_run(const DQ_t *in, float sinTh, float cosTh, AB_t *out)
{
    out->alpha = in->d * cosTh - in->q * sinTh;
    out->beta  = in->d * sinTh + in->q * cosTh;
}

//===========================================================================
// SVPWM: αβ 电压 → 三相占空比 (标准七段式) （零序注入法）
// 输入: Vα, Vβ (V), vDC (V)
// 输出: duty.a/b/c (0.0~1.0)
//===========================================================================
void SVPWM_run(const AB_t *vAlBe, float vDC, ABC_t *duty)
{
    float vRef1, vRef2, vRef3;
    float vDC_inv = 1.0f / vDC;

    // 标准逆 Clarke: αβ → abc (与正 Clarke α=Ia 对应)
    // Va = Vα,  Vb = (-Vα + √3·Vβ)/2,  Vc = (-Vα - √3·Vβ)/2
    vRef1 = vAlBe->alpha;
    vRef2 = (-vAlBe->alpha + MATH_SQRT3 * vAlBe->beta) * 0.5f;
    vRef3 = (-vAlBe->alpha - MATH_SQRT3 * vAlBe->beta) * 0.5f;

    // 找最大最小值
    float vMax = vRef1;
    float vMin = vRef1;

    if(vRef2 > vMax) vMax = vRef2;
    if(vRef3 > vMax) vMax = vRef3;
    if(vRef2 < vMin) vMin = vRef2;
    if(vRef3 < vMin) vMin = vRef3;

    // 中心偏移 (零序注入, SPWM → SVPWM)
    float vOffset = -(vMax + vMin) * 0.5f;

    // 归一化到占空比 (0~1)
    duty->a = (vRef1 + vOffset) * vDC_inv + 0.5f;
    duty->b = (vRef2 + vOffset) * vDC_inv + 0.5f;
    duty->c = (vRef3 + vOffset) * vDC_inv + 0.5f;

    // 限幅
    if(duty->a > 0.98f) duty->a = 0.98f;
    if(duty->a < 0.02f) duty->a = 0.02f;
    if(duty->b > 0.98f) duty->b = 0.98f;
    if(duty->b < 0.02f) duty->b = 0.02f;
    if(duty->c > 0.98f) duty->c = 0.98f;
    if(duty->c < 0.02f) duty->c = 0.02f;
}

//===========================================================================
// 位置环 P 控制
// 输入: posRef (rad), posFbk (rad), spdLimit (rad/s)
// 输出: spdRef (rad/s)
//===========================================================================
float PosLoop_run(float posRef, float posFbk, float spdLimit)
{
    float posErr = posRef - posFbk;
    float spdRef = POS_LOOP_KP * posErr;

    // 速度限幅
    if(spdRef > spdLimit)
    {
        spdRef = spdLimit;
    }
    else if(spdRef < -spdLimit)
    {
        spdRef = -spdLimit;
    }

    return spdRef;
}

//===========================================================================
// FOC 结构体初始化
//===========================================================================
void FOC_init(FOC_t *foc)
{
    // 清零采样
    foc->iABC_A.a = 0.0f;
    foc->iABC_A.b = 0.0f;
    foc->iABC_A.c = 0.0f;
    foc->vDC_V    = MOTOR_RATED_VOLTAGE_V;

    // 清零变换结果
    foc->iAlBe.alpha = 0.0f;
    foc->iAlBe.beta  = 0.0f;
    foc->iDQ.d  = 0.0f;
    foc->iDQ.q  = 0.0f;
    foc->vDQ.d  = 0.0f;
    foc->vDQ.q  = 0.0f;
    foc->vAlBe.alpha = 0.0f;
    foc->vAlBe.beta  = 0.0f;
    foc->duty.a = 0.5f;
    foc->duty.b = 0.5f;
    foc->duty.c = 0.5f;

    // 参考值
    foc->idRef_A    = 0.0f;
    foc->iqRef_A    = 0.0f;
    foc->spdRef_rads = 0.0f;
    foc->posRef_rad  = 0.0f;

    // 角度
    foc->thetaFOC_rad = 0.0f;
    foc->spdLoopCounter = 0;

    // 初始化 PI 控制器
    PI_init(&foc->piId,  PI_ID_KP,  PI_ID_KI,  PI_ID_OUT_MAX,  PI_ID_OUT_MIN);
    PI_init(&foc->piIq,  PI_IQ_KP,  PI_IQ_KI,  PI_IQ_OUT_MAX,  PI_IQ_OUT_MIN);
    PI_init(&foc->piSpd, PI_SPD_KP, PI_SPD_KI, PI_SPD_OUT_MAX, PI_SPD_OUT_MIN);

    // 初始化编码器
    Encoder_init(&foc->enc);
}

//===========================================================================
// 快速 sincos (使用标准 math 库, F2800157 有 FPU 加速)
//===========================================================================
void FOC_sincos(float theta, float *sinVal, float *cosVal)
{
    *sinVal = sinf(theta);
    *cosVal = cosf(theta);
}
