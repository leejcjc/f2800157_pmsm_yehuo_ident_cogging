/*
 * cogging.c
 *
 *  Created on: 2026年5月21日
 *      Author: Lenovo
 */
//###########################################################################
// cogging.c - Position LUT cogging torque compensation
//###########################################################################
#include "cogging.h"
#include <math.h>

// 本文件实现“一圈机械角度 -> 补偿电流”的齿槽力矩查表补偿。
// 基本流程:
// 1. 正转/反转低速运行时，按机械位置记录 q 轴电流残差。
// 2. 生成 LUT 时，把正反转数据合并、去均值、平滑。
// 3. 闭环运行时，根据当前位置查表并插值，叠加到 Iq_ref_A。

// 这些变量暴露给 CCS Expressions / VOFA 观察和手动控制。
// cog_enable = 0: 允许记录样本；cog_enable = 1: 启用补偿输出。
volatile uint16_t cog_enable       = 0U;
volatile uint16_t cog_ready        = 0U;
volatile uint16_t cog_record_mode  = COG_RECORD_STOP;
volatile uint16_t cog_generate_req = 0U;
volatile uint16_t cog_reset_req    = 0U;
volatile uint16_t cog_zero_count   = 0U;
volatile uint16_t cog_last_index   = 0U;
volatile uint16_t cog_missing_bins = COG_LUT_SIZE;

volatile uint32_t cog_pos_samples  = 0UL;
volatile uint32_t cog_neg_samples  = 0UL;

volatile float cog_gain       = 0.2f;
volatile float cog_iqComp_A   = 0.0f;
volatile float cog_iqRaw_A    = 0.0f;
volatile float cog_iqRecord_A = 0.0f;

//测试
volatile uint16_t cog_raw_dbg      = 0U;
volatile uint16_t cog_zero_dbg     = 0U;
volatile int32_t  cog_diff_dbg     = 0L;
volatile int32_t  cog_mod_dbg      = 0L;
volatile uint16_t cog_posCount_dbg = 0U;
volatile uint32_t cog_idx32_dbg    = 0UL;
volatile uint16_t cog_idx_dbg      = 0U;

// 齿槽补偿需要较大的数组，不能放默认 .bss 小 RAM 区。
// 这里把数组放到 linker 文件里的 cog_data 段，再由 cmd 文件映射到 RAMLS0D。
#pragma DATA_SECTION(cog_sum_pos, "cog_data");
static float    cog_sum_pos[COG_LUT_SIZE];     // 正转累计
#pragma DATA_SECTION(cog_sum_neg, "cog_data");
static float    cog_sum_neg[COG_LUT_SIZE];     // 反转累计
#pragma DATA_SECTION(cog_cnt_pos, "cog_data");
static uint16_t cog_cnt_pos[COG_LUT_SIZE];    // 正转每个位置采了多少次
#pragma DATA_SECTION(cog_cnt_neg, "cog_data");
static uint16_t cog_cnt_neg[COG_LUT_SIZE];    // 反转每个位置采了多少次
#pragma DATA_SECTION(cog_lut, "cog_data");
static float    cog_lut[COG_LUT_SIZE];     // 最终补偿表 (单位 A)，在 ISR 中使用
#pragma DATA_SECTION(cog_tmp, "cog_data");
static float    cog_tmp[COG_LUT_SIZE]; // 生成 LUT 时的平滑临时表

// 简单限幅，防止记录值或补偿值过大。
static float Cogging_limit(float x, float minVal, float maxVal)
{
    if(x > maxVal)
    {
        return maxVal;
    }
    if(x < minVal)
    {
        return minVal;
    }
    return x;
}

// 把任意编码器计数折回到 [0, 10000)。
static uint16_t Cogging_wrapCount(int32_t count)
{
    count %= (int32_t)ENC_COUNTS_PER_REV;
    if(count < 0)
    {
        count += (int32_t)ENC_COUNTS_PER_REV;
    }
    return (uint16_t)count;
}

//-------新增-------
static uint16_t Cogging_posCountFromRaw(uint16_t rawCount)
{
    uint32_t cpr;
    uint32_t raw;
    uint32_t zero;
    uint32_t pos;

    cpr  = (uint32_t)ENC_COUNTS_PER_REV;
    raw  = (uint32_t)rawCount;
    zero = (uint32_t)cog_zero_count;

    if(zero >= cpr)
    {
        zero = zero % cpr;
    }

    if(raw >= zero)
    {
        pos = raw - zero;
    }
    else
    {
        pos = raw + cpr - zero;
    }

    if(pos >= cpr)
    {
        pos = pos % cpr;
    }

    return (uint16_t)pos;
}

// 根据原始编码器计数计算 LUT 下标。  索引标号
// cog_zero_count 是 Z 相确定的齿槽零点，使每次上电后的表位置一致。
// static uint16_t Cogging_indexFromRaw(uint16_t rawCount)
// {
//     uint16_t posCount = Cogging_wrapCount((int32_t)rawCount -
//                                           (int32_t)cog_zero_count);
//     uint32_t idx = ((uint32_t)posCount * (uint32_t)COG_LUT_SIZE) /
//                    (uint32_t)ENC_COUNTS_PER_REV;
   
//     if(idx >= (uint32_t)COG_LUT_SIZE)
//     {
//         idx = (uint32_t)COG_LUT_SIZE - 1UL;
//     }

//     //测试
//     cog_posCount_dbg = posCount;
//     cog_idx_dbg = (uint16_t)idx;


//     return (uint16_t)idx;
// }


// static uint16_t Cogging_indexFromRaw(uint16_t rawCount)
// {
//     int32_t diff;
//     int32_t mod;
//     uint32_t idx32;

//     cog_raw_dbg  = rawCount;
//     cog_zero_dbg = cog_zero_count;

//     diff = (int32_t)rawCount - (int32_t)cog_zero_count;
//     cog_diff_dbg = diff;

//     mod = diff % (int32_t)ENC_COUNTS_PER_REV;
//     if(mod < 0L)
//     {
//         mod += (int32_t)ENC_COUNTS_PER_REV;
//     }

//     cog_mod_dbg = mod;
//     cog_posCount_dbg = (uint16_t)mod;

//     idx32 = ((uint32_t)mod * (uint32_t)COG_LUT_SIZE) /
//             (uint32_t)ENC_COUNTS_PER_REV;

//     cog_idx32_dbg = idx32;

//     if(idx32 >= (uint32_t)COG_LUT_SIZE)
//     {
//         idx32 = (uint32_t)COG_LUT_SIZE - 1UL;
//     }

//     cog_idx_dbg = (uint16_t)idx32;

//     return (uint16_t)idx32;
// }

static uint16_t Cogging_indexFromRaw(uint16_t rawCount)//-------修改-------
{
    uint16_t posCount;
    uint32_t idx;

    posCount = Cogging_posCountFromRaw(rawCount);

    idx = ((uint32_t)posCount * (uint32_t)COG_LUT_SIZE) /
          (uint32_t)ENC_COUNTS_PER_REV;

    if(idx >= (uint32_t)COG_LUT_SIZE)
    {
        idx = (uint32_t)COG_LUT_SIZE - 1UL;
    }

    cog_raw_dbg      = rawCount;
    cog_zero_dbg     = cog_zero_count;
    cog_diff_dbg     = (int32_t)rawCount - (int32_t)cog_zero_count;
    cog_mod_dbg      = (int32_t)posCount;
    cog_posCount_dbg = posCount;
    cog_idx32_dbg    = idx;
    cog_idx_dbg      = (uint16_t)idx;

    return (uint16_t)idx;
}



// 根据已经记录的正转/反转样本生成最终补偿表。
// 注意: 该函数在主循环中执行；生成期间先把 cog_ready 清零，避免 ISR 读到半成品表。
static void Cogging_generateLut(void)
{
    uint16_t k;
    uint16_t missing = 0U;
    float mean = 0.0f;

    cog_ready = 0U;
    Cogging_clearRuntimeOutput();

    for(k = 0U; k < COG_LUT_SIZE; k++)
    {
        float value = 0.0f;
        uint16_t hasPos = (cog_cnt_pos[k] > 0U) ? 1U : 0U;
        uint16_t hasNeg = (cog_cnt_neg[k] > 0U) ? 1U : 0U;

        // 同一位置如果正反转都有数据，就取平均，尽量抵消摩擦方向项。
        if(hasPos && hasNeg)
        {
            float pos = cog_sum_pos[k] / (float)cog_cnt_pos[k];
            float neg = cog_sum_neg[k] / (float)cog_cnt_neg[k];
            value = 0.5f * (pos + neg);
        }
        else if(hasPos)
        {
            value = cog_sum_pos[k] / (float)cog_cnt_pos[k];
        }
        else if(hasNeg)
        {
            value = cog_sum_neg[k] / (float)cog_cnt_neg[k];
        }
        else
        {
            missing++;
        }

        value = Cogging_limit(value,
                              -COG_COMP_CURRENT_LIMIT_A,
                               COG_COMP_CURRENT_LIMIT_A);
        cog_lut[k] = value;
        mean += value;
    }

    // 如果有位置点完全没采到，先不允许启用补偿。
    // 调试时看 cog_missing_bins，等它变成 0 再生成/启用。
    cog_missing_bins = missing;
    if(missing > 0U)
    {
        cog_ready = 0U;
        return;
    }

    mean /= (float)COG_LUT_SIZE;

    // 去掉整圈平均值，只保留随位置变化的齿槽分量。
    for(k = 0U; k < COG_LUT_SIZE; k++)
    {
        cog_lut[k] -= mean;
    }

    // 环形移动平均平滑，避免相邻表项跳变太硬。
    for(k = 0U; k < COG_LUT_SIZE; k++)
    {
        int16_t offset;
        float sum = 0.0f;
        uint16_t cnt = 0U;

        for(offset = -(int16_t)COG_SMOOTH_RADIUS;
            offset <= (int16_t)COG_SMOOTH_RADIUS;
            offset++)
        {
            int32_t idx = (int32_t)k + (int32_t)offset;
            idx %= (int32_t)COG_LUT_SIZE;
            if(idx < 0)
            {
                idx += (int32_t)COG_LUT_SIZE;
            }

            sum += cog_lut[(uint16_t)idx];
            cnt++;
        }

        cog_tmp[k] = sum / (float)cnt;
    }

    for(k = 0U; k < COG_LUT_SIZE; k++)
    {
        cog_lut[k] = Cogging_limit(cog_tmp[k],
                                   -COG_COMP_CURRENT_LIMIT_A,
                                    COG_COMP_CURRENT_LIMIT_A);
    }

    cog_ready = 1U;
}

// 上电初始化齿槽模块。
void Cogging_init(void)
{
    Cogging_reset();
}

// 清空记录数据和补偿输出。
// 复位开始就先关闭 ready/enable，避免 ISR 使用正在清空的数据。
void Cogging_reset(void)
{
    uint16_t k;

    cog_enable       = 0U;
    cog_ready        = 0U;
    cog_record_mode  = COG_RECORD_STOP;
    cog_generate_req = 0U;
    cog_reset_req    = 0U;
    cog_gain         = 0.0f;
    Cogging_clearRuntimeOutput();

    for(k = 0U; k < COG_LUT_SIZE; k++)
    {
        cog_sum_pos[k] = 0.0f;
        cog_sum_neg[k] = 0.0f;
        cog_cnt_pos[k] = 0U;
        cog_cnt_neg[k] = 0U;
        cog_lut[k]     = 0.0f;
        cog_tmp[k]     = 0.0f;
    }

    cog_last_index   = 0U;
    cog_missing_bins = COG_LUT_SIZE;
    cog_pos_samples  = 0UL;
    cog_neg_samples  = 0UL;
    cog_iqRecord_A   = 0.0f;
}

// 主循环调用，用于响应 CCS Expressions 手动置位的请求。
// cog_reset_req = 1: 清空所有齿槽数据。
// cog_generate_req = 1: 用当前采样数据生成 LUT。
void Cogging_serviceRequests(void)
{
    if(cog_reset_req != 0U)
    {
        Cogging_reset();
    }

    if(cog_generate_req != 0U)
    {
        cog_generate_req = 0U;
        cog_record_mode = COG_RECORD_STOP;
        Cogging_generateLut();
    }
}

// 设置齿槽补偿的一圈零点。
// 当前工程在检测到 Z 相后调用，使 LUT 和机械绝对位置对齐。
void Cogging_setZeroCount(uint16_t rawCount)
{
    cog_zero_count = Cogging_wrapCount((int32_t)rawCount);
}

// 清掉运行时输出，通常在未启用补偿或辨识模式下调用。
void Cogging_clearRuntimeOutput(void)
{
    cog_iqComp_A = 0.0f;
    cog_iqRaw_A  = 0.0f;
}

// 记录一个齿槽样本。
// iq_fbk 是当前实际 q 轴电流；减去 Bm/Cm 估算的摩擦电流后，剩下部分近似认为是齿槽扰动。
void Cogging_recordSample(uint16_t rawCount, float omega_m, float iq_fbk,
                          float Bm, float Cm)
{
    uint16_t k;
    float sign_w;
    float iq_fric;
    float iq_res;

    // 补偿启用时不记录，避免把自己的补偿又记录进表里。
    if(cog_enable != 0U)
    {
        return;
    }

    // 正转和反转分别记录。速度太低时符号和电流不稳定，直接跳过。
    if(cog_record_mode == COG_RECORD_POSITIVE)
    {
        if(omega_m < COG_RECORD_MIN_SPEED_RADPS)
        {
            return;
        }
        sign_w = 1.0f;
    }
    else if(cog_record_mode == COG_RECORD_NEGATIVE)
    {
        if(omega_m > -COG_RECORD_MIN_SPEED_RADPS)
        {
            return;
        }
        sign_w = -1.0f;
    }
    else
    {
        return;
    }

    k = Cogging_indexFromRaw(rawCount);
    iq_fric = (Bm * omega_m + Cm * sign_w) / MOTOR_KT_NM_PER_A;
    iq_res  = iq_fbk - iq_fric;
    iq_res  = Cogging_limit(iq_res,
                            -COG_COMP_CURRENT_LIMIT_A,
                             COG_COMP_CURRENT_LIMIT_A);

    if(cog_record_mode == COG_RECORD_POSITIVE)
    {
        // 每个位置桶保存“累计值 + 计数”，生成 LUT 时再求平均。
        if(cog_cnt_pos[k] < 0xFFFFU)
        {
            cog_sum_pos[k] += iq_res;
            cog_cnt_pos[k]++;
            cog_pos_samples++;
        }
    }
    else
    {
        // 反转单独保存，后续与正转数据平均。
        if(cog_cnt_neg[k] < 0xFFFFU)
        {
            cog_sum_neg[k] += iq_res;
            cog_cnt_neg[k]++;
            cog_neg_samples++;
        }
    }

    cog_last_index  = k;
    cog_iqRecord_A  = iq_res;
}

// 根据当前位置查表得到补偿电流。
// 返回值会在 main.c 中叠加到 foc.iqRef_A。
float Cogging_getCompCurrent(uint16_t rawCount)
{
    uint16_t k0;
    uint16_t k1;
    uint16_t posCount;
    float x;
    float alpha;
    float raw;
    float gain;

    // 只有 enable 和 ready 同时为 1，补偿才真正输出。
    if((cog_enable == 0U) || (cog_ready == 0U))
    {
        Cogging_clearRuntimeOutput();
        return 0.0f;
    }

    // posCount = Cogging_wrapCount((int32_t)rawCount - (int32_t)cog_zero_count);
    posCount = Cogging_posCountFromRaw(rawCount);//-------修改-------

    x = (float)posCount * ((float)COG_LUT_SIZE / (float)ENC_COUNTS_PER_REV);
    k0 = (uint16_t)x;
    if(k0 >= COG_LUT_SIZE)
    {
        k0 = COG_LUT_SIZE - 1U;
    }

    alpha = x - (float)k0;
    k1 = k0 + 1U;
    if(k1 >= COG_LUT_SIZE)
    {
        k1 = 0U;
    }

    // 线性插值，避免 LUT 格点之间产生阶梯状电流。
    raw = (1.0f - alpha) * cog_lut[k0] + alpha * cog_lut[k1];
    raw = Cogging_limit(raw,
                        -COG_COMP_CURRENT_LIMIT_A,
                         COG_COMP_CURRENT_LIMIT_A);

    // cog_gain 建议调试时从 0.2 开始慢慢加到 1.0。
    gain = Cogging_limit(cog_gain, -COG_GAIN_LIMIT, COG_GAIN_LIMIT);
    cog_iqRaw_A   = raw;
    cog_iqComp_A  = gain * raw;
    cog_last_index = k0;

    return cog_iqComp_A;
}

