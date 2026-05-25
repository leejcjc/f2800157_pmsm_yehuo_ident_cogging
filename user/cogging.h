/*
 * cogging.h
 *
 *  Created on: 2026年5月21日
 *      Author: Lenovo
 */
// cogging.h - Position LUT cogging torque compensation

#ifndef COGGING_H
#define COGGING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "user_config.h"

#define COG_LUT_SIZE                 512U
#define COG_RECORD_MIN_SPEED_RADPS   0.05f
#define COG_COMP_CURRENT_LIMIT_A     1.0f
#define COG_GAIN_LIMIT               1.0f
#define COG_SMOOTH_RADIUS            2U

typedef enum {
    COG_RECORD_STOP     = 0U,  //停止记录
    COG_RECORD_POSITIVE = 1U,  //正转记录
    COG_RECORD_NEGATIVE = 2U   //反转记录
} CoggingRecordMode_e;

extern volatile uint16_t cog_enable;
extern volatile uint16_t cog_ready;
extern volatile uint16_t cog_record_mode;
extern volatile uint16_t cog_generate_req;
extern volatile uint16_t cog_reset_req;
extern volatile uint16_t cog_zero_count;
extern volatile uint16_t cog_last_index;
extern volatile uint16_t cog_missing_bins;

extern volatile uint32_t cog_pos_samples;
extern volatile uint32_t cog_neg_samples;

extern volatile float cog_gain;
extern volatile float cog_iqComp_A;
extern volatile float cog_iqRaw_A;
extern volatile float cog_iqRecord_A;

//测试
extern volatile uint16_t cog_raw_dbg;
extern volatile uint16_t cog_zero_dbg;
extern volatile int32_t  cog_diff_dbg;
extern volatile int32_t  cog_mod_dbg;
extern volatile uint16_t cog_posCount_dbg;
extern volatile uint32_t cog_idx32_dbg;
extern volatile uint16_t cog_idx_dbg;

void  Cogging_init(void);
void  Cogging_reset(void);
void  Cogging_serviceRequests(void);
void  Cogging_setZeroCount(uint16_t rawCount);
void  Cogging_clearRuntimeOutput(void);
void  Cogging_recordSample(uint16_t rawCount, float omega_m, float iq_fbk,
                           float Bm, float Cm);
float Cogging_getCompCurrent(uint16_t rawCount);

//新增
static uint16_t Cogging_posCountFromRaw(uint16_t rawCount);

#ifdef __cplusplus
}
#endif

#endif // COGGING_H
