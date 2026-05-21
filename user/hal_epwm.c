//###########################################################################
// EPWM.C — ePWM 驱动实现（由 hal.c 拆分）
// 包含三路互补中心对齐 PWM 初始化、死区设置、Trip Zone 保护等。
//###########################################################################
#include "hal_epwm.h"



// ePWM 配置: 中心对齐互补模式, 15kHz, 带死区
static void HAL_setupOnePWM(uint32_t base)
{
    // 禁用 TBCLKSYNC 同步 (初始化期间)
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // 时基: Up-Down 计数, 周期 = TBPRD
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(base, PWM_TBPRD);
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setTimeBaseCounter(base, 0U);

    // 计数器比较: CMPA 控制占空比, 初始 50%
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, PWM_TBPRD / 2U);
    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // 动作限定: EPWM_A (高侧) — TI 标准做法
    //   计数上行 CMPA 匹配 → 拉高
    //   计数下行 CMPA 匹配 → 拉低
    //   谷底(counter=0) = V0(低侧全导通), 峰值(counter=TBPRD) = V7
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    // EPWM_B (低侧): 由死区模块自动从 A 取反生成, 不需要单独 AQ 配置
    // 死区模块: A→上升沿延迟, B→下降沿延迟 (互补+死区)
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);
    EPWM_setRisingEdgeDelayCount(base, PWM_DEADBAND_CNT);
    EPWM_setFallingEdgeDelayCount(base, PWM_DEADBAND_CNT);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_A, false);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_B, false);

    // Trip Zone: 上电后默认 force low (安全状态), 由软件释放
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
    EPWM_forceTripZoneEvent(base, EPWM_TZ_FORCE_EVENT_OST);
}

void EPWM_init(void)
{
    // 禁用 TBCLKSYNC
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // 配置三路 PWM
    HAL_setupOnePWM(MTR_PWM_A_BASE);
    HAL_setupOnePWM(MTR_PWM_B_BASE);
    HAL_setupOnePWM(MTR_PWM_C_BASE);

    // ePWM2/3 与 ePWM1 同步 (确保三角波相位一致)
    EPWM_enableSyncOutPulseSource(MTR_PWM_A_BASE, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);
    EPWM_enablePhaseShiftLoad(MTR_PWM_B_BASE);
    EPWM_setPhaseShift(MTR_PWM_B_BASE, 0U);
    EPWM_enablePhaseShiftLoad(MTR_PWM_C_BASE);
    EPWM_setPhaseShift(MTR_PWM_C_BASE, 0U);

    // ePWM1 SOCA: 在 TBCTR=0 (谷底) 触发 ADC 采样
    EPWM_enableADCTrigger(MTR_PWM_A_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(MTR_PWM_A_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    EPWM_setADCTriggerEventPrescale(MTR_PWM_A_BASE, EPWM_SOC_A, 1U);

    // 使能 TBCLKSYNC, 三路 PWM 同步启动
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

void HAL_writePWM(float dutyA,float dutyB,float dutyC)
{
    uint16_t cmpA = (uint16_t)((1.0f - dutyA) * (float)PWM_TBPRD);
    uint16_t cmpB = (uint16_t)((1.0f - dutyB) * (float)PWM_TBPRD);
    uint16_t cmpC = (uint16_t)((1.0f - dutyC) * (float)PWM_TBPRD);
    EPWM_setCounterCompareValue(MTR_PWM_A_BASE, EPWM_COUNTER_COMPARE_A, cmpA);
    EPWM_setCounterCompareValue(MTR_PWM_B_BASE, EPWM_COUNTER_COMPARE_A, cmpB);
    EPWM_setCounterCompareValue(MTR_PWM_C_BASE, EPWM_COUNTER_COMPARE_A, cmpC);
}

//===========================================================================
// PWM 输出使能 / 禁用 (通过 Trip Zone 控制)
//===========================================================================
void HAL_enablePWMoutput(void)
{
    EPWM_clearTripZoneFlag(MTR_PWM_A_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(MTR_PWM_B_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(MTR_PWM_C_BASE, EPWM_TZ_FLAG_OST);
}

void HAL_disablePWMoutput(void)
{
    EPWM_forceTripZoneEvent(MTR_PWM_A_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(MTR_PWM_B_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(MTR_PWM_C_BASE, EPWM_TZ_FORCE_EVENT_OST);
}
