//===========================================================================
// hal_eqep.h — eQEP1 增量编码器接口
// 编码器: 2500 PPR ABZ 增量编码器 (推挽 5V 输出), 4× 倍频后 10000 计数/转
//
// 板载电平转换: SN74LVC8T245PW (5V 编码器 → 3.3V MCU)
//   编码器(5V) → J12 → 电平转换 → MCU(3.3V) GPIO
//
// 引脚映射:
//   J12.5 (1A) → 电平转换 B1 → A1 → GPIO20 (EQEP1_A,    mux 1)
//   J12.4 (1B) → 电平转换 B2 → A2 → GPIO21 (EQEP1_B,    mux 1)
//   J12.3 (1I) → 电平转换 B3 → A3 → GPIO43 (EQEP1_INDEX, mux 10)
//   J12.2 (5V) → +5V_MCU (供编码器电源)
//   J12.1 (GND)→ GND
//===========================================================================
 
#ifndef HAL_EQEP_H
#define HAL_EQEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "device.h"

//===========================================================================
// eQEP1 资源映射
//===========================================================================
#define ENC_EQEP_BASE               EQEP1_BASE
#define ENC_EQEP_PERIPH             SYSCTL_PERIPH_CLK_EQEP1

#define ENC_EQEP_A_GPIO             20U
#define ENC_EQEP_A_CFG              GPIO_20_EQEP1_A
 
#define ENC_EQEP_B_GPIO             21U
#define ENC_EQEP_B_CFG              GPIO_21_EQEP1_B
 
#define ENC_EQEP_INDEX_GPIO         43U
#define ENC_EQEP_INDEX_CFG          GPIO_43_EQEP1_INDEX

//===========================================================================
// 函数声明
//===========================================================================

// eQEP1 初始化 (GPIO + 解码器 + 位置计数器 + 单位定时器)
void HAL_EQEP_init(void);

// 读取当前位置计数 (0 ~ ENC_COUNTS_PER_REV-1)
// 直接对应 QPOSCNT 寄存器, 由硬件维护
static inline uint16_t HAL_EQEP_readAngle(void)
{
    return (uint16_t)EQEP_getPosition(ENC_EQEP_BASE);
}

// 软件清零位置计数器 (用于零位标定)
static inline void HAL_EQEP_resetPosition(void)
{
    EQEP_setPosition(ENC_EQEP_BASE, 0U);
}

// 读取方向标志: 1 = 正转(CW), 0 = 反转(CCW)
static inline uint16_t HAL_EQEP_getDirection(void)
{
    return (EQEP_getStatus(ENC_EQEP_BASE) & EQEP_STS_DIR_FLAG) ? 1U : 0U;
}

//---------------------------------------------------------
// Z 相事件判定与读取 (用于 PPR 验证)
//---------------------------------------------------------

// 检测 Z 上升沿事件是否发生 (检测后自动清标志)
static inline bool HAL_EQEP_indexEventOccurred(void)
{
    if(EQEP_getInterruptStatus(ENC_EQEP_BASE) & EQEP_INT_INDEX_EVNT_LATCH)
    {
        EQEP_clearInterruptStatus(ENC_EQEP_BASE,
                                  EQEP_INT_INDEX_EVNT_LATCH | EQEP_INT_GLOBAL);
        return true;
    }
    return false;
}

// 读取 Z 锁存的位置值 (QPOSILAT 寄存器)
static inline uint32_t HAL_EQEP_getIndexLatch(void)
{
    return EQEP_getIndexPositionLatch(ENC_EQEP_BASE);
}
//---------------------------------------------------------




#ifdef __cplusplus
}
#endif

#endif // HAL_EQEP_H
