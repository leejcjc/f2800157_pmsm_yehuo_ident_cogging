
//===========================================================================
// hal_eqep.c — eQEP1 增量编码器初始化实现
//===========================================================================
#include "hal_eqep.h"
#include "user_config.h"

//===========================================================================
// HAL_EQEP_init
// eQEP1 初始化:
//   - 使能 EQEP1 外设时钟
//   - 配置 GPIO28/29/9 复用为 EQEP1_A/B/INDEX
//   - 解码器: 正交模式 + 4× 倍频
//             (注: EQEP_CONFIG_2X_RESOLUTION 实际是 4× 倍频, TI 命名反直觉)
//   - 位置计数器: 0 ~ (ENC_COUNTS_PER_REV-1), 满量程自动回 0, 不依赖 Z 相
//   - 单位定时器: 1 ms 周期 (备用, 用于硬件锁存测速)
//   - 调试器暂停时计数器自由运行
//===========================================================================
void HAL_EQEP_init(void)
{
#if ENCODER_USE_ABZ_INCREMENTAL

    // 1. 使能 EQEP1 外设时钟
    SysCtl_enablePeripheral(ENC_EQEP_PERIPH);

    // 2. 配置 GPIO 复用为 EQEP1 信号
    GPIO_setPinConfig(ENC_EQEP_A_CFG);
    GPIO_setPadConfig(ENC_EQEP_A_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ENC_EQEP_A_GPIO, GPIO_QUAL_6SAMPLE);

    GPIO_setPinConfig(ENC_EQEP_B_CFG);
    GPIO_setPadConfig(ENC_EQEP_B_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ENC_EQEP_B_GPIO, GPIO_QUAL_6SAMPLE);

    GPIO_setPinConfig(ENC_EQEP_INDEX_CFG);
    GPIO_setPadConfig(ENC_EQEP_INDEX_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ENC_EQEP_INDEX_GPIO, GPIO_QUAL_6SAMPLE);

    // 3. 解码器配置:
    //    QUADRATURE        - 正交模式 (A/B 信号)
    //    2X_RESOLUTION     - 4× 倍频 (A/B 上下沿都计数)
    //    NO_SWAP           - A/B 不交换
    //    IGATE_DISABLE     - 不门控 Index 脉冲
    EQEP_setDecoderConfig(ENC_EQEP_BASE,
                          EQEP_CONFIG_QUADRATURE    |
                          EQEP_CONFIG_2X_RESOLUTION |
                          EQEP_CONFIG_NO_SWAP       |
                          EQEP_CONFIG_IGATE_DISABLE);

    // 4. 输入极性: 全部不反转 (后续若发现方向反, 可改这里)
    EQEP_setInputPolarity(ENC_EQEP_BASE, false, false, false, false);

    // 5. 位置计数器: 计满 maxPosition 后回 0
    //    maxPosition = ENC_COUNTS_PER_REV - 1 = 9999
    //    范围 0~9999

    EQEP_setPositionCounterConfig(ENC_EQEP_BASE,
                                  EQEP_POSITION_RESET_MAX_POS,
                                  (uint32_t)(ENC_COUNTS_PER_REV - 1U));
                                  
    // EQEP_setPositionCounterConfig(ENC_EQEP_BASE,
    //                           EQEP_POSITION_RESET_MAX_POS,
    //                           0xFFFFFFFFU);   // 临时: 不循环, 累加到溢出

    // 6. 不在硬件事件下复位计数器
    EQEP_setPositionInitMode(ENC_EQEP_BASE, EQEP_INIT_DO_NOTHING);

    // 7. 软件设位置初值为 0 (与 Encoder_init 中 prevAngle=0 同步)
    EQEP_setPosition(ENC_EQEP_BASE, 0U);

    // 8. 单位定时器: 1 ms 周期 (供后续硬件锁存测速使用, 当前 ISR 差分测速够用)
    EQEP_enableUnitTimer(ENC_EQEP_BASE, (uint32_t)(SYS_CLK_FREQ_Hz / 1000U));

    // 9. 锁存模式: 单位定时器溢出时锁存位置
    // EQEP_setLatchMode(ENC_EQEP_BASE, EQEP_LATCH_UNIT_TIME_OUT);

    // 9. 锁存模式: Z 上升沿锁存
    EQEP_setLatchMode(ENC_EQEP_BASE, EQEP_LATCH_RISING_INDEX);

    // 9.1 清 Z 中断标志 (避免上电误触发)
    EQEP_clearInterruptStatus(ENC_EQEP_BASE,
                              EQEP_INT_INDEX_EVNT_LATCH | EQEP_INT_GLOBAL);
    
    // 10. 调试器暂停时计数器自由运行 (方便单步调试)
    EQEP_setEmulationMode(ENC_EQEP_BASE, EQEP_EMULATIONMODE_RUNFREE);

    // 11. 使能 eQEP1 模块
    EQEP_enableModule(ENC_EQEP_BASE);

#endif  // ENCODER_USE_ABZ_INCREMENTAL
}
