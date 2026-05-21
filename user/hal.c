// hal.c — 硬件初始化实现

#include "hal.h"


// GPIO 配置
void HAL_setupGPIO(void)
{
    // --- ePWM 引脚 (A相: GPIO0/1, B相: GPIO2/3, C相: GPIO4/5) ---
    GPIO_setPinConfig(MTR_PWM_AH_CFG);
    GPIO_setPadConfig(MTR_PWM_AH_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(MTR_PWM_AL_CFG);
    GPIO_setPadConfig(MTR_PWM_AL_GPIO, GPIO_PIN_TYPE_STD);

    GPIO_setPinConfig(MTR_PWM_BH_CFG);
    GPIO_setPadConfig(MTR_PWM_BH_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(MTR_PWM_BL_CFG);
    GPIO_setPadConfig(MTR_PWM_BL_GPIO, GPIO_PIN_TYPE_STD);

    GPIO_setPinConfig(MTR_PWM_CH_CFG);
    GPIO_setPadConfig(MTR_PWM_CH_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(MTR_PWM_CL_CFG);
    GPIO_setPadConfig(MTR_PWM_CL_GPIO, GPIO_PIN_TYPE_STD);

#if ENCODER_USE_MT6701_SPI
    // --- SPIA 引脚 (MT6701 磁编码器) ---
    // SIMO (GPIO8): MT6701 不用 MOSI, 保留配置
    GPIO_setPinConfig(ENC_SPI_SIMO_CFG);
    GPIO_setPadConfig(ENC_SPI_SIMO_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ENC_SPI_SIMO_GPIO, GPIO_QUAL_ASYNC);

    // SOMI (GPIO10): 接 MT6701 A/MISO
    GPIO_setPinConfig(ENC_SPI_SOMI_CFG);
    GPIO_setAnalogMode(ENC_SPI_SOMI_GPIO, GPIO_ANALOG_DISABLED);
    GPIO_setPadConfig(ENC_SPI_SOMI_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(ENC_SPI_SOMI_GPIO, GPIO_QUAL_ASYNC);

    // CLK (GPIO9): 接 MT6701 B/CLK
    GPIO_setPinConfig(ENC_SPI_CLK_CFG);
    GPIO_setPadConfig(ENC_SPI_CLK_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ENC_SPI_CLK_GPIO, GPIO_QUAL_ASYNC);

    // CS (GPIO11): GPIO 手动控制, 接 MT6701 Z/CSn
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setPadConfig(ENC_SPI_CS_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ENC_SPI_CS_GPIO, GPIO_QUAL_6SAMPLE);
    GPIO_setDirectionMode(ENC_SPI_CS_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_writePin(ENC_SPI_CS_GPIO, 1);   // CS 默认高 (未选中)
#endif

    // --- 野火驱动板 Enable (GPIO7, 输出高电平使能) ---
    GPIO_setPinConfig(DRV_EN_CFG);
    GPIO_setDirectionMode(DRV_EN_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DRV_EN_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_writePin(DRV_EN_GPIO, 0);   // 初始禁用, 待初始化完成后使能

    // --- 板载 LED (GPIO31) 用于状态指示 ---
    GPIO_setPinConfig(GPIO_49_GPIO49);
    GPIO_setDirectionMode(49U, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(49U, GPIO_PIN_TYPE_STD);
    GPIO_writePin(49U, 1);
}



// 中断配置: ADCA INT1 → ISR

// ISR 函数声明 (在 main.c 中实现)
extern __interrupt void motorControlISR(void);

void HAL_setupInterrupt(void)
{
    // 注册 ADCA INT1 到 PIE
    Interrupt_register(INT_ADCA1, &motorControlISR);

    // 使能 PIE 中的 ADCA INT1 (Group 1, Channel 1)
    Interrupt_enable(INT_ADCA1);

    // 使能 CPU 中断 (Group 1)
    Interrupt_enableInCPU(INTERRUPT_CPU_INT1);
}


// CPU Timer0: 1ms 后台任务定时
void HAL_setupCPUTimer0(void)
{
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);
    CPUTimer_setPeriod(CPUTIMER0_BASE, SYS_CLK_FREQ_Hz / 1000U - 1U);  // 1ms
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                               CPUTIMER_EMULATIONMODE_RUNFREE);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER0_BASE);
}


// 野火驱动板使能/禁用
void HAL_enableDRV(void)
{
    GPIO_writePin(DRV_EN_GPIO, 1);
    DEVICE_DELAY_US(1000);  // 等待驱动板启动
}

void HAL_disableDRV(void)
{
    GPIO_writePin(DRV_EN_GPIO, 0);
}


// 一键初始化
void HAL_init(void)
{
    HAL_setupGPIO();
    EPWM_init();
    ADC_init();
#if ENCODER_USE_MT6701_SPI
    SPI_init();
#endif
#if ENCODER_USE_ABZ_INCREMENTAL
    HAL_EQEP_init();
#endif
    HAL_SCI_init();
    HAL_setupCPUTimer0();
    HAL_setupInterrupt();

#if ENCODER_USE_MT6701_SPI
    // 等待 MT6701 上电就绪 (≥ 32ms)
    DEVICE_DELAY_US(ENC_MT6701_STARTUP_US);
#endif
}
