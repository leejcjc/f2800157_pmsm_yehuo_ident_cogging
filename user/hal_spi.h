//===========================================================================
// SPIA 配置: 读取 MT6701 磁编码器 (14-bit 绝对角度)
// MT6701 SPI 模式: CPOL=1, CPHA=1 (SPI Mode 3), 16-bit 帧, MSB 先发
//===========================================================================

#ifndef SPI_USER_H
#define SPI_USER_H

#include "device.h"

// 编码器状态 (MT6701 14-bit 绝对角度, SPI 读取)
typedef struct {
    uint16_t rawAngle;          // MT6701 原始角度 (0~16383)
    uint16_t prevAngle;         // 上次角度值 (用于差分速度计算)
    int32_t  offset;            // 对齐偏移量 (角度计数值)
    float    thetaElec_rad;     // 电角度 (rad)
    float    thetaMech_rad;     // 机械角度 (rad)
    float    speedElec_Hz;      // 电频率 (Hz)
    float    speedMech_rpm;     // 机械转速 (rpm)
    float    speedMech_rads;    // 机械角速度 (rad/s)
    float    posMech_rad;       // 机械绝对位置 (rad, 连续累计)
    int32_t  turnCount;         // 圈数累计
} Encoder_t;

//===========================================================================
// MT6701 磁编码器配置 (SPI 接口, 14-bit 绝对角度)
//===========================================================================

// #define ENC_RESOLUTION_BITS         14U
// #define ENC_COUNTS_PER_REV          16384U              // 2^14 = 16384

// SPIA 引脚配置
#define ENC_SPI_BASE                SPIA_BASE
#define ENC_SPI_SIMO_GPIO           8U                  // GPIO8 = SPIA_SIMO (MT6701 不用, 保留)
#define ENC_SPI_SIMO_CFG            GPIO_8_SPIA_SIMO
#define ENC_SPI_SOMI_GPIO           10U                 // GPIO10 = SPIA_SOMI (MT6701 A/MISO)
#define ENC_SPI_SOMI_CFG            GPIO_10_SPIA_SOMI
#define ENC_SPI_CLK_GPIO            9U                  // GPIO9 = SPIA_CLK (MT6701 B/CLK)
#define ENC_SPI_CLK_CFG             GPIO_9_SPIA_CLK
#define ENC_SPI_CS_GPIO             11U                 // GPIO11 = CS (GPIO 手动控制, MT6701 Z/CSn)

// SPI 时钟: MT6701 最大支持约 10MHz, 设为 120MHz / 24 = 5 MHz
#define ENC_SPI_BITRATE             5000000UL
// SPI 波特率寄存器值: BRR = (LSPCLK / bitrate) - 1
// LSPCLK = SYSCLK / 4 = 30 MHz (默认), BRR = 30M/5M - 1 = 5
#define ENC_SPI_BRR                 5U

// MT6701 上电就绪等待时间 (≥32ms)
#define ENC_MT6701_STARTUP_US       35000UL

void SPI_init(void);
// MT6701 SPI 角度读取 (14-bit)。
// 加入超时计数，防止 SPI 硬件异常导致 ISR 死等。
uint16_t SPI_readMT6701(void);

// 编码器处理
void    Encoder_init(Encoder_t *enc);
void    Encoder_run(Encoder_t *enc);
void    Encoder_calcSpeed(Encoder_t *enc);

#endif // SPI_USER_H
