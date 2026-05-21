#include "hal_spi.h"
#include "user_config.h"
#include <math.h>

//===========================================================================
// SPIA 配置: 读取 MT6701 磁编码器 (14-bit 绝对角度)
// MT6701 SPI 模式: CPOL=1, CPHA=1 (SPI Mode 3), 16-bit 帧, MSB 先发
//===========================================================================
void SPI_init(void)
{
#if ENCODER_USE_MT6701_SPI
    // 复位 SPI 模块
    SPI_disableModule(ENC_SPI_BASE);

    // SPI 主模式
    SPI_setConfig(ENC_SPI_BASE, SYS_CLK_FREQ_Hz / 4U,  // LSPCLK = 30 MHz
                  SPI_PROT_POL0PHA0,                     // 模式0
                  SPI_MODE_MASTER,
                  ENC_SPI_BITRATE,
                  16U);                                  // 16-bit 字长

    // 使能 FIFO
    SPI_enableFIFO(ENC_SPI_BASE);
    SPI_setFIFOInterruptLevel(ENC_SPI_BASE, SPI_FIFO_TX0, SPI_FIFO_RX1);
    SPI_resetRxFIFO(ENC_SPI_BASE);
    SPI_resetTxFIFO(ENC_SPI_BASE);

    // 使能 SPI 模块
    SPI_enableModule(ENC_SPI_BASE);
#endif
}


// MT6701 SPI 角度读取 (14-bit)。
// 加入超时计数，防止 SPI 硬件异常导致 ISR 死等。
uint16_t SPI_readMT6701(void)
{
#if ENCODER_USE_MT6701_SPI
    static uint16_t lastGoodAngle = 0U;   // 记住上一帧有效数据

    // 1. 选通 CS
    GPIO_writePin(ENC_SPI_CS_GPIO, 0);

    DEVICE_DELAY_US(1);//片选后延时

    // 2. 发送虚拟 16-bit 数据以产生时钟
    SPI_writeDataNonBlocking(ENC_SPI_BASE, 0x0000U);

    // 3. 等待 RX FIFO 有数据；加简单超时
    uint16_t timeout = 0U;
    while(SPI_getRxFIFOStatus(ENC_SPI_BASE) == SPI_FIFO_RX0)
    {
        if(++timeout > 3000U)      // ~3 µs @ 1 tick = 1 SPI clk (5 MHz) → 裕度足够
        {
            GPIO_writePin(ENC_SPI_CS_GPIO, 1);   // 释放 CS
            return lastGoodAngle;                // 超时，返回上次有效角度
        }
    }

    // 4. 读取数据
    uint16_t rawData = SPI_readDataNonBlocking(ENC_SPI_BASE);

    // 5. 释放 CS
    GPIO_writePin(ENC_SPI_CS_GPIO, 1);
 
    uint16_t angle = (rawData >> 2) & 0x3FFFu;  //取高 14 位（D13…D0）
    // uint16_t angle = (rawData >> 2) & 0x1FFFu;  // 只取低13位，去掉恒为1的bit13

    // 7. 正常更新角度并返回
    lastGoodAngle = angle;
    return angle;
#else
    return 0U;
#endif
}

//===========================================================================
// 编码器初始化 (MT6701 14-bit 绝对角度)
//===========================================================================
void Encoder_init(Encoder_t *enc)
{
    enc->rawAngle       = 0;
    enc->prevAngle      = 0;
    enc->offset         = 0;
    enc->thetaElec_rad  = 0.0f;
    enc->thetaMech_rad  = 0.0f;
    enc->speedElec_Hz   = 0.0f;
    enc->speedMech_rpm  = 0.0f;
    enc->speedMech_rads = 0.0f;
    enc->posMech_rad    = 0.0f;
    enc->turnCount      = 0;
}

//===========================================================================
// 编码器角度更新 (每次 ISR 调用)
//===========================================================================
void Encoder_run(Encoder_t *enc)
{
    int32_t angle = (int32_t)enc->rawAngle;
#if ENC_DIRECTION_REVERSED
    angle = (int32_t)(ENC_COUNTS_PER_REV - enc->rawAngle) % (int32_t)ENC_COUNTS_PER_REV;
#endif

    // 机械角度 (0 ~ 2π) 机械角度的作用： 在你的代码里，机械角度的作用是计算转速，不管 $x$ 有没有偏置，偏置在相减时都会被消掉
    enc->thetaMech_rad = (float)angle / (float)ENC_COUNTS_PER_REV * MATH_TWO_PI;

    // 1.电角度: θe = p * θm - offset_elec
    float thetaElec = (float)MOTOR_NUM_POLE_PAIRS * enc->thetaMech_rad;

    // 2. 将 offset (计数) 转成弧度   防止 C 语言中恶心的“负数取余（Modulo）” Bug
    float offsetElec = ((float)(enc->offset) / (float)ENC_COUNTS_PER_REV)
                       * (float)MOTOR_NUM_POLE_PAIRS * MATH_TWO_PI;

    thetaElec -= offsetElec;
                            
                    
    // 3. Wrap 到 0~2π
    thetaElec = fmodf(thetaElec, MATH_TWO_PI);
    if(thetaElec < 0.0f)
        thetaElec += MATH_TWO_PI;

    enc->thetaElec_rad = thetaElec;

    // 连续机械位置 (含圈数) 用于位置环
    // 检测过零跳变来更新圈数
    int32_t delta = angle - (int32_t)enc->prevAngle;
    if(delta > (int32_t)(ENC_COUNTS_PER_REV / 2))
    {
        enc->turnCount--;
    }
    else if(delta < -(int32_t)(ENC_COUNTS_PER_REV / 2))
    {
        enc->turnCount++;
    }
    enc->prevAngle = (uint16_t)angle;  // 存反转后的值，与 delta 计算一致

    // 绝对机械位置
    enc->posMech_rad = ((float)enc->turnCount * MATH_TWO_PI)
                     + enc->thetaMech_rad;
}

//===========================================================================
// 编码器速度计算 (在速度环分频中调用)
// 差分法: 用 posMech_rad 的变化量 / 时间间隔 计算速度
//===========================================================================
void Encoder_calcSpeed(Encoder_t *enc)
{
    // speedMech_rads 已由 ISR 差分计算写入
    // 这里做单位转换
    enc->speedMech_rpm = enc->speedMech_rads * 60.0f / MATH_TWO_PI;
    enc->speedElec_Hz  = enc->speedMech_rads * (float)MOTOR_NUM_POLE_PAIRS / MATH_TWO_PI;
}
