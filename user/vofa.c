// vofa.c — VOFA+ JustFloat 协议实现
//
// JustFloat 帧格式:
//   [float1_byte0..3][float2_byte0..3]...[0x00][0x00][0x80][0x7F]
//   每个 float 按小端序 (little-endian) 发送 4 字节
//   帧尾固定 4 字节: 0x00 0x00 0x80 0x7F

#include "vofa.h"
#include "hal_sci.h"
#include "user_config.h"

// 帧总字节数: 通道数 × 4 + 4(帧尾)
#define VOFA_FRAME_BYTES    (VOFA_CH_NUM * 4U + 4U)

// 双缓冲: ISR 写 dataNew[], 主循环读 dataNew[] 并打包发送
static volatile float dataNew[VOFA_CH_NUM];
static volatile bool  dataReady = false;

// 发送缓冲 (每个元素存 1 字节, 低 8 位有效)
static uint16_t txBuf[VOFA_FRAME_BYTES];
static uint16_t txIdx  = 0U;
static uint16_t txLen  = 0U;
static bool     txBusy = false;

//-----------------------------------------------------------------------
// 阻塞发送 1 字节 (等待 FIFO 有空位)
//-----------------------------------------------------------------------
static void sciWriteBlocking(uint16_t byte)
{
    while(SCI_getTxFIFOStatus(SCIA_BASE) >= SCI_FIFO_TX16) {}
    SCI_writeCharNonBlocking(SCIA_BASE, byte);
}

//-----------------------------------------------------------------------
// 启动时调用: 阻塞发送 10 帧同步帧 (固定值 0.1, 0.2, 0.3, 0.4)
// 帮助 VOFA+ 锁定 4 通道
//-----------------------------------------------------------------------
void VOFA_sendSyncFrames(void)
{
    // 使用无零字节的测试值, 避免和帧尾 00 00 80 7F 混淆
    // 0.1f=0x3DCCCCCD  0.2f=0x3E4CCCCD  0.3f=0x3E99999A  0.4f=0x3ECCCCCD
    static const float syncVal[4] = {0.1f, 0.2f, 0.3f, 0.4f};
    uint16_t frame, ch, j;

    for(frame = 0U; frame < 10U; frame++)
    {
        for(ch = 0U; ch < VOFA_CH_NUM; ch++)
        {
            float tmp = syncVal[ch];
            uint32_t raw = *(uint32_t *)&tmp;
            for(j = 0U; j < 4U; j++)
            {
                sciWriteBlocking((uint16_t)((raw >> (j * 8U)) & 0xFFU));
            }
        }
        // 帧尾
        sciWriteBlocking(0x00U);
        sciWriteBlocking(0x00U);
        sciWriteBlocking(0x80U);
        sciWriteBlocking(0x7FU);
    }
}

//-----------------------------------------------------------------------
// ISR 中调用: 仅拷贝 4 个 float, 极快
//-----------------------------------------------------------------------
void VOFA_updateData(float ch1, float ch2, float ch3, float ch4)
{
    if(txBusy) return;  // 上一帧还没发完, 跳过

    dataNew[0] = ch1;
    dataNew[1] = ch2;
    dataNew[2] = ch3;
    dataNew[3] = ch4;
    dataReady  = true;
}

//-----------------------------------------------------------------------
// 将 float 数组打包为字节流 (小端序)
//-----------------------------------------------------------------------
static void packFrame(void)
{
    uint16_t i, j;
    uint16_t idx = 0U;

    for(i = 0U; i < 4; i++)
    {
        // 将 float 重新解释为 uint32_t 以提取字节
        uint32_t raw;
        float tmp = dataNew[i];
        raw = *(uint32_t *)&tmp;

        // 小端序: 低字节先发
        for(j = 0U; j < 4U; j++)
        {
            txBuf[idx++] = (uint16_t)((raw >> (j * 8U)) & 0xFFU);
        }
    }

    // FireWater 帧尾
    txBuf[idx++] = 0x00U;
    txBuf[idx++] = 0x00U;
    txBuf[idx++] = 0x80U;
    txBuf[idx++] = 0x7FU;

    txLen  = idx;
    txIdx  = 0U;
    txBusy = true;
}

//-----------------------------------------------------------------------
// 主循环中调用: 打包 + 非阻塞发送
//-----------------------------------------------------------------------
void VOFA_sendBackground(void)
{
    // 1. 上一帧发完 + 新数据就绪 → 打包新帧
    if(!txBusy && dataReady)
    {
        packFrame();
        dataReady = false;
    }

    // 2. 逐字节写入 SCI TX FIFO (非阻塞)
    if(txBusy)
    {
        while(txIdx < txLen)
        {
            if(SCI_getTxFIFOStatus(SCIA_BASE) < SCI_FIFO_TX16)
            {
                SCI_writeCharNonBlocking(SCIA_BASE, txBuf[txIdx]);
                txIdx++;
            }
            else
            {
                return;  // FIFO 满, 下次再来
            }
        }
        txBusy = false;
    }
}
