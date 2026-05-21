// vofa.h — VOFA+ FireWater 协议, 通过 SCI 发送浮点数据到上位机
#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>
#include <stdbool.h>

// 通道数 (可修改, 最大建议 8)
#define VOFA_CH_NUM     4U

// ISR 中调用: 更新待发送数据 (仅拷贝 float, 不占 ISR 时间)
void VOFA_updateData(float ch1, float ch2, float ch3, float ch4);

// 启动时调用: 阻塞发送几帧同步帧, 帮助 VOFA+ 锁定通道数
void VOFA_sendSyncFrames(void);

// 主循环中调用: 将数据打包为 FireWater 帧并通过 SCI 发送
void VOFA_sendBackground(void);

#endif // VOFA_H
