// hal_sci.h — SCI (UART) 驱动, 用于 VOFA+ 上位机通信
#ifndef HAL_SCI_H
#define HAL_SCI_H

#include "driverlib.h"
#include "device.h"

//===========================================================================
// SCI (UART → VOFA+ 上位机)
//===========================================================================

#define SCI_TX_GPIO                 29U
#define SCI_TX_CFG                  GPIO_29_SCIA_TX
#define SCI_RX_GPIO                 28U
#define SCI_RX_CFG                  GPIO_28_SCIA_RX
#define SCI_BAUDRATE                921600UL
// #define SCI_BAUDRATE                115200UL
#define VOFA_SEND_DIVIDER           5U      // 每 5 次 ISR 发送一次 (3 kHz, 60KB/s < 92KB/s)

void HAL_SCI_init(void);

#endif // HAL_SCI_H
