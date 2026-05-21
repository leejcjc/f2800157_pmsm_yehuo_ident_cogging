// hal_sci.c — SCI (UART) 驱动实现
#include "hal_sci.h"
#include "user_config.h"

void HAL_SCI_init(void)
{
    // --- GPIO 配置 ---
    // TX (GPIO29 → SCIA_TX)
    GPIO_setPinConfig(SCI_TX_CFG);
    GPIO_setDirectionMode(SCI_TX_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(SCI_TX_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(SCI_TX_GPIO, GPIO_QUAL_ASYNC);

    // RX (GPIO28 → SCIA_RX)
    GPIO_setPinConfig(SCI_RX_CFG);
    GPIO_setDirectionMode(SCI_RX_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(SCI_RX_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(SCI_RX_GPIO, GPIO_QUAL_ASYNC);

    // --- SCI 模块配置 ---
    SCI_performSoftwareReset(SCIA_BASE);

    // LSPCLK = SYSCLK / 4 = 30 MHz
    SCI_setConfig(SCIA_BASE,
                  SYS_CLK_FREQ_Hz / 4U,
                  SCI_BAUDRATE,
                  (SCI_CONFIG_WLEN_8 |
                   SCI_CONFIG_STOP_ONE |
                   SCI_CONFIG_PAR_NONE));

    // 使能 TX FIFO
    SCI_enableFIFO(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
//    SCI_resetChannels(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
}
