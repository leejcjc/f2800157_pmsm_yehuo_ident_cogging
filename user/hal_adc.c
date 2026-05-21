#include "hal_adc.h"

volatile uint16_t raw_all = 0U;

volatile float ia_raw_rg;
volatile float ib_raw_rg;
volatile float ic_raw_rg;

// ADC 读取 (含 4x 过采样平均)
float HAL_readCurrentA(void)
{
    uint32_t sum = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0)
                 + ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1)
                 + ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2)
                 + ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3);
    ia_raw_rg = (float)sum * 0.25f;
//    return (ia_raw_rg ) * ADC_CURRENT_SF;
    // return (ia_raw_rg - ADC_CURRENT_OFFSET_COUNT) * ADC_CURRENT_SF;
     return ia_raw_rg ;

}

float HAL_readCurrentB(void)
{
    uint32_t sum = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0)
                 + ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1)
                 + ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2)
                 + ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER3);
    ib_raw_rg = (float)sum * 0.25f;
//    return (ib_raw_rg ) * ADC_CURRENT_SF;
    // return (ib_raw_rg - ADC_CURRENT_OFFSET_COUNT) * ADC_CURRENT_SF;
     return ib_raw_rg;
    
}

float HAL_readCurrentC(void)
{
    uint32_t sum = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4)
                 + ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5)
                 + ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER6)
                 + ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER7);
    ic_raw_rg = (float)sum * 0.25f;
//    return (ic_raw_rg ) * ADC_CURRENT_SF;
    // return (ic_raw_rg - ADC_CURRENT_OFFSET_COUNT) * ADC_CURRENT_SF;
     return  ic_raw_rg;
}

float HAL_readVdc(void)
{
    uint16_t raw = ADC_readResult(ADCARESULT_BASE, MTR_ADC_VDC_SOC);
    raw_all = raw;
    return (float)raw * ADC_VOLTAGE_SF;
}

// ADC 中断应答
void HAL_ackADCInt(void)
{
    ADC_clearInterruptStatus(MTR_ADC_IA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}


void ADC_init(void)
{
    //ADCA
    //IA   ADCINA0 — 专用 AIO 引脚，无需 GPIO 配置

    //IC   ADCINA2 (GPIO224, AGPIO 引脚，需要切换为模拟模式)
    GPIO_setPinConfig(GPIO_224_GPIO224);
    GPIO_setAnalogMode(224U, GPIO_ANALOG_ENABLED);

    //Vdc  ADCINA3 (GPIO242, AGPIO 引脚，需要切换为模拟模式)
    GPIO_setPinConfig(GPIO_242_GPIO242);
    GPIO_setAnalogMode(242U, GPIO_ANALOG_ENABLED);


    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);  // ADC 上电延迟
    ADC_disableBurstMode(ADCA_BASE);
    ADC_setSOCPriority(ADCA_BASE, ADC_PRI_ALL_ROUND_ROBIN);//设置转换优先级

    ADC_setInterruptPulseMode(MTR_ADC_IA_BASE, ADC_PULSE_END_OF_CONV);//中断（或结果读取）一定要在 ADC 把模拟量彻底转换为数字量之后进行

    // ADCA SOC0~SOC3: Ia 4x 过采样
    uint16_t soc;
    for(soc = 0; soc < MTR_ADC_IA_OVERSAMPLE; soc++)
    {
        ADC_setupSOC(MTR_ADC_IA_BASE, (ADC_SOCNumber)(ADC_SOC_NUMBER0 + soc),
                     MTR_ADC_TRIGGER, MTR_ADC_IA_CHANNEL, 14U);
    }

    // ADCA SOC4~SOC7: Ic 4x 过采样
    for(soc = 0; soc < MTR_ADC_IC_OVERSAMPLE; soc++)
    {
        ADC_setupSOC(MTR_ADC_IC_BASE, (ADC_SOCNumber)(MTR_ADC_IC_SOC_START + soc),
                     MTR_ADC_TRIGGER, MTR_ADC_IC_CHANNEL, 14U);
    }

    // ADCA SOC8: Vdc 单次采样
    ADC_setupSOC(MTR_ADC_VDC_BASE, MTR_ADC_VDC_SOC,
                 MTR_ADC_TRIGGER, MTR_ADC_VDC_CHANNEL, 14U);

    // ADCA INT1: SOC8 完成后触发 (Ia+Ic过采样+Vdc全部完成)
    ADC_setInterruptSource(MTR_ADC_IA_BASE, ADC_INT_NUMBER1, MTR_ADC_VDC_SOC);
    ADC_enableInterrupt(MTR_ADC_IA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(MTR_ADC_IA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(MTR_ADC_IA_BASE, ADC_INT_NUMBER1);



    // --- ADCC 配置 ---
    //IB   ADCC_IN1 — 专用 AIO 引脚，无需 GPIO 配置

    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_4_0);
    ADC_enableConverter(ADCC_BASE);
    DEVICE_DELAY_US(1000);
    ADC_disableBurstMode(ADCC_BASE);
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setSOCPriority(ADCC_BASE, ADC_PRI_ALL_ROUND_ROBIN);//设置转换优先级 我不知道这行代码是否必须用


    // ADCC SOC0~SOC3: Ib 4x 过采样
    for(soc = 0; soc < MTR_ADC_IB_OVERSAMPLE; soc++)
    {
        ADC_setupSOC(ADCC_BASE, (ADC_SOCNumber)(ADC_SOC_NUMBER0 + soc),
                     MTR_ADC_TRIGGER, MTR_ADC_IB_CHANNEL, MTR_ADC_ACQPS);
    }
}
