#ifndef ADC_USER_H
#define ADC_USER_H

#include "device.h"
#include "user_config.h"

//===========================================================================
// ADC 配置: ADCA + ADCC, 12-bit, 4x 过采样
//===========================================================================
void ADC_init(void);

// ADC 读取 (含 4x 过采样平均)
float HAL_readCurrentA(void);
float HAL_readCurrentB(void);
float HAL_readCurrentC(void);
float HAL_readVdc(void);
// ADC 中断应答
void  HAL_ackADCInt(void);

extern volatile uint16_t raw_all;

//===========================================================================
// 三、ADC 配置 (三电阻采样, 4x 过采样)
//===========================================================================
// Ia → ADCA / AIN0  (48QFP Pin11, AIO231),  4x 过采样 SOC0~SOC3
// Ib → ADCC / CIN1  (48QFP Pin14, AIO238),  4x 过采样 SOC0~SOC3
// Ic → ADCA / AIN2  (48QFP Pin6,  GPIO224),  4x 过采样 SOC4~SOC7
// Vdc→ ADCA / AIN3  (48QFP Pin5,  GPIO242),  单次 SOC8
#define MTR_ADC_IA_BASE         ADCA_BASE
#define MTR_ADC_IA_CHANNEL      ADC_CH_ADCIN0
#define MTR_ADC_IA_SOC_START    ADC_SOC_NUMBER0     // SOC0~SOC3
#define MTR_ADC_IA_OVERSAMPLE   4U

#define MTR_ADC_IB_BASE         ADCC_BASE
#define MTR_ADC_IB_CHANNEL      ADC_CH_ADCIN1
#define MTR_ADC_IB_SOC_START    ADC_SOC_NUMBER0     // SOC0~SOC3
#define MTR_ADC_IB_OVERSAMPLE   4U

#define MTR_ADC_IC_BASE         ADCA_BASE
#define MTR_ADC_IC_CHANNEL      ADC_CH_ADCIN2       //
#define MTR_ADC_IC_SOC_START    ADC_SOC_NUMBER4     // SOC4~SOC7
#define MTR_ADC_IC_OVERSAMPLE   4U

//母线电压 的采集定义
#define MTR_ADC_VDC_BASE        ADCA_BASE
#define MTR_ADC_VDC_CHANNEL     ADC_CH_ADCIN3       // 改用 AIN3 (主板Pin24)，绕开有问题的 AIN6
#define MTR_ADC_VDC_SOC         ADC_SOC_NUMBER8     // Ia(SOC0-3) + Ic(SOC4-7) 之后

// ADC 触发: ePWM1 SOCA 在计数器=0时触发 (三角波谷底,低侧导通,最佳采样时刻)
#define MTR_ADC_TRIGGER         ADC_TRIGGER_EPWM1_SOCA

// ADC 采样窗口 (ACQPS), 至少 75ns → 120MHz*75ns = 9 → 设 14 (多留余量)
#define MTR_ADC_ACQPS           14U


#define ADC_REF_V                   3.3f
#define ADC_CURRENT_FULL_RANGE_A    15.5f      
#define ADC_CURRENT_OFFSET_COUNT    2048.0f    // 12-bit 中点偏置 (无电流时的 ADC 值)
#define ADC_CURRENT_V_PER_A       (ADC_REF_V / ADC_CURRENT_FULL_RANGE_A)
#define ADC_CURRENT_SF            ((ADC_REF_V / 4096.0f) / ADC_CURRENT_V_PER_A)//相当于 15.5 / 4096


// 母线电压: 野火驱动板电压采样范围 12~48V, 隔离运放输出
 #define ADC_FULL_SCALE_VOLTAGE_V    34.8f   // 24V对应2.3V实测推算 (24 * 3 / 2.27)
//#define ADC_FULL_SCALE_VOLTAGE_V    30.39f
#define ADC_VOLTAGE_SF              (ADC_FULL_SCALE_VOLTAGE_V / 4096.0f)




#endif // ADC_USER_H
