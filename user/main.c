//###########################################################################
// main.c — 主程序入口 + 电机控制 ISR + 状态机
//
// 控制架构: 位置环(P) → 速度环(PI) → 电流环(PI, Id+Iq) → SVPWM
// ISR 频率: 15 kHz (电流环每次执行, 速度/位置环 10:1 分频 = 1.5kHz)
//
// 电机: 24V PMSM (5极对, 增量式编码器)
// 驱动: 野火直流无刷电机驱动板 (三电阻采样, 信号隔离, 12~48V)
// 主控: TMS320F2800157 @ 120 MHz
//###########################################################################
#include "hal.h"
#include "foc.h"
#include "vofa.h"
#include "ident.h"
#include "cogging.h"
#include <math.h>

//===========================================================================
// 全局变量
//===========================================================================
volatile FOC_t          foc;
volatile MotorState_e   motorState  = MTR_STATE_IDLE;
volatile CtrlMode_e     ctrlMode    = CTRL_MODE_SPEED;
static CtrlMode_e       prevCtrlMode = CTRL_MODE_SPEED;
volatile FaultFlag_t    faultFlags;

//开环
volatile  float openLoopElecHz = 10.0f;
volatile  float openLoopVq_V   = 0.75f;

// === Z 相校准结果 ===
static volatile int32_t  g_zCntOffset = 0;       // Z 处锁存的 QPOSCNT (归一化到一个电周期)
static volatile bool     g_zCalDone   = false;   // 是否已找到 Z

// === Z 相校准内部变量 ===
static volatile uint32_t g_zCalCounter = 0U;     // ISR 节拍计数 (替代 sysTickMs)
static volatile float    g_zCalThetaE  = 0.0f;   // V/f 开环电角度

// 对齐相关
static volatile float   Id_align  = 0.0f;
static volatile uint32_t alignCounter = 0U;

// 速度差分计算用 (在速度环分频中使用)
static volatile float   prevPosMech_rad = 0.0f;

// ADC 偏置校准 (最终偏置值)
static volatile float   offsetIa = 0.0f;
static volatile float   offsetIb = 0.0f;
static volatile float   offsetIc = 0.0f;
// ADC 偏置校准 (独立累加器, 仅在 OFFSET_CAL 状态使用)
static volatile float   offsetSumIa = 0.0f;
static volatile float   offsetSumIb = 0.0f;
static volatile float   offsetSumIc = 0.0f;
static volatile uint32_t offsetCounter = 0U;
#define OFFSET_CAL_COUNT    2000U   // 校准采样次数 (~133ms @15kHz)

// VOFA 发送分频计数
static volatile uint16_t vofaCounter = 0U;

// 后台 1ms 计数
static volatile uint16_t ledCounter = 0U;

// 用户指令 (通过调试器 Expressions 窗口修改)------------
volatile bool   flagEnableMotor = false;    // 设为 true 启动电机
volatile bool   flagClearFault  = false;    // 设为 true 清除故障
volatile float  Iq_ref_A     = 0.0f;     // 转矩模式: Iq 指令 (A)
volatile float  speed_ref_rpm    = 0.0f;     // 速度模式: 速度指令 (rpm)
volatile float  pos_ref_rad = 0.0f;     // 位置模式: 位置指令 (rad)
volatile uint16_t cmdCtrlMode   = 0U;       // 0=转矩, 1=速度, 2=位置

volatile uint32_t zLatch_prev = 0;
volatile int32_t  zDelta      = 0;
volatile int32_t  zMin        = +99999;
volatile int32_t  zMax        = -99999;
volatile uint16_t zEventCount = 0;
volatile bool     zFirst      = true;

volatile float ia_raw;
volatile float ib_raw;
volatile float ic_raw;

volatile float ia_net;
volatile float ib_net;
volatile float ic_net;

volatile float ia_sensed;
volatile float ib_sensed;
volatile float ic_sensed;

volatile float thetaOpen_rad  = 0.0f;
extern  float ident_iq_pred;


//===========================================================================
// 前向声明
//===========================================================================
static void runStateMachine(void);
static void runFaultCheck(void);

//===========================================================================
// 电机控制 ISR (ADCA INT1, 15kHz)
//===========================================================================
#pragma CODE_SECTION(motorControlISR, ".TI.ramfunc")
__interrupt void motorControlISR(void)
{
    // ---- 1. 读取 ADC 采样 (三电阻全采样)8311特殊处理 电流校正系数（硬件自带） ----
    ia_raw = HAL_readCurrentA();
    ib_raw = HAL_readCurrentB();
    ic_raw = HAL_readCurrentC();

    // 扣除动态校准出来的零点偏置，得到净计数值 (在 0 附近波动) ----
    // 注意：在 MTR_STATE_OFFSET_CAL 跑完之前，offsetIa 等于 0，
    // 所以在校准阶段，不要把这里的 net 值送给 FOC，FOC 此时也不应工作。
    float ia_net = ia_raw - offsetIa;
    float ib_net = ib_raw - offsetIb;
    float ic_net = ic_raw - offsetIc;

    // ---- 3. 将净计数值转换为真实的物理电流 (安培) ----
    ia_sensed = ia_net * ADC_CURRENT_SF;
    ib_sensed = ib_net * ADC_CURRENT_SF;
    ic_sensed = ic_net * ADC_CURRENT_SF;

    //野火
    foc.iABC_A.a = -ia_sensed;
    foc.iABC_A.b = -ib_sensed;
    foc.iABC_A.c = -ic_sensed;


    // //DRV8311 专属处理：相电流串扰硬件解耦矩阵 (必须在安培域进行)
    // foc.iABC_A.a = 1.001152*ia_sensed - 0.003375*ib_sensed - 0.003103*ic_sensed ;
    // foc.iABC_A.b = 0.002369*ia_sensed + 1.000665*ib_sensed - 0.019126*ic_sensed ;
    // foc.iABC_A.c = 0.001234*ia_sensed + 0.001595*ib_sensed + 0.998166*ic_sensed ;

    // 硬件采样极性取反 (因为低边电阻采样，电流流出为正，运算放大器通常反相放大或根据你的硬件接法决定)
    // 野火的要取反
    // 8311不用取反，测试过
    // foc.iABC_A.a = -foc.iABC_A.a;
    // foc.iABC_A.b = -foc.iABC_A.b;
    // foc.iABC_A.c = -foc.iABC_A.c;


    // 母线电压
    foc.vDC_V = HAL_readVdc();
    if(foc.vDC_V < 6.0f)
    {
        foc.vDC_V = MOTOR_RATED_VOLTAGE_V;   // 防止除零 (调试器无母线时)
    }

    // ---- 2. 编码器角度更新 ----
#if ENCODER_USE_MT6701_SPI
    foc.enc.rawAngle = SPI_readMT6701();
#elif ENCODER_USE_ABZ_INCREMENTAL
    // ABZ/eQEP 解码完成后在这里更新 foc.enc.rawAngle
    foc.enc.rawAngle = HAL_EQEP_readAngle();
#endif
    Encoder_run((Encoder_t *)&foc.enc);
    foc.thetaFOC_rad = foc.enc.thetaElec_rad;

    
    // ---- 3. 状态机分支 ----
    switch(motorState)
    {
    //------------------------------------------------------------------
    case MTR_STATE_OFFSET_CAL:
    {
        // 累加 ADC 原始值到独立累加器
        offsetSumIa += ia_raw;
        offsetSumIb += ib_raw;
        offsetSumIc += ic_raw;
        offsetCounter++;

        if(offsetCounter >= OFFSET_CAL_COUNT)//零偏检测
        {
            offsetIa = offsetSumIa / (float)OFFSET_CAL_COUNT;
            offsetIb = offsetSumIb / (float)OFFSET_CAL_COUNT;
            offsetIc = offsetSumIc / (float)OFFSET_CAL_COUNT;
            offsetCounter = 0U;
            // 重置 PI (避免偏置校准期残留)
            PI_reset((PI_t *)&foc.piId);
            PI_reset((PI_t *)&foc.piIq);
            PI_reset((PI_t *)&foc.piSpd);

            // 准备进入 ALIGN
            alignCounter = 0U;
            // motorState = MTR_STATE_RUN;//开环测试
            motorState = MTR_STATE_ALIGN;
        }

        // 校准期间 PWM 50%
        HAL_writePWM(0.5f, 0.5f, 0.5f);
        break;
    }

     case MTR_STATE_ALIGN:
    {
        // 强制电角度 = 0, 注入 d 轴电压, 拉转子到 N 极对准 A 轴位置
        float sinTh, cosTh;
        FOC_sincos(0.0f, &sinTh, &cosTh);

        foc.vDQ.d = 0.8f;   // 1.5f
        foc.vDQ.q = 0.0f;
        // Clarke_run(foc.iABC_A.a, foc.iABC_A.b, (AB_t *)&foc.iAlBe);
        // Park_run((const AB_t *)&foc.iAlBe, sinTh, cosTh, (DQ_t *)&foc.iDQ);

        // // 3. ★ 闭环控制：让实际的 d 轴电流去追踪 Id_align_ref (1.5A)
        // foc.vDQ.d = PI_run((PI_t *)&foc.piId, Id_align_ref, foc.iDQ.d);
        // foc.vDQ.q = PI_run((PI_t *)&foc.piIq, 0.0f,         foc.iDQ.q);

        IPark_run((const DQ_t *)&foc.vDQ, sinTh, cosTh, (AB_t *)&foc.vAlBe);
        SVPWM_run((const AB_t *)&foc.vAlBe, foc.vDC_V, (ABC_t *)&foc.duty);
        HAL_writePWM(foc.duty.a, foc.duty.b, foc.duty.c);
        
        alignCounter++;

        // 持续 ALIGN_DURATION_MS 后, 转子已稳定 → 建立编码器零点基准
        if(alignCounter >= (uint32_t)(ALIGN_DURATION_MS * ISR_FREQ_Hz / 1000.0f))
        {
        alignCounter = 0U;
 
        // ★ 关键: 清零 QPOSCNT, 建立 "CNT=0 ↔ θ_e=0" 基准
        EQEP_setPosition(ENC_EQEP_BASE, 0U);
 
        // 同步软件编码器结构体
        foc.enc.offset      = 0;
        foc.enc.turnCount   = 0;
        foc.enc.prevAngle   = 0;
        foc.enc.rawAngle    = 0;
        foc.enc.posMech_rad = 0.0f;
        foc.enc.thetaMech_rad = 0.0f;
        foc.enc.thetaElec_rad = 0.0f;
        prevPosMech_rad     = 0.0f;
 
        // 清 Z 中断标志 (避免对齐期间产生的误触发)
        EQEP_clearInterruptStatus(ENC_EQEP_BASE,
                                  EQEP_INT_INDEX_EVNT_LATCH | EQEP_INT_GLOBAL);
 
        // 初始化 Z_CAL 状态变量
        g_zCalThetaE  = 0.0f;
        g_zCalCounter = 0U;
        g_zCalDone    = false;
 
        motorState = MTR_STATE_Z_CAL;
        
        }

        break;
    }

    case MTR_STATE_Z_CAL:
    {
        // 开环 V/f 旋转: θ_e 单调累加
        g_zCalThetaE += MATH_TWO_PI * ZCAL_FIND_FREQ_HZ * ISR_PERIOD_S;//开环 10Hz 8311数值
        if(g_zCalThetaE >= MATH_TWO_PI) g_zCalThetaE -= MATH_TWO_PI;//归一化
 
        // V/f 电压 (与 OPEN_LOOP_VF 公式一致)
        // float vq = OPEN_LOOP_VF_BOOST + OPEN_LOOP_VF_SLOPE * ZCAL_FIND_FREQ_HZ;
        float vq = 0.75;

        // 输出 SVPWM
        float sinTh, cosTh;
        FOC_sincos(g_zCalThetaE, &sinTh, &cosTh);
        foc.vDQ.d = 0.0f;
        foc.vDQ.q = vq;
        IPark_run((const DQ_t *)&foc.vDQ, sinTh, cosTh, (AB_t *)&foc.vAlBe);
        SVPWM_run((const AB_t *)&foc.vAlBe, foc.vDC_V, (ABC_t *)&foc.duty);
        HAL_writePWM(foc.duty.a, foc.duty.b, foc.duty.c);
 
        // 检测 Z 上升沿事件 (硬件已自动锁存 QPOSILAT)
        if(HAL_EQEP_indexEventOccurred())
        {
            int32_t zCntRaw = (int32_t)HAL_EQEP_getIndexLatch();//读取此时的z相位置
            int32_t zCogCnt = zCntRaw;
            int32_t zCnt = zCntRaw;

            zCogCnt %= (int32_t)ENC_COUNTS_PER_REV;
            if(zCogCnt < 0) zCogCnt += (int32_t)ENC_COUNTS_PER_REV;
            Cogging_setZeroCount((uint16_t)zCogCnt);
 
            // 归一化到一个电周期 [0, 2000)
            zCnt %= (int32_t)ENC_COUNTS_PER_ELEC_REV;

            if(zCnt < 0) zCnt += (int32_t)ENC_COUNTS_PER_ELEC_REV;
 
            g_zCntOffset = zCnt;//？为什么偏置值不是当前的读数zCnt减去刚开始的位置读数？
            g_zCalDone   = true;
 
            // ★ 把 Z 偏移写入 enc->offset, Encoder_run 自动用其修正电角度
            foc.enc.offset = g_zCntOffset;
 
            // 同步速度环 / 位置环初值, 防切换瞬间冲击
            Encoder_run((Encoder_t *)&foc.enc);          // 用最新 offset 重算一次
            prevPosMech_rad = foc.enc.posMech_rad;
            foc.posRef_rad  = foc.enc.posMech_rad;
 
            // 重置 PI (避免 V/f 阶段的积分残留)
            PI_reset((PI_t *)&foc.piId);
            PI_reset((PI_t *)&foc.piIq);
            PI_reset((PI_t *)&foc.piSpd);
 
            motorState = MTR_STATE_RUN;
        }
        else
        {
            // 超时保护: V/f 跑了 ZCAL_FIND_TIMEOUT_MS 仍没看到 Z → 报故障
            g_zCalCounter++;
            if(g_zCalCounter >= (uint32_t)(ZCAL_FIND_TIMEOUT_MS * ISR_FREQ_Hz / 1000.0f))
            {
                faultFlags.bit.encoderFault = 1;
                HAL_disablePWMoutput();
                motorState = MTR_STATE_FAULT;
            }
        }


        break;
    }

    //------------------------------------------------------------------
    case MTR_STATE_RUN:
    {
        // 直接用 foc.thetaFOC_rad (已在 ISR 开头由 Encoder_run 算出, 含 offset 修正)
        float sinTh, cosTh; 

        FOC_sincos(foc.thetaFOC_rad, &sinTh, &cosTh);

        Clarke_run(foc.iABC_A.a, foc.iABC_A.b, (AB_t *)&foc.iAlBe);
        Park_run((const AB_t *)&foc.iAlBe, sinTh, cosTh, (DQ_t *)&foc.iDQ);
 
         // ------------------------------ 速度环 + 位置环 (10:1 分频执行) ---------------------------------
         foc.spdLoopCounter++;
         if(foc.spdLoopCounter >= SPEED_LOOP_EXEC_RATIO)//10次电流环才进行一次速度环和位置环的计算
         {
             foc.spdLoopCounter = 0;

             // 编码器速度更新 (差分法: 位置变化量 / 时间间隔)
             float deltaPos = foc.enc.posMech_rad - prevPosMech_rad;
             prevPosMech_rad = foc.enc.posMech_rad;

             float rawSpeed = deltaPos / (SPEED_LOOP_EXEC_RATIO * ISR_PERIOD_S);
             // 速度 = 位置差 / 时间

            //一阶低通滤波（有什么影响？）
            foc.enc.speedMech_rads = 0.295f * rawSpeed + 0.705f * foc.enc.speedMech_rads;
           

            Encoder_calcSpeed((Encoder_t *)&foc.enc);//单位转换

            Cogging_recordSample(foc.enc.rawAngle,
                                 foc.enc.speedMech_rads,
                                 foc.iDQ.q,
                                 ident_res.Bm_Active,
                                 ident_res.Cm_Active);

             // 根据控制模式设定 Iq 参考
             ctrlMode = (CtrlMode_e)cmdCtrlMode;


             switch(ctrlMode)
             {
             case CTRL_MODE_TORQUE:
                 foc.iqRef_A = Iq_ref_A;
                 break;

             case CTRL_MODE_SPEED:
                 foc.iqRef_A = PI_run((PI_t *)&foc.piSpd,
                                      speed_ref_rpm * MATH_TWO_PI / 60.0f,
                                      foc.enc.speedMech_rads);
                 break;

             case CTRL_MODE_POSITION:
                 // 设定一个允许的误差范围，比如 0.05 rad (约 3度)
                if (fabsf(pos_ref_rad) < 0.05f)
                {
                  pos_ref_rad = 0.0f; // 误差足够小，直接归零
                }

                foc.posRef_rad = pos_ref_rad;
                foc.spdRef_rads = PosLoop_run(foc.posRef_rad,
                                               foc.enc.posMech_rad,
                                               POS_LOOP_SPD_LIMIT);
                 foc.iqRef_A = PI_run((PI_t *)&foc.piSpd,
                                      foc.spdRef_rads,
                                      foc.enc.speedMech_rads);
                 break;

             case CTRL_MODE_IDENT:
                if(ctrlMode == CTRL_MODE_IDENT && prevCtrlMode != CTRL_MODE_IDENT)
                {
                    Ident_reset();
                }       
                 foc.iqRef_A = Ident_speedLoop_run(foc.enc.speedMech_rads);
                 break;



             default:
                 foc.iqRef_A = 0.0f;
                 break;
             }

             if(ctrlMode != CTRL_MODE_IDENT)
             {
                 foc.iqRef_A += Cogging_getCompCurrent(foc.enc.rawAngle);

                 if(foc.iqRef_A > MOTOR_MAX_CURRENT_A)
                 {
                     foc.iqRef_A = MOTOR_MAX_CURRENT_A;
                 }
                 else if(foc.iqRef_A < -MOTOR_MAX_CURRENT_A)
                 {
                     foc.iqRef_A = -MOTOR_MAX_CURRENT_A;
                 }
             }
             else
             {
                 Cogging_clearRuntimeOutput();
             }

             prevCtrlMode = ctrlMode;//有什么用？
         }



         // ---- 电流环 (每次 ISR) ----
         // Id 参考 = 0 (表贴/混合步进, 无弱磁)
         foc.idRef_A = 0.0f;

         foc.vDQ.d = PI_run((PI_t *)&foc.piId, foc.idRef_A, foc.iDQ.d);
         foc.vDQ.q = PI_run((PI_t *)&foc.piIq, foc.iqRef_A, foc.iDQ.q);

         IPark_run((const DQ_t *)&foc.vDQ, sinTh, cosTh, (AB_t *)&foc.vAlBe);
         SVPWM_run((const AB_t *)&foc.vAlBe, foc.vDC_V, (ABC_t *)&foc.duty);
         HAL_writePWM(foc.duty.a, foc.duty.b, foc.duty.c);
     

         runFaultCheck();

        break;
    }

    case MTR_STATE_FAULT:
    case MTR_STATE_STOP:
    case MTR_STATE_IDLE:
    default:
        HAL_writePWM(0.5f, 0.5f, 0.5f);
        break;
    }

    // ---- VOFA 数据采集 (分频发送) ----
//    static float vofaTime_ms = 0.0f;
//    static float test_time_s = 0.0f;
    if(++vofaCounter >= 5)
    {
       vofaCounter = 0U;

    //    VOFA_updateData(speed_ref_rpm,
    //             foc.enc.speedMech_rads * 9.54929,
    //             foc.iqRef_A,
    //             foc.iDQ.q);


            //辨识数据
            VOFA_updateData(ident_res.speed_Ref * 60.0f / MATH_TWO_PI,                                 
                            foc.enc.speedMech_rads * 60.0f / MATH_TWO_PI,   // 转换为 rpm 发送
                            foc.iqRef_A,                                   
                            foc.iDQ.q); 
                       

    }

    // 中断应答
    HAL_ackADCInt();
}

//===========================================================================
// 故障检测 (在 ISR 中调用)
//===========================================================================
static void runFaultCheck(void)
{
    // 过压
    if(foc.vDC_V > OVER_VOLTAGE_V)
    {
        faultFlags.bit.overVoltage = 1;
    }
    // 欠压
    if(foc.vDC_V < UNDER_VOLTAGE_V)
    {
        faultFlags.bit.underVoltage = 1;
    }
    // 过流 (任一相)
    float iaAbs = (foc.iABC_A.a > 0) ? foc.iABC_A.a : -foc.iABC_A.a;
    float ibAbs = (foc.iABC_A.b > 0) ? foc.iABC_A.b : -foc.iABC_A.b;
    float icAbs = (foc.iABC_A.c > 0) ? foc.iABC_A.c : -foc.iABC_A.c;

    if(iaAbs > OVER_CURRENT_A || ibAbs > OVER_CURRENT_A || icAbs > OVER_CURRENT_A)
    {
        faultFlags.bit.overCurrent = 1;
    }

    // 有故障则立即停机
    if(faultFlags.all != 0)
    {
        motorState = MTR_STATE_FAULT;
        HAL_disablePWMoutput();
    }
}

//===========================================================================
// 后台状态机 (主循环 1ms 调用)
//===========================================================================
static void runStateMachine(void)
{
    switch(motorState)
    {
    case MTR_STATE_IDLE:
        if(flagEnableMotor)
        {
            /* 保留使能标志为真，用于保持 RUN 状态 */
            faultFlags.all = 0;

            // 重置 FOC (临界区: 防止 ISR 访问半初始化的结构体)
            DINT;
            FOC_init((FOC_t *)&foc);
            offsetIa = 0.0f;
            offsetIb = 0.0f;
            offsetIc = 0.0f;
            offsetSumIa = 0.0f;
            offsetSumIb = 0.0f;
            offsetSumIc = 0.0f;
            offsetCounter = 0U;
            EINT;

            // 使能驱动芯片和 PWM
            HAL_enableDRV();
            HAL_enablePWMoutput();

            motorState = MTR_STATE_OFFSET_CAL;
        }
        break;


    case MTR_STATE_FAULT:
        // 禁用 PWM 和驱动
        HAL_disablePWMoutput();
        HAL_disableDRV();

        if(flagClearFault)
        {
            flagClearFault = false;
            faultFlags.all = 0;
            motorState = MTR_STATE_IDLE;
        }
        break;

    case MTR_STATE_ALIGN:
    case MTR_STATE_Z_CAL:
    case MTR_STATE_RUN:
        if(flagEnableMotor == false)
        {
            // 用户请求停机
            motorState = MTR_STATE_STOP;
            HAL_disablePWMoutput();
            HAL_disableDRV();
            PI_reset((PI_t *)&foc.piId);
            PI_reset((PI_t *)&foc.piIq);
            PI_reset((PI_t *)&foc.piSpd);
            motorState = MTR_STATE_IDLE;
        }
        break;

    default:
        break;
    }
}

//===========================================================================
// main 函数
//===========================================================================
void main(void)
{
    // 1. 芯片初始化 (时钟、看门狗、外设时钟)

    Device_init();
    Device_initGPIO();

    // 2. PIE 初始化
    Interrupt_initModule();
    Interrupt_initVectorTable();

    // 3. 硬件初始化 (PWM, ADC, SPI, GPIO, 中断)
    HAL_init();

    // 4. FOC 初始化
    FOC_init((FOC_t *)&foc);

    Cogging_init();

    // 4.5 发送 VOFA 同步帧 (阻塞, 帮助 VOFA+ 锁定 4 通道)
    VOFA_sendSyncFrames();

    // 5. 使能全局中断
    EINT;
    ERTM;

    // 6. 主循环 (后台任务)
    for(;;)
    {
        // VOFA 非阻塞发送 (每次循环尝试填充 SCI FIFO)
        VOFA_sendBackground();
        Cogging_serviceRequests();

        if(HAL_getCPUTimerFlag())
        {
            HAL_clearCPUTimerFlag();

            // 状态机
            runStateMachine();

            // LED 闪烁 (运行=快闪, 故障=慢闪, 空闲=常亮)
            ledCounter++;
            switch(motorState)
            {
            case MTR_STATE_RUN:
                if(ledCounter >= 200U)   // 200ms
                {
                    GPIO_togglePin(31U);
                    ledCounter = 0;
                }
                break;
            case MTR_STATE_FAULT:
                if(ledCounter >= 1000U)  // 1s
                {
                    GPIO_togglePin(31U);
                    ledCounter = 0;
                }
                break;
            default:
                GPIO_writePin(31U, 1);
                ledCounter = 0;
                break;
            }
        }
    }
}


            //PPR测试  放runStateMachine()后面
            // if(HAL_EQEP_indexEventOccurred())
            // {
            // uint32_t zLatch_now = HAL_EQEP_getIndexLatch();

            // if(!zFirst)
            // {
            //     zDelta = (int32_t)(zLatch_now - zLatch_prev);   // 一圈应为 ±10000
            //     if(zDelta < zMin) zMin = zDelta;
            //     if(zDelta > zMax) zMax = zDelta;
            // }
            // zLatch_prev = zLatch_now;
            // zFirst = false;
            // zEventCount++;
            // }

        

        //开环测试  MTR_STATE_RUN
        
        /*
        thetaOpen_rad += MATH_TWO_PI * openLoopElecHz * ISR_PERIOD_S;

        if(thetaOpen_rad >= MATH_TWO_PI)
        {   
            thetaOpen_rad -= MATH_TWO_PI;
        }

        foc.vDQ.d = 0.0f;
        foc.vDQ.q = openLoopVq_V;

        // V/f 自动匹配 (低频电压补偿 + 与频率线性增长)
        // foc.vDQ.q = OPEN_LOOP_VF_BOOST + OPEN_LOOP_VF_SLOPE * openLoopElecHz; //野火测出来的  0.5+0.02*5

        FOC_sincos(thetaOpen_rad, &sinTh, &cosTh);

        // 2. 加入这两句：正向坐标变换观测实际电流（切记用开环的 sinTh, cosTh）
        Clarke_run(foc.iABC_A.a, foc.iABC_A.b, (AB_t *)&foc.iAlBe);
        Park_run((const AB_t *)&foc.iAlBe, sinTh, cosTh, (DQ_t *)&foc.iDQ);

        IPark_run((const DQ_t *)&foc.vDQ, sinTh, cosTh, (AB_t *)&foc.vAlBe);

        SVPWM_run((const AB_t *)&foc.vAlBe, foc.vDC_V, (ABC_t *)&foc.duty);

        HAL_writePWM(foc.duty.a, foc.duty.b, foc.duty.c);
        // 故障检测
        runFaultCheck();
        */



        // 可选: 每次过 Z 记录一次 QPOSILAT, 用于诊断 (不修改 offset 防抖动)
        // 暂时不用，后续优化再考虑,放在run最末端
        // if(HAL_EQEP_indexEventOccurred())
        // {
        //     int32_t zCnt = (int32_t)HAL_EQEP_getIndexLatch();
        //     zCnt %= (int32_t)ENC_COUNTS_PER_ELEC_REV;
        //     if(zCnt < 0) zCnt += (int32_t)ENC_COUNTS_PER_ELEC_REV;
        //     g_zCntOffset = zCnt;   // 实时更新, 用于在 watch 窗口观察是否漂移
        // }


               // 用于验证角度跟随
    //    VOFA_updateData(foc.vDQ.q,      // CH1: 我们注入的开环 Vq
    //                    foc.iDQ.q,      // CH2: 坐标变换反解析出来的 Iq
    //                    foc.enc.thetaElec_rad,      // 反馈角度
    //                    thetaOpen_rad); // CH4: 开环虚拟电角度

    //    test_time_s += 0.000333333f; // 加上 333.33us
       // 生成一个 50Hz 的标准正弦波：sin(2 * PI * f * t)
    //    float sine_50Hz = sinf(6.2831853f * 50.0f * test_time_s);

    //    vofaTime_ms += (float)VOFA_SEND_DIVIDER * ISR_PERIOD_S * 1000.0f;  // 单位 ms
       
    //电流环
    // VOFA_updateData(foc.iqRef_A,                                    // CH1: Iq指令 (A)
    //                 foc.iDQ.q,                                      // CH2: Iq实际 (A)
    //                 foc.iDQ.d,                                      // CH3: Id实际 (A), 应≈0
    //                 foc.enc.thetaElec_rad);                         // CH4: 电角度 (rad)

           
    //   //速度环
        // VOFA_updateData(speed_ref_rpm,                                  // CH1: 目标转速 (RPM)
        //               foc.enc.speedMech_rads * 60.0f / MATH_TWO_PI,   // CH2: 实际转速 (RPM)
        //               foc.iqRef_A,                                    // CH3: Iq电流指令 (A) - 速度环的输出
        //               foc.iDQ.q);                                     // CH4: Iq电流实际 (A) - 检验电流内环

    //  // 位置环
        // VOFA_updateData(pos_ref_rad,              // CH1: 位置指令 (rad)
        //         foc.enc.posMech_rad,              // CH2: 实际位置 (rad)
        //        foc.enc.speedMech_rads * 60.0f / MATH_TWO_PI, // CH3: 实际转速 (RPM)
        //         foc.iqRef_A);                        // CH4: Iq电流指令 (A)

    //  // 编码器数据
    // VOFA_updateData(foc.enc.thetaMech_rad,        // CH1: 机械角度 0~2π
    //             foc.enc.thetaElec_rad,        // CH2: 电角度 0~2π，会重复跳变
    //             foc.enc.rawAngle,             // CH3: 编码器原始值 0~16383
    //             foc.enc.posMech_rad);         // CH4: 连续机械位置
            //辨识数据
            // VOFA_updateData(ident_res.speed_Ref * 60.0f / MATH_TWO_PI,                                 
            //             foc.enc.speedMech_rads * 60.0f / MATH_TWO_PI,   // 转换为 rpm 发送
            //             foc.iqRef_A,                                   
            //             foc.iDQ.q);  

            //实验A辨识出BC，预测iq
            // VOFA_updateData(ident_iq_pred,
            // foc.enc.speedMech_rads * 60.0f / MATH_TWO_PI,
            //             foc.iqRef_A,                                   
            //             foc.iDQ.q);  
            
            //三相电流
                // float iSum = foc.iABC_A.a + foc.iABC_A.b + foc.iABC_A.c;
                // VOFA_updateData(foc.iABC_A.a,
                //                 foc.iABC_A.b,
                //                 foc.iABC_A.c,
                //                 iSum);

