//###########################################################################
// ident.c — 机械参数在线辨识 (吴春正交积分法为核心, 融合Kim初始惯量自整定)
// 硬件平台: TI F2800157 DSP, 速度环执行频率 1.5 kHz
// 辨识参数: 转动惯量Jm(kg·m²)、粘滞摩擦系数Bm(N·m·s/rad)、库仑摩擦系数Cm(N·m)
// 核心特性: 支持常值负载下辨识, 无需脱钩; 自动整定速度环PI增益; 迭代摩擦补偿
//###########################################################################
#include "ident.h"
#include "foc.h"
#include "user_config.h"
#include <math.h>

// === 数学函数替换 (原 CMSIS 版本 → 标准 math.h) 适配F2800157编译器 ===
#define sin_f32(x)          sinf(x) // 单精度正弦函数
#define cos_f32(x)          cosf(x) // 单精度余弦函数

// === 速度环周期 (与 SPEED_LOOP_EXEC_RATIO 同步  1.5kHz → 约666.67us) ===
#define TS                  (1.0f / SPEED_LOOP_FREQ_Hz)   // ≈ 0.000667

// === 复用工程全局电机参数 (避免重复定义, 保证一致性) ===
#define KT                  MOTOR_KT_NM_PER_A             // 转矩常数 Kt = 1.5Pnψf ≈ 0.04061 Nm/A
#define MAX_CURRENT_A       MOTOR_MAX_CURRENT_A           // 电机最大允许电流 5.0A, 用于PI限幅
#define PI_VAL              MATH_PI                       // π ≈ 3.1415926535
#define RPM_TO_RADPS(rpm)   ((rpm) * (MATH_TWO_PI / 60.0f))// 转速单位转换: 转/分 → 弧度/秒

// === 算法收敛与稳定性参数 ===
// 一阶低通滤波器系数: 截止频率0.5Hz @1.5kHz采样, α = 1 - exp(-2π·f·Ts) ≈ 0.00209
// 作用: 平滑Jm辨识结果, 避免速度环PI增益突变导致的振荡 (来自Kim2018文献)
#define LPF_ALPHA           0.00209f
#define MACRO_ITER_MAX      8U          // 最大迭代次数, 8轮迭代后收敛
#define A1_SETTLE_CYCLES    4U          // 首轮A1激励缓冲4个周期, 等待速度环稳定
#define JM_ABSOLUTE_MAX     0.001f      // 惯量上限, 防止计算错误导致飞车
#define ITER_DELAY_TIME     0.5f        // 迭代间歇0.5s, 让系统完全静止, 消除累积误差

// === 初始惯量粗测参数 (来自Kim2018文献) ===
#define TEST_iq             0.3f        // 粗测注入q轴电流 1A (约0.04Nm转矩, 安全且足够加速)
#define TEST_TIME           0.2f        // 粗测时长0.3s, 避免速度过高

// === 正弦激励频率 (两篇文献均推荐0.5Hz, 核心原因) ===
// 1. 电流环带宽(通常>1kHz)远高于0.5Hz, 可近似为理想跟随
// 2. 速度环带宽(10Hz)是激励频率的20倍, 速度跟踪误差极小
// 3. 低频率下, 逆变器死区、齿槽转矩等高频干扰的影响被积分平均掉
#define W_H                 MATH_PI     // 机械角频率 ωh = π rad/s → 频率f = ωh/(2π) = 0.5Hz

// === 速度环PI增益自整定参数 (用于把 Jm 折算成 PI 增益, 仅供观察, 不回写) ===
#define W_SC                62.83f      // 期望速度环闭环带宽 10Hz (ωsc = 2π·10 ≈ 62.83 rad/s)
#define H_RATIO             5.0f        // 积分系数比例: Ki = Kp·ωsc / 5, 保证相位裕度>60°

//===========================================================================
// 全局变量 (供 VOFA / 调试器访问 加volatile防止编译器优化)
//===========================================================================
volatile IdentState_e ident_state = STATE_ROUGH_J; // 辨识状态机当前状态
Ident_Output_t        ident_res   = {0};            // 辨识结果输出结构体

float rpm1     = 300.0f;        // 第一个正弦激励幅值A1 (转/分), 对应吴春文献A1
float rpm2     = 600.0f;        // 第二个正弦激励幅值A2 (转/分), 对应吴春文献A2
float test_rpm1 = 200.0f;       // 辨识完成后跟踪测试速度

//===========================================================================
// 内部状态 (仅在速度环ISR中使用, 加static限制作用域, 提高安全性)
//===========================================================================
static float    dtheta;// 每个速度环周期的角度增量 dθ = ωh·Ts, 正弦指令相位步进
static float    A1, A2;// 转换为rad/s的实际激励幅值 A1=RPM_TO_RADPS(rpm1)

static float    time_cnt        = 0.0f;// 通用计时器, 用于粗测、回零、迭代延迟等
static float    theta0          = 0.0f;// 正弦速度指令的当前相位 θ = ωh·t
static float    initial_speed   = 0.0f;// 粗测开始时的初始速度, 用于计算速度变化量

static float    Jm_roughly      = 0.0f;// 粗测得到的近似惯量 (来自Kim2018)
static float    Jm_rate         = 0.0f;// 初始惯量爬坡速率 (来自Kim2018)
static float    Jm_ini          = 0.00001f;// 初始惯量, 从极小值开始爬坡

static float    sum_Te_cos      = 0.0f;// 吴春核心积分1: ∫Te·cosθ dθ, 用于计算Jm
static float    sum_Te_sin1     = 0.0f;// 吴春核心积分2: A1幅值下∫Te·sinθ dθ, 用于计算Bm
static float    sum_Te_sin2     = 0.0f;// 吴春核心积分3: A2幅值下∫Te·sinθ dθ, 用于计算Cm

static float    Jm_target       = 0.0f;// 本轮计算得到的Jm原始值
static float    Jm_lpf_out      = 0.0f;// 低通滤波后的Jm, 用于速度环PI增益计算
static uint8_t  is_Jm_switched  = 0U;// 标志位: 0=使用爬坡Jm_ini, 1=使用辨识Jm_target
static uint16_t cycle_count     = 0U;// 正弦激励周期计数器

static uint8_t  macro_iter_cnt  = 0U;// 大迭代次数计数器
static float    Jm_last         = 0.0f;// 上一轮迭代的Jm结果
static float    Bm_last         = 0.0f;// 上一轮迭代的Bm结果
static float    Cm_last         = 0.0f;// 上一轮迭代的Cm结果

static float    Jm_calc         = 0.0f;// 单周期计算得到的Jm临时值
static float    delta_speed     = 0.0f;// 粗测阶段的速度变化量

// 速度环PI控制器状态 (积分器单位:A, 与电流环输入单位一致)
static float    speed_pi_integral = 0.0f;// PI积分器状态
static float    Iq_ref            = 0.0f;// 上一周期的q轴电流参考, 用于计算电磁转矩

// 动态PI增益 (运行时由当前Jm_Active实时计算, 保证带宽恒定)
static float    spdPID_Kp_iq;
static float    spdPID_Ki_iq;

volatile float ident_iq_pred = 0.0f;

//===========================================================================
// Ident_reset — 重置所有算法状态, 回到初始粗测状态
// 调用时机: 上电初始化、辨识失败重启、用户手动触发重新辨识
//===========================================================================
void Ident_reset(void)
{
    ident_state = STATE_ROUGH_J;// 状态机回到初始粗测惯量状态

    dtheta          = W_H * TS;// 计算每个周期的角度增量 dθ = ωh·Ts
    A1              = RPM_TO_RADPS(rpm1);// 转换A1为rad/s
    A2              = RPM_TO_RADPS(rpm2);// 转换A2为rad/s

    // 重置计时器和相位
    time_cnt        = 0.0f;
    theta0          = 0.0f;
    initial_speed   = 0.0f;

    // 重置初始惯量相关变量
    Jm_roughly      = 0.0f;
    Jm_rate         = 0.0f;
    Jm_ini          = 0.00001f;

    // 重置吴春核心积分器
    sum_Te_cos      = 0.0f;
    sum_Te_sin1     = 0.0f;
    sum_Te_sin2     = 0.0f;

    // 重置Jm滤波和切换标志
    Jm_target       = 0.0f;
    Jm_lpf_out      = 0.0f;
    is_Jm_switched  = 0U;
    cycle_count     = 0U;

    // 重置迭代计数器和上轮结果
    macro_iter_cnt  = 0U;
    Jm_last         = 0.0f;
    Bm_last         = 0.0f;
    Cm_last         = 0.0f;

    // 重置速度环PI状态
    speed_pi_integral = 0.0f;
    Iq_ref            = 0.0f;

    // 重置输出结构体
    ident_res.speed_Ref   = 0.0f;// 速度参考输出
    ident_res.Jm_Active   = 0.0f;// 当前生效的惯量
    ident_res.Bm_Active   = 0.0f;// 当前生效的粘滞摩擦系数
    ident_res.Cm_Active   = 0.0f;// 当前生效的库仑摩擦系数
    ident_res.Enable_FF   = 0U;// 摩擦前馈使能标志
    ident_res.Is_Finished = 0U;// 辨识完成标志
}

//===========================================================================
// Motor_Parameter_Ident_Process — 核心辨识状态机
// 调用频率: 每1.5kHz速度环周期调用一次
// 输入: speed_fbk = 机械角速度反馈 (rad/s), Te_last = 上一周期电磁转矩 (Nm)
//===========================================================================
static void Motor_Parameter_Ident_Process(float speed_fbk, float Te_last)
{
    // 支持调试器实时修改rpm1/rpm2, 修改后立即生效
    A1 = RPM_TO_RADPS(rpm1);
    A2 = RPM_TO_RADPS(rpm2);

    //-----------------------------------------------------------------------
    // Jm低通滤波与切换逻辑 (融合Kim2018的平滑增益更新)
    // 1. 未切换前: Jm_ini线性爬坡, 逐步提高速度环增益
    // 2. 切换后: 对辨识得到的Jm_target做一阶低通滤波, 避免增益突变
    //-----------------------------------------------------------------------
    if (ident_state >= STATE_RETURN_ZERO && ident_state <= STATE_EVALUATE)
    {
        if (!is_Jm_switched)//爬坡状态
        {
            Jm_lpf_out = Jm_ini;// 使用爬坡的初始惯量
        }
        else//辨识状态
        {
            Jm_lpf_out = LPF_ALPHA * Jm_target + (1.0f - LPF_ALPHA) * Jm_lpf_out;// 一阶低通滤波: y(n) = α·x(n) + (1-α)·y(n-1)
        }   //Jm_lpf_out是上次的值  Jm_target是本次的值
    }
    ident_res.Jm_Active = Jm_lpf_out;

    //--------------------------
    // 状态机主逻辑
    //--------------------------
    switch (ident_state)
    {
        //-------------------------------------------------------------------
        // 状态1: 初始惯量粗测 (来自Kim2018文献)
        // 原理: 施加恒定电流, 测量速度变化, 由T=J·α 得 J = T·Δt/Δω
        // 作用: 得到近似惯量, 为后续Jm_ini爬坡提供基准, 避免初始增益过低导致跟踪崩溃
        //-------------------------------------------------------------------
        case STATE_ROUGH_J:
            if (time_cnt == 0.0f)
            {
                initial_speed = speed_fbk;// 记录粗测开始时的初始速度
            }
            ident_res.speed_Ref = 0.0f;  // 速度环开环, 不使用PI控制
            time_cnt += TS;              // 计时

            if (time_cnt >= TEST_TIME) // 粗测时长到, 计算近似惯量
            {
                delta_speed = speed_fbk - initial_speed;
                Jm_roughly  = (delta_speed > 0.1f)// 防止速度变化过小导致除零错误
                            ? ((TEST_iq * KT * TEST_TIME) / delta_speed)
                            : JM_ABSOLUTE_MAX;

                // 限制粗测惯量上限, 防止飞车
                if (Jm_roughly > JM_ABSOLUTE_MAX) Jm_roughly = JM_ABSOLUTE_MAX;

                // 注入频率 0.5 Hz, 周期 2 s, 自增 5 s 即 2.5 个周期到达粗略值
                // 计算Jm_ini爬坡速率
                Jm_rate = Jm_roughly / (10.0f * (W_H / (2.0f * PI_VAL)));

                // 重置状态, 进入回零阶段
                theta0   = 0.0f;
                time_cnt = 0.0f;
                ident_state = STATE_RETURN_ZERO;
            }
            break;


        //--------------------------------------------------
        // 状态2: 电机回零 + Jm_ini爬坡 (来自Kim2018文献)
        // 作用: 1. 让电机完全静止, 为正弦激励做准备
        //       2. Jm_ini线性爬坡, 逐步提高速度环增益, 避免振荡
        //--------------------------------------------------
        case STATE_RETURN_ZERO:
            ident_res.speed_Ref = 0.0f;     // 速度给 0, 电磁刹车

            // Jm_ini持续爬坡, 直到达到粗测值（这里可能没爬完，到A1继续爬）
            if (!is_Jm_switched)
            {
                Jm_ini += Jm_rate * TS;
                if (Jm_ini > Jm_roughly) Jm_ini = Jm_roughly;
            }

            // 电机速度接近0时, 进入A1激励阶段
            if (fabsf(speed_fbk) < 1.0f)
            {
                theta0      = 0.0f;// 重置正弦相位
                sum_Te_cos  = 0.0f;// 重置积分器
                sum_Te_sin1 = 0.0f;
                cycle_count = 0U;// 重置周期计数器
                ident_state = STATE_INT_A1;
            }
            break;

        //-------------------------------------------------------------------
        // 状态3: 施加A1幅值正弦激励 + 正交积分 (吴春2021核心算法)
        // 对应吴春文献式(13)(14): 利用cosθ正交性分离惯量项
        // 原理:
        // 运动方程: Te = J·A1·ωh·cosθ + B·A1·sinθ + sgn(sinθ)·C + TL
        // 两边乘cosθ并在0~2π积分:
        // ∫Te·cosθ dθ = J·A1·ωh·π + 0 + 0 + 0 = J·A1·ωh·π
        // → J = ∫Te·cosθ dθ / (π·A1·ωh)
        //-------------------------------------------------------------------
        case STATE_INT_A1:
        {
            theta0 += dtheta;// 相位步进
            ident_res.speed_Ref = A1 * sin_f32(theta0);// 生成正弦速度参考: ω_ref = A1·sin(ωh·t)

            if (!is_Jm_switched)// 继续爬坡Jm_ini, 直到切换到辨识值
            {
                Jm_ini += Jm_rate * TS;
                if (Jm_ini > Jm_roughly) Jm_ini = Jm_roughly;
            }

            // 【文献核心算法】：利用正交特性将信号乘以 cos 和 sin 并在一个完整周期(0~2π)内积分(即累加)
            // sum_Te_cos: 积分Te·cosθ, 仅保留惯量项, 摩擦和负载项积分抵消为0
            // sum_Te_sin1: 积分Te·sinθ, 仅保留摩擦项, 惯量和负载项积分抵消为0
            sum_Te_cos  += Te_last * cos_f32(theta0) * dtheta;
            sum_Te_sin1 += Te_last * sin_f32(theta0) * dtheta;

            if (theta0 >= 2.0f * PI_VAL)// 完成一个完整2π周期的积分
            {
                // 计算惯量 Jm: 累加值 = Jm * ω_h * A1 * π
                Jm_calc = sum_Te_cos / (PI_VAL * A1 * W_H);

                // 如果 is_Jm_switched 还是 0：说明这是第一回算出真正的 Jm。
                if (!is_Jm_switched && (Jm_calc > 0.000001f))
                {
                    is_Jm_switched = 1U;//得到真正的Jm后切换标志位置1, 之后不再使用爬坡的Jm_ini, 直接使用辨识的Jm_target
                    Jm_target      = Jm_calc;
                }
                 // 如果已经置 1 了，后续迭代更新Jm_target
                else if (is_Jm_switched && (Jm_calc > 0.000001f))
                {
                    Jm_target      = Jm_calc;//辨识模式下进行迭代
                }

                cycle_count++;// 周期计数+1

                // 首轮A1缓冲4个周期(等待速度环稳定), 后续迭代每轮1个周期
                //如果这是整个辨识流程刚启动的第一轮，(macro_iter_cnt == 0)，系统还没稳定，
                //摩擦补偿也没加上，所以系统特别宽容，要求缓冲跑 4圈（抛弃前3圈，只要最后1圈数据）；

                //如果是后面的迭代（macro_iter_cnt == 1），跑 1 圈就够了。
                //因为系统已经很稳了并且加上了前馈补偿，所以就不需要再浪费那么多圈去缓冲了，直接跑 1圈 出一次成绩即可。
                uint16_t required_cycles = (macro_iter_cnt == 0U) ? A1_SETTLE_CYCLES : 1U;
                if (cycle_count >= required_cycles)
                {
                    // 达到要求周期数, 进入A2激励阶段
                    theta0      = 0.0f;
                    sum_Te_cos  = 0.0f;
                    sum_Te_sin2 = 0.0f;
                    ident_state = STATE_INT_A2;
                }
                else
                {
                    // 未达到要求周期数, 说明还在“练手”，清积分 续跑A1
                    theta0      = 0.0f;
                    sum_Te_cos  = 0.0f;
                    sum_Te_sin1 = 0.0f;
                }
            }
            break;
        }

        //-------------------------------------------------------------------
        // 状态4: 施加A2幅值正弦激励 + 正交积分 (吴春2021核心算法)
        // 作用: 得到第二个关于B和C的方程, 联立求解二元一次方程组
        // 对应吴春文献式(21)(23):
        // sum_Te_sin1 = B·A1·π + C·4
        // sum_Te_sin2 = B·A2·π + C·4
        //-------------------------------------------------------------------
        case STATE_INT_A2:
            theta0 += dtheta;
            // 生成A2幅值的正弦速度参考
            ident_res.speed_Ref = A2 * sin_f32(theta0);

            // 继续正交积分
            sum_Te_cos  += Te_last * cos_f32(theta0) * dtheta;
            sum_Te_sin2 += Te_last * sin_f32(theta0) * dtheta;

            // 完成一个2π周期积分
            if (theta0 >= 2.0f * PI_VAL)
            {
                // 用A2幅值的数据再次验证Jm, 提高可靠性
                Jm_calc = sum_Te_cos / (PI_VAL * A2 * W_H);
                if (is_Jm_switched && (Jm_calc > 0.000001f))
                {
                    Jm_target = Jm_calc;
                }
                ident_state = STATE_EVALUATE;
            }
            break;

        //-------------------------------------------------------------------
        // 状态5: 计算Bm和Cm + 迭代判断 (吴春2021核心算法)
        // 对应吴春文献式(24)(25): 解二元一次方程组得到B和C
        //-------------------------------------------------------------------
        case STATE_EVALUATE:
        {
            ident_res.speed_Ref = 0.0f;
            
            // 【文献核心算法】：分离粘性摩擦系数(Bm)和库仑摩擦力矩(Cm)
            // 在 A1 和 A2 两种幅值下，sin 的积分值分别满足以下二元一次方程组：
            // sum_Te_sin1 = Bm * A1 * π + Cm * 4
            // sum_Te_sin2 = Bm * A2 * π + Cm * 4
            // 解方程组可得 Bm 和 Cm：
            float Bm_calc = (sum_Te_sin1 - sum_Te_sin2) / (PI_VAL * (A1 - A2));
            float Cm_calc = (A1 * sum_Te_sin2 - A2 * sum_Te_sin1) / (4.0f * (A1 - A2));

            // 更新当前生效的摩擦系数
            ident_res.Bm_Active = Bm_calc;
            ident_res.Cm_Active = Cm_calc;
            ident_res.Enable_FF = 1U; // 使能摩擦前馈, 下一轮迭代开始补偿

            macro_iter_cnt++;// 大迭代次数+1

            // 达到最大迭代次数, 辨识完成
            if (macro_iter_cnt >= MACRO_ITER_MAX)
            {
                ident_res.Is_Finished = 1U;
                ident_state           = STATE_SUCCESS;
            }
            else// 未达到最大迭代次数, 进入间歇后继续下一轮
            {
                // 保存上一轮结果
                Jm_last = Jm_target;
                Bm_last = Bm_calc;
                Cm_last = Cm_calc;

                // 重置状态, 准备下一轮迭代
                theta0      = 0.0f;
                sum_Te_cos  = 0.0f;
                sum_Te_sin1 = 0.0f;
                cycle_count = 0U;
                time_cnt    = 0.0f;
                ident_state = STATE_ITER_DELAY;
            }
            break;
        }

        //-------------------------------------------------------------------
        // 状态6: 迭代间歇
        // 作用: 让系统完全静止, 消除上一轮激励的残余振动, 提高下一轮精度
        //-------------------------------------------------------------------
        case STATE_ITER_DELAY:
            ident_res.speed_Ref = 0.0f;
            time_cnt += TS;
            if (time_cnt >= ITER_DELAY_TIME)
            {
                time_cnt    = 0.0f;
                ident_state = STATE_INT_A1;
            }
            break;

        // 状态7: 辨识成功完成
        case STATE_SUCCESS:
            ident_res.speed_Ref = 0.0f;
            time_cnt = 0.0f;        // 一旦切到 TEST_TRACKING, 时间从 0 起
            break;

        // 状态8: 速度跟踪测试 (可选)
        // 作用: 验证辨识结果的有效性, 测试速度跟踪性能
        case STATE_TEST_TRACKING:
            theta0 += dtheta;// 相位步进
            ident_res.speed_Ref = RPM_TO_RADPS(test_rpm1) * sin_f32(theta0);
            time_cnt += TS;
            // ident_res.speed_Ref = RPM_TO_RADPS(test_rpm1);
            break;

        // 异常状态: 重置算法
        case STATE_RESTART:
        default:
            Ident_reset();
            break;
    }
}

//===========================================================================
// Ident_speedLoop_run — 速度环主入口, 1.5kHz调用
// 输入: speed_fbk = 机械角速度反馈 (rad/s)
// 返回: Iq_ref = q轴电流参考 (A), 输入到FOC电流环
//===========================================================================
// float Ident_speedLoop_run(float speed_fbk)
float Ident_speedLoop_run(float speed_fbk)
{
    // 用上一周期 Iq 还原电磁力矩 (假设电流环理想跟随)
    float Te_fbk = Iq_ref * KT;
    // float Te_fbk = iq_fbk * KT;


    // 调用核心辨识状态机
    Motor_Parameter_Ident_Process(speed_fbk,Te_fbk);

    //----------------------------
    // 不同状态下的Iq_ref生成逻辑
    //----------------------------
    if (ident_state == STATE_ROUGH_J)
    {
        // 粗测阶段: 开环注入恒定电流
        Iq_ref = TEST_iq;
    }
    else if (ident_state == STATE_SUCCESS || ident_state == STATE_RESTART)
    {
        // 辨识完成或重启: 输出0电流, 清空积分器
        Iq_ref            = 0.0f;
        speed_pi_integral = 0.0f;
    }
    else
    {
        //-------------------------------------------------------------------
        // 【Kim2018核心: 动态PI增益自整定】
        // 对应Kim文献式(24)(25): 根据当前Jm实时计算PI增益, 保证带宽恒定
        // 原理:
        // 速度环开环传递函数: Gsc(s) = (Kp + Ki/s) * (1/(J·s))
        // 期望带宽ωsc下, 令|Gsc(jωsc)| = 1, 得 Kp = J·ωsc
        // 积分系数 Ki = Kp·ωsc / 5, 保证相位裕度>60°
        // 转换为电流环增益: Kp_iq = Kp / Kt = J·ωsc / Kt
        // 这样可以确保无论辨识出的惯量怎么变，系统频带始终维持在 10Hz (W_sc)
        //-------------------------------------------------------------------

        spdPID_Kp_iq = (ident_res.Jm_Active * W_SC) / KT;
        spdPID_Ki_iq = (spdPID_Kp_iq * W_SC) / H_RATIO;

        // 计算速度误差
        float err = ident_res.speed_Ref - speed_fbk;

        // PI控制器计算
        speed_pi_integral += spdPID_Ki_iq * err * TS;
        // 积分限幅, 防止积分饱和
        if (speed_pi_integral >  MAX_CURRENT_A) speed_pi_integral =  MAX_CURRENT_A;
        if (speed_pi_integral < -MAX_CURRENT_A) speed_pi_integral = -MAX_CURRENT_A;

        float Iq_pi = spdPID_Kp_iq * err + speed_pi_integral;

        //-------------------------------------------------------------------
        // 【吴春2021核心: 摩擦前馈补偿(库仑 + 粘性)】
        // 对应吴春文献式(26): 提前计算摩擦转矩, 前馈到电流环
        // 原理: 摩擦转矩 Tf = B·ω + C·sgn(ω)
        // 转换为前馈电流: Iq_ff = Tf / Kt
        // 作用: 克服零速附近库仑摩擦的强非线性, 消除"爬行"现象
        //-------------------------------------------------------------------
        float Iq_ff = 0.0f;
        if (ident_res.Enable_FF)
        {
            // 计算速度符号, 零速时输出0避免抖动
            float sign_w = (ident_res.speed_Ref > 0.0f) ?  1.0f
                         : (ident_res.speed_Ref < 0.0f) ? -1.0f : 0.0f;
            // 计算摩擦转矩
            float Te_ff  = ident_res.Bm_Active * ident_res.speed_Ref
                         + ident_res.Cm_Active * sign_w;
            // 转换为前馈电流
            Iq_ff = Te_ff / KT;
        }

        // PI输出 + 前馈补偿 = 最终Iq参考
        Iq_ref = Iq_pi + Iq_ff;
        // 总电流限幅, 保护电机和逆变器
        if (Iq_ref >  MAX_CURRENT_A) Iq_ref =  MAX_CURRENT_A;
        if (Iq_ref < -MAX_CURRENT_A) Iq_ref = -MAX_CURRENT_A;

        float sign_w = (ident_res.speed_Ref > 0.0f) ?  1.0f :
               (ident_res.speed_Ref < 0.0f) ? -1.0f : 0.0f;

        ident_iq_pred = (ident_res.Bm_Active * ident_res.speed_Ref
              + ident_res.Cm_Active * sign_w) / KT;
    }

    return Iq_ref;
}
