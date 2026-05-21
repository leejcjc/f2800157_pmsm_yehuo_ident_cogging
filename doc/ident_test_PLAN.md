# ident_experiment_plan.md

## 1. 实验目标

本实验用于验证工程中的 `CTRL_MODE_IDENT` 机械参数辨识功能是否能在实物上完整跑通，并判断辨识出的 `Jm`、`Bm`、`Cm` 是否具备工程可信度。

本实验分为 5 个部分独立验证：

1. 硬件与 FOC 基础闭环验证
2. 电流环与速度反馈验证
3. `CTRL_MODE_IDENT` 状态机低风险验证
4. 完整机械参数辨识验证
5. 辨识结果有效性验证

必须按顺序执行。前一部分不通过，不要进入下一部分。

---

## 2. 实验前必须确认的事项

### 2.1 必须确认 `Ident_reset()` 已执行

当前 `ident.c` 中：

```c
dtheta = W_H * TS;
```

只在 `Ident_reset()` 中初始化。如果没有执行 `Ident_reset()`，`dtheta` 可能为 0，导致 `STATE_INT_A1` 中：

```c
theta0 += dtheta;
```

不递增，正弦速度指令无法生成，状态机会卡在 `STATE_INT_A1 = 2`。

实验前必须满足以下条件之一：

- 进入 `CTRL_MODE_IDENT` 前，代码中已经调用过一次 `Ident_reset()`
- 或者在烧录前确认 `dtheta` 有静态初始化
- 或者在 CCS 调试中确认能可靠调用 `Ident_reset()`

若不能确认，不要开始辨识实验。

### 2.2 第一次实验必须降低辨识幅值

默认值：

```c
rpm1 = 300
rpm2 = 600
```

第一次实物实验建议先改为：

```text
rpm1 = 100
rpm2 = 200
```

原因：

- 100/200 rpm 更安全
- 可以先验证状态机是否能走完
- 可以避免速度环、电流环、编码器方向问题造成大幅往复运动

等低风险实验通过后，再恢复到：

```text
rpm1 = 300
rpm2 = 600
```

### 2.3 第一次实验建议空载或轻载

第一次验证建议电机空载或仅带轻负载。

不要第一次就连接高惯量机构、丝杆、机械臂或刚性负载。

原因：

- 粗测阶段会注入 `TEST_iq = 1A`
- 若机构卡滞或方向错误，可能快速进入过流或机械冲击
- 低风险验证通过后再接入真实负载更稳妥

---

## 3. CCS Expressions 变量监控表

### 3.1 必须添加的控制变量

在 CCS Expressions 中添加：

```text
flagEnableMotor
flagClearFault
cmdCtrlMode
Iq_ref_A
speed_ref_rpm
pos_ref_rad
rpm1
rpm2
test_rpm1
ident_state
```

用途：

| 变量 | 用途 | 实验中如何使用 |
|---|---|---|
| `flagEnableMotor` | 使能电机 | 置 `true` 启动 |
| `flagClearFault` | 清故障 | 故障后置 `true` |
| `cmdCtrlMode` | 控制模式选择 | `0` 转矩，`1` 速度，`3` 辨识 |
| `Iq_ref_A` | 转矩模式电流指令 | 小电流测试用 |
| `speed_ref_rpm` | 速度模式速度指令 | 普通速度环测试用 |
| `rpm1` | 辨识第一正弦幅值 | 第一次设 `100` |
| `rpm2` | 辨识第二正弦幅值 | 第一次设 `200` |
| `test_rpm1` | 辨识后测试速度 | 建议 `100` 或 `200` |
| `ident_state` | 辨识状态机 | 判断辨识进度 |

### 3.2 必须添加的状态变量

```text
motorState
faultFlags.all
faultFlags.bit.overVoltage
faultFlags.bit.underVoltage
faultFlags.bit.overCurrent
faultFlags.bit.encoderFault
faultFlags.bit.driverFault
foc.vDC_V
```

用途：

| 变量 | 正常预期 |
|---|---|
| `motorState` | 最终应进入 `4`，即 `MTR_STATE_RUN` |
| `faultFlags.all` | 应始终为 `0` |
| `foc.vDC_V` | 应接近实际母线电压，例如 20 V 左右 |
| `overCurrent` | 必须为 `0` |
| `encoderFault` | 必须为 `0` |

### 3.3 必须添加的 FOC 变量

```text
foc.iqRef_A
foc.iDQ.q
foc.iDQ.d
foc.vDQ.q
foc.vDQ.d
foc.enc.speedMech_rads
foc.enc.speedMech_rpm
foc.enc.posMech_rad
foc.enc.thetaElec_rad
foc.duty.a
foc.duty.b
foc.duty.c
```

正常预期：

| 变量 | 正常现象 |
|---|---|
| `foc.iqRef_A` | 电流指令不应长期顶到 ±5 A |
| `foc.iDQ.q` | 应跟随 `foc.iqRef_A` |
| `foc.iDQ.d` | 应接近 0 A |
| `foc.vDQ.q` | 不应长期顶到 ±5 V |
| `speedMech_rpm` | 应跟随目标速度变化 |
| `thetaElec_rad` | 运行时应连续在 0 到 2π 变化 |
| `duty.a/b/c` | 应在 0.02 到 0.98 之间 |

### 3.4 必须添加的辨识结果变量

```text
ident_res.Omega_Ref
ident_res.Jm_Active
ident_res.Bm_Active
ident_res.Cm_Active
ident_res.Enable_FF
ident_res.Is_Finished
```

正常预期：

| 变量 | 正常现象 |
|---|---|
| `Omega_Ref` | 辨识时为正弦速度指令 |
| `Jm_Active` | 应从小值逐渐变为稳定正值 |
| `Bm_Active` | 通常应为正值 |
| `Cm_Active` | 通常应为正值 |
| `Enable_FF` | 第一次 `STATE_EVALUATE` 后变为 `1` |
| `Is_Finished` | 完整辨识结束后变为 `1` |

---

## 4. VOFA 观察通道建议

### 4.1 当前代码默认 VOFA 通道

当前 `main.c` 中 VOFA 输出为：

```text
CH1 = foc.iqRef_A
CH2 = foc.iDQ.q
CH3 = foc.iDQ.d
CH4 = foc.enc.thetaElec_rad
```

这组通道适合验证电流环：

| 通道 | 观察目的 |
|---|---|
| CH1 | q 轴电流指令 |
| CH2 | q 轴实际电流 |
| CH3 | d 轴实际电流 |
| CH4 | 电角度 |

正常现象：

- CH2 应跟随 CH1
- CH3 应接近 0
- CH4 应连续变化
- CH1 不应长期顶到 ±5 A

### 4.2 辨识实验更推荐的 VOFA 通道

为了看清辨识效果，建议临时改成：

```text
CH1 = ident_res.Omega_Ref * 60.0f / MATH_TWO_PI
CH2 = foc.enc.speedMech_rads * 60.0f / MATH_TWO_PI
CH3 = foc.iqRef_A
CH4 = foc.iDQ.q
```

含义：

| 通道 | 含义 | 正常现象 |
|---|---|---|
| CH1 | 目标速度 rpm | 0.5 Hz 正弦 |
| CH2 | 实际速度 rpm | 跟随 CH1 |
| CH3 | q 轴电流指令 | 随速度和加速度变化 |
| CH4 | q 轴实际电流 | 跟随 CH3 |

算法成功时，CH1 与 CH2 的关系应满足：

- 频率一致
- 方向一致
- 幅值接近
- 没有明显削顶
- 没有明显反相
- 过零附近不应长时间卡住

---

## 5. 第一部分：硬件与 FOC 基础验证

### 5.1 上电前设置

确认：

```text
flagEnableMotor = false
cmdCtrlMode = 0
Iq_ref_A = 0
speed_ref_rpm = 0
rpm1 = 100
rpm2 = 200
```

确认电机固定可靠，周围无机械干涉。

### 5.2 烧录并进入调试

在 CCS 中：

1. 编译工程
2. 烧录到芯片
3. 进入 Debug
4. 打开 Expressions
5. 打开 VOFA
6. 确认串口数据正常刷新

### 5.3 使能电机

设置：

```text
flagEnableMotor = true
```

观察 `motorState`。

正常状态顺序应为：

```text
0 -> 1 -> 2 -> 3 -> 4
```

对应：

```text
IDLE -> OFFSET_CAL -> ALIGN -> Z_CAL -> RUN
```

### 5.4 成功判据

必须同时满足：

```text
motorState = 4
faultFlags.all = 0
foc.vDC_V > 10
foc.vDC_V < 25
```

并且：

```text
foc.iDQ.d 没有大幅偏置
foc.iDQ.q 没有大幅偏置
foc.duty.a/b/c 不贴边
```

### 5.5 失败处理

若 `motorState = 5`，即故障：

1. 查看 `faultFlags.bit.overCurrent`
2. 查看 `faultFlags.bit.underVoltage`
3. 查看 `faultFlags.bit.encoderFault`
4. 设置 `flagEnableMotor = false`
5. 设置 `flagClearFault = true`
6. 不要继续辨识

---

## 6. 第二部分：电流环验证

### 6.1 切到转矩模式

设置：

```text
cmdCtrlMode = 0
Iq_ref_A = 0
```

确认电机已经在：

```text
motorState = 4
faultFlags.all = 0
```

### 6.2 小电流正向测试

依次设置：

```text
Iq_ref_A = 0.1
Iq_ref_A = 0.2
Iq_ref_A = 0.3
```

每个电流保持 2 秒。

观察：

```text
foc.iqRef_A
foc.iDQ.q
foc.iDQ.d
foc.enc.speedMech_rpm
faultFlags.all
```

正常现象：

- `foc.iDQ.q` 应接近 `Iq_ref_A`
- `foc.iDQ.d` 应接近 0
- 电机会轻微转动或产生力矩
- 不应过流
- 不应明显抖动或尖叫

### 6.3 小电流反向测试

依次设置：

```text
Iq_ref_A = -0.1
Iq_ref_A = -0.2
Iq_ref_A = -0.3
```

正常现象：

- 电机力矩方向应反向
- `foc.iDQ.q` 应跟随负电流
- `foc.iDQ.d` 仍接近 0

### 6.4 电流环通过判据

通过条件：

```text
abs(foc.iDQ.q - foc.iqRef_A) 不应长期很大
abs(foc.iDQ.d) 不应长期大于 0.2A
faultFlags.all = 0
```

若 `foc.iDQ.q` 与 `foc.iqRef_A` 比例明显不对，必须先检查电流采样比例，不要继续辨识。

---

## 7. 第三部分：普通速度环验证

### 7.1 切到速度模式

先清零转矩指令：

```text
Iq_ref_A = 0
```

设置：

```text
cmdCtrlMode = 1
speed_ref_rpm = 0
```

### 7.2 低速正向测试

依次设置：

```text
speed_ref_rpm = 50
speed_ref_rpm = 100
speed_ref_rpm = 200
```

每个速度保持 3 秒。

观察：

```text
foc.enc.speedMech_rpm
foc.iqRef_A
foc.iDQ.q
faultFlags.all
```

正常现象：

- 实际速度方向与给定一致
- 实际速度能稳定到给定附近
- `foc.iqRef_A` 不应长期饱和在 ±1 A
- 不应进入故障

### 7.3 低速反向测试

依次设置：

```text
speed_ref_rpm = -50
speed_ref_rpm = -100
speed_ref_rpm = -200
```

正常现象：

- 实际速度方向反向
- 速度能稳定
- 没有明显失控

### 7.4 速度环通过判据

通过条件：

```text
50 rpm、100 rpm、200 rpm 都能稳定运行
速度方向正确
faultFlags.all = 0
```

如果普通速度环都无法稳定，暂时不要进入 `CTRL_MODE_IDENT`。

---

## 8. 第四部分：辨识状态机低风险验证

### 8.1 设置低风险辨识参数

在 Expressions 中设置：

```text
rpm1 = 100
rpm2 = 200
test_rpm1 = 100
```

确保：

```text
cmdCtrlMode = 1 或 0
speed_ref_rpm = 0
Iq_ref_A = 0
```

### 8.2 准备进入辨识

必须确认已执行：

```text
Ident_reset()
```

然后设置：

```text
cmdCtrlMode = 3
```

### 8.3 观察状态机

观察：

```text
ident_state
ident_res.Omega_Ref
foc.enc.speedMech_rpm
foc.iqRef_A
foc.iDQ.q
ident_res.Jm_Active
ident_res.Bm_Active
ident_res.Cm_Active
ident_res.Enable_FF
```

状态机预期顺序：

```text
0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 2 -> 3 -> 4 -> 5 ... -> 6
```

状态含义：

| 状态值 | 状态名 | 预期现象 |
|---|---|---|
| 0 | `STATE_ROUGH_J` | 注入 1 A 粗测，电机短暂加速 |
| 1 | `STATE_RETURN_ZERO` | 速度给 0，等待停稳 |
| 2 | `STATE_INT_A1` | 100 rpm，0.5 Hz 正弦往复 |
| 3 | `STATE_INT_A2` | 200 rpm，0.5 Hz 正弦往复 |
| 4 | `STATE_EVALUATE` | 计算 `J/B/C` |
| 5 | `STATE_ITER_DELAY` | 停顿 0.5 s |
| 6 | `STATE_SUCCESS` | 辨识完成 |

### 8.4 各阶段预期现象

#### STATE_ROUGH_J = 0

持续时间约：

```text
0.3 s
```

现象：

- `foc.iqRef_A` 约为 `1A`
- 电机有短暂加速
- `ident_state` 很快进入 `1`

异常：

- 若马上过流，停止实验
- 若速度方向不对，检查编码器方向和 FOC 角度

#### STATE_RETURN_ZERO = 1

现象：

- `ident_res.Omega_Ref = 0`
- 速度逐渐回到接近 0
- 当 `abs(speedMech_rads) < 1` 后进入 `2`

异常：

- 若长时间不进入 `2`，说明速度反馈没有回零或存在漂移

#### STATE_INT_A1 = 2

现象：

- `ident_res.Omega_Ref` 为 0.5 Hz 正弦
- 速度幅值约为 `100 rpm`
- 首轮会跑约 4 个周期，即约 8 秒
- `Jm_Active` 会逐渐变化

异常：

- 若 `ident_state` 卡在 `2`
- 且 `ident_res.Omega_Ref` 一直为 0
- 则高度怀疑 `dtheta` 未初始化

#### STATE_INT_A2 = 3

现象：

- 正弦速度幅值变为约 `200 rpm`
- 持续约 2 秒
- 之后进入 `4`

异常：

- 若速度明显削顶，说明电流或电压限幅影响辨识

#### STATE_EVALUATE = 4

现象：

- `Bm_Active` 更新
- `Cm_Active` 更新
- `Enable_FF` 变成 `1`

异常：

- 若 `Bm_Active` 或 `Cm_Active` 数值巨大或符号反复乱跳，说明速度跟踪或转矩估算不可靠

### 8.5 低风险验证通过判据

必须满足：

```text
ident_state 能从 0 走到 2
ident_res.Omega_Ref 能生成正弦
实际速度能跟随正弦
faultFlags.all = 0
foc.iDQ.q 能跟随 foc.iqRef_A
```

推荐满足：

```text
ident_state 至少走完一次 2 -> 3 -> 4
Enable_FF 变为 1
Jm_Active 为正值
```

---

## 9. 第五部分：完整辨识实验

### 9.1 设置正式辨识参数

低风险验证通过后，停止电机：

```text
flagEnableMotor = false
```

等待进入空闲后重新开始。

设置：

```text
rpm1 = 300
rpm2 = 600
test_rpm1 = 200
```

重新使能：

```text
flagEnableMotor = true
```

待进入：

```text
motorState = 4
faultFlags.all = 0
```

执行 `Ident_reset()`，然后设置：

```text
cmdCtrlMode = 3
```

### 9.2 完整辨识预计时间

参数：

```text
W_H = π rad/s
正弦频率 = 0.5 Hz
周期 = 2 s
A1_SETTLE_CYCLES = 4
MACRO_ITER_MAX = 8
ITER_DELAY_TIME = 0.5 s
```

预计时间：

- 粗测：约 0.3 s
- 回零：约 1 到 3 s
- 首轮 A1：约 8 s
- 首轮 A2：约 2 s
- 后续每轮：A1 约 2 s，A2 约 2 s，间歇 0.5 s
- 总时间：约 35 到 50 s

### 9.3 完整辨识过程应观察的重点

观察：

```text
ident_state
ident_res.Jm_Active
ident_res.Bm_Active
ident_res.Cm_Active
ident_res.Enable_FF
ident_res.Is_Finished
foc.enc.speedMech_rpm
foc.iqRef_A
foc.iDQ.q
foc.iDQ.d
faultFlags.all
```

预期：

- `ident_state` 多次循环 `2 -> 3 -> 4 -> 5`
- `Enable_FF` 在第一次 `STATE_EVALUATE` 后变为 `1`
- `Jm_Active` 后几轮趋于稳定
- `Bm_Active` 后几轮趋于稳定
- `Cm_Active` 后几轮趋于稳定
- 最终 `Is_Finished = 1`
- 最终 `ident_state = 6`

### 9.4 完整辨识成功判据

最低成功判据：

```text
ident_state = 6
ident_res.Is_Finished = 1
faultFlags.all = 0
ident_res.Jm_Active > 0
```

较好成功判据：

```text
ident_res.Bm_Active > 0
ident_res.Cm_Active > 0
后 3 轮 Jm_Active 变化小于 10%
速度正弦没有明显削顶
电流实际值能跟随电流指令
```

高质量成功判据：

```text
重复辨识 2 次，Jm_Active 数量级一致
重复辨识 2 次，Bm_Active 数量级一致
重复辨识 2 次，Cm_Active 数量级一致
开启摩擦前馈后低速跟踪明显改善
```

---

## 10. 第六部分：辨识结果验证

### 10.1 使用测试状态验证

辨识完成后设置：

```text
ident_state = 7
test_rpm1 = 100
```

观察：

```text
ident_res.Omega_Ref
foc.enc.speedMech_rpm
foc.iqRef_A
foc.iDQ.q
ident_res.Bm_Active
ident_res.Cm_Active
```

预期：

- `Omega_Ref` 对应 100 rpm
- 实际速度接近 100 rpm
- `Iq_ref_A` 中包含摩擦前馈分量
- 低速运行不应明显卡顿

再设置：

```text
test_rpm1 = -100
```

预期：

- 电机反向稳定运行
- 电流前馈方向反向
- 速度稳定

### 10.2 对比摩擦前馈效果

在 Expressions 中对比：

```text
ident_res.Enable_FF = 0
ident_res.Enable_FF = 1
```

分别观察低速运行。

成功现象：

- `Enable_FF = 1` 时，低速速度误差更小
- 过零附近更顺
- `foc.iqRef_A` 变化更主动
- `foc.enc.speedMech_rpm` 不容易卡住

### 10.3 重复性验证

完整辨识建议至少重复 2 次。

每次记录：

```text
rpm1
rpm2
Jm_Active
Bm_Active
Cm_Active
是否空载
是否带负载
母线电压
是否发生故障
```

记录模板：

```markdown
| 次数 | rpm1 | rpm2 | Jm_Active | Bm_Active | Cm_Active | 是否完成 | 备注 |
|---|---:|---:|---:|---:|---:|---|---|
| 1 | 300 | 600 |  |  |  |  |  |
| 2 | 300 | 600 |  |  |  |  |  |
| 3 | 300 | 600 |  |  |  |  |  |
```

重复性判断：

- `Jm_Active` 不应每次差一个数量级
- `Bm_Active` 不应正负乱跳
- `Cm_Active` 不应正负乱跳
- 空载和带载时 `Jm_Active` 可以不同，带载通常更大

---

## 11. 常见异常与判断

### 11.1 卡在 `STATE_INT_A1 = 2`

现象：

```text
ident_state = 2
ident_res.Omega_Ref = 0 或几乎不变
速度不动
```

最可能原因：

```text
Ident_reset() 未执行
dtheta 未初始化
```

处理：

```text
先解决 Ident_reset() 初始化问题
不要继续跑
```

### 11.2 一进入辨识就过流

现象：

```text
faultFlags.bit.overCurrent = 1
motorState = 5
```

可能原因：

- 电流采样比例错误
- FOC 电角度偏置错误
- 编码器方向错误
- `TEST_iq = 1A` 对当前机构太大
- 机械卡住

处理：

- 回到转矩模式 `0.1A` 验证
- 降低粗测电流
- 不要直接跑完整辨识

### 11.3 正弦速度明显削顶

现象：

```text
目标速度是正弦
实际速度顶部变平
foc.iqRef_A 长期接近 ±5A
```

可能原因：

- `rpm1/rpm2` 太大
- 速度环带宽不足
- 电流限幅不足
- 电压限幅不足
- 负载惯量太大

处理：

- 降低 `rpm1/rpm2`
- 先用 `100/200 rpm`
- 确认电流环没有饱和

### 11.4 `Bm_Active` 或 `Cm_Active` 为负

少量波动可以接受，但若长期为负，通常说明：

- 电流方向与速度方向符号不一致
- 编码器方向反
- `Iq_ref` 与真实转矩方向不一致
- 速度跟踪严重滞后
- 电流采样比例或极性错误

处理：

- 先回到转矩模式验证正负电流对应正负速度变化
- 再回到普通速度模式验证正负速度闭环

### 11.5 `Jm_Active` 极大或接近上限

若接近：

```text
JM_ABSOLUTE_MAX = 0.001
```

可能原因：

- 粗测阶段速度变化太小
- 电机没有转起来
- 负载太大
- 摩擦太大
- 电流实际没有跟上
- 编码器速度反馈异常

处理：

- 空载重试
- 检查速度反馈
- 检查电流环
- 降低机械负载

---

## 12. 最终验收标准

本功能可以认为已经在实物上成功实现，当且仅当满足以下条件：

```text
1. 电机能稳定进入 MTR_STATE_RUN
2. 普通转矩模式和速度模式都能低速稳定运行
3. CTRL_MODE_IDENT 能跑到 STATE_SUCCESS
4. ident_res.Is_Finished = 1
5. faultFlags.all = 0
6. Jm_Active 为稳定正值
7. Bm_Active 和 Cm_Active 不发散
8. 正弦目标速度和实际速度方向一致、频率一致
9. 电流实际值能跟随电流指令
10. 重复实验结果数量级一致
```

如果还满足以下条件，则说明算法不仅跑通，而且效果较好：

```text
1. 后 3 轮 Jm_Active 变化小于 10%
2. 开启 Enable_FF 后低速速度误差明显减小
3. 过零附近卡顿明显减轻
4. 300/600 rpm 正式辨识也能稳定完成
```

---

## 13. 推荐实验顺序简表

```text
步骤 1：flagEnableMotor = true，确认 motorState = RUN
步骤 2：cmdCtrlMode = 0，Iq_ref_A = ±0.1 到 ±0.3，验证电流环
步骤 3：cmdCtrlMode = 1，speed_ref_rpm = ±50 到 ±200，验证速度环
步骤 4：rpm1 = 100，rpm2 = 200，执行 Ident_reset()
步骤 5：cmdCtrlMode = 3，观察 ident_state 是否跑通
步骤 6：若低风险通过，rpm1 = 300，rpm2 = 600，重新完整辨识
步骤 7：观察 Is_Finished、Jm/Bm/Cm
步骤 8：ident_state = 7，test_rpm1 = ±100，验证低速跟踪
步骤 9：对比 Enable_FF = 0 和 Enable_FF = 1
步骤 10：重复 2 到 3 次，记录结果
```
