# 齿槽力矩补偿功能实验计划

## Summary

目标是验证三件事：

- 齿槽数据能否完整采集：`cog_pos_samples / cog_neg_samples` 增长，`cog_missing_bins` 最终为 `0`
- LUT 能否生成并启用：`cog_generate_req -> cog_ready = 1`
- 补偿前后低速运行是否改善：低速转速波动、`Iq` 波动、卡顿/爬行感是否下降

实验默认在**机械参数辨识完成后**做，因为齿槽记录阶段会用 `ident_res.Bm_Active / Cm_Active` 扣除摩擦影响。

## Expressions 变量

启动/模式：

```text
flagEnableMotor
flagClearFault
motorState
cmdCtrlMode
speed_ref_rpm
Iq_ref_A
faultFlags.all
```

辨识结果：

```text
ident_state
ident_res.Is_Finished
ident_res.Enable_FF
ident_res.Jm_Active
ident_res.Bm_Active
ident_res.Cm_Active
fric_iqComp_A
```

齿槽补偿：

```text
cog_enable
cog_ready
cog_record_mode
cog_generate_req
cog_reset_req
cog_missing_bins
cog_pos_samples
cog_neg_samples
cog_last_index
cog_gain
cog_iqRecord_A
cog_iqRaw_A
cog_iqComp_A
```

运行观察：

```text
foc.enc.speedMech_rads
foc.enc.speedMech_rpm
foc.enc.rawAngle
foc.iqRef_A
foc.iDQ.q
foc.vDQ.q
```

## 实验步骤

### 1. 上电启动并进入 RUN

先设置：

```text
flagEnableMotor = true
cmdCtrlMode = 1
speed_ref_rpm = 0
cog_enable = 0
cog_record_mode = 0
cog_gain = 0
```

等待状态机完成：

```text
motorState = 4
```

含义：

```text
0 IDLE
1 OFFSET_CAL
2 ALIGN
3 Z_CAL
4 RUN
5 FAULT
```

预期现象：

- 电机会经历零偏校准、对齐、找 Z 相，然后进入闭环运行
- `cog_zero_count` 会在 Z 相捕获后被设置
- 若进入 `motorState = 5`，先停机排查编码器/Z 相/过流，再 `flagClearFault = true`

### 2. 先做机械参数辨识

设置：

```text
cmdCtrlMode = 3
```

等待：

```text
ident_res.Is_Finished = 1
ident_res.Enable_FF = 1
```

预期现象：

- 电机会按照辨识算法做正弦速度激励
- `ident_res.Jm_Active / Bm_Active / Cm_Active` 逐步得到非零结果
- 辨识完成后 `ident_state` 应进入成功状态，`ident_res.Is_Finished = 1`

完成后切回速度模式：

```text
cmdCtrlMode = 1
speed_ref_rpm = 0
```

注意：后面普通速度模式里，`fric_iqComp_A` 才会使用辨识出来的摩擦参数。

### 3. 补偿前低速基线测试

先确保齿槽补偿关闭：

```text
cog_enable = 0
cog_gain = 0
cog_record_mode = 0
```

测试低速正转：

```text
cmdCtrlMode = 1
speed_ref_rpm = 10
```

保持 10 到 20 秒，记录/观察：

```text
foc.enc.speedMech_rpm
foc.iqRef_A
foc.iDQ.q
fric_iqComp_A
cog_iqComp_A
```

再测试反转：

```text
speed_ref_rpm = -10
```

同样保持 10 到 20 秒。

预期现象：

- `cog_iqComp_A = 0`
- `fric_iqComp_A` 在速度模式下应有正负变化
- 低速时可能能看到速度轻微周期波动、卡顿、爬行或电流周期纹波

这一步作为补偿前对照。

### 4. 清空旧齿槽数据

让电机低速停止：

```text
speed_ref_rpm = 0
```

然后设置：

```text
cog_enable = 0
cog_record_mode = 0
cog_reset_req = 1
```

等待主循环处理后观察：

```text
cog_reset_req = 0
cog_ready = 0
cog_missing_bins = 512
cog_pos_samples = 0
cog_neg_samples = 0
```

### 5. 正转采集齿槽数据

设置：

```text
cmdCtrlMode = 1
cog_enable = 0
cog_record_mode = 1
speed_ref_rpm = 10
```

保持至少 3 到 5 圈。  
10 rpm 时一圈约 6 秒，建议保持 25 到 40 秒。

观察：

```text
cog_pos_samples 持续增加
cog_last_index 在 0~511 间循环
cog_iqRecord_A 有小幅变化
cog_neg_samples 不应明显增加
```

正转采完后：

```text
speed_ref_rpm = 0
cog_record_mode = 0
```

预期现象：

- `cog_pos_samples` 明显大于 512
- 如果机械运行平稳，每个位置桶应基本都采到

### 6. 反转采集齿槽数据

设置：

```text
cmdCtrlMode = 1
cog_enable = 0
cog_record_mode = 2
speed_ref_rpm = -10
```

同样保持 25 到 40 秒。

观察：

```text
cog_neg_samples 持续增加
cog_last_index 在 0~511 间循环
cog_iqRecord_A 有小幅变化
cog_pos_samples 基本不再增加
```

反转采完后：

```text
speed_ref_rpm = 0
cog_record_mode = 0
```

预期现象：

- `cog_neg_samples` 明显大于 512
- 正反转都有足够样本

### 7. 生成齿槽 LUT

确认电机低速停止或保持很低速：

```text
speed_ref_rpm = 0
cog_record_mode = 0
cog_enable = 0
```

触发生成：

```text
cog_generate_req = 1
```

等待主循环处理后观察：

```text
cog_generate_req = 0
cog_record_mode = 0
cog_missing_bins
cog_ready
```

判断：

```text
如果 cog_missing_bins = 0 且 cog_ready = 1
说明 LUT 生成成功

如果 cog_missing_bins > 0 且 cog_ready = 0
说明有位置桶没采到，需要继续正转/反转采集
```

如果没采满，重复步骤 5 和 6，时间加长，或者把速度略降到 `5 rpm`，再重新：

```text
cog_generate_req = 1
```

### 8. 小增益启用齿槽补偿

先用小增益：

```text
cog_gain = 0.2
cog_enable = 1
cmdCtrlMode = 1
speed_ref_rpm = 10
```

观察：

```text
cog_iqRaw_A 随机械位置周期变化
cog_iqComp_A = cog_gain * cog_iqRaw_A
foc.iqRef_A 包含 PI + 摩擦 + 齿槽补偿
foc.enc.speedMech_rpm 是否更平滑
foc.iDQ.q 纹波是否变小
```

然后测试反转：

```text
speed_ref_rpm = -10
```

预期现象：

- 低速卡顿感减弱
- 速度波动变小
- `Iq` 周期性纹波下降
- `cog_iqComp_A` 不应超过 `±1A * cog_gain`

如果启用后抖动变大，先不要继续加增益，尝试：

```text
cog_gain = -0.2
```

如果反向后明显改善，说明补偿符号方向原来反了。

### 9. 增益扫描

在 `10 rpm` 正反转都正常后，逐步调整：

```text
cog_gain = 0.2
cog_gain = 0.4
cog_gain = 0.6
cog_gain = 0.8
cog_gain = 1.0
```

每档都分别测试：

```text
speed_ref_rpm = 10
speed_ref_rpm = -10
speed_ref_rpm = 5
speed_ref_rpm = -5
```

推荐记录表：

```text
cog_gain
speed_ref_rpm
速度波动大小
Iq 实际纹波
是否卡顿
是否噪声变大
是否过流/失稳
```

选择标准：

- 速度最平滑
- 电流纹波较小
- 无明显振动或噪声增加
- 正反转都稳定

通常不一定 `1.0` 最好，实际可能 `0.4~0.8` 更稳。

## 对比实验

### A 组：无齿槽补偿

```text
cog_enable = 0
cog_gain = 0
cmdCtrlMode = 1
speed_ref_rpm = 5, 10, -5, -10
```

记录：

```text
foc.enc.speedMech_rpm
foc.iqRef_A
foc.iDQ.q
fric_iqComp_A
cog_iqComp_A
```

预期：

```text
cog_iqComp_A = 0
低速可能有周期性速度波动
Iq 可能有周期纹波
```

### B 组：开启齿槽补偿

```text
cog_ready = 1
cog_enable = 1
cog_gain = 选定值，比如 0.4 或 0.6
cmdCtrlMode = 1
speed_ref_rpm = 5, 10, -5, -10
```

记录同样变量。

预期：

```text
cog_iqComp_A 随位置周期变化
低速速度波动下降
Iq 实际纹波下降或更规律
电机爬行/顿挫感减轻
```

### 判断结论

补偿有效的表现：

```text
同一低速下，补偿后速度更均匀
foc.iDQ.q 的周期波动变小
foc.iqRef_A 中出现与位置相关的 cog_iqComp_A
正反转都没有明显恶化
```

补偿异常的表现：

```text
cog_enable = 1 后抖动更大
低速噪声变大
某些角度突然冲击
正转有效但反转恶化
```

异常时优先处理顺序：

```text
1. 降低 cog_gain 到 0.2
2. 尝试 cog_gain = -0.2 判断符号
3. 检查 cog_missing_bins 是否为 0
4. 重新采集正反转数据
5. 确认采集时 cog_enable = 0
6. 确认机械参数辨识已完成
```

## Assumptions

- 电机可以安全低速正反转，建议从 `5~10 rpm` 开始。
- 齿槽 LUT 当前只在 RAM 中，掉电后需要重新采集。
- 采集时必须关闭齿槽补偿：`cog_enable = 0`。
- 启用补偿前必须生成成功：`cog_ready = 1` 且 `cog_missing_bins = 0`。
