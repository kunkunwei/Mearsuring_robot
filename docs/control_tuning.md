# 控制模块与调参指南（v3.0）

本文档说明 v3.0 固件的控制架构、每个参数的位置与含义，以及实车调参顺序。

## 1. 控制架构

`Chassis_ControlManager_Update()`（`Components/Controller/Src/chassis_control_manager.c`）每个控制周期执行一次，状态机：

```
DISABLED --(有指令)--> DRIVE --(指令归零)--> BRAKE --(低速持续)--> HOLD
   |                      |                    |                    |
   +------ 任何状态下触发保护条件 ------> FAULT（锁存，需重新上电/重新使能）
```

- **DRIVE**：有速度指令，按轮执行有界 MIT 电流控制 + 转向破静摩擦补偿 + pitch 重力前馈。
- **BRAKE**：指令归零后立即制动（速度阻尼 + 斜坡位置补偿）。
- **HOLD**：四轮速度低于阈值持续 100 ms 后进入，每轮独立位置闭环 + 重力前馈（驻坡/平地锁车）。
- **FAULT**：反馈丢失、超速、震荡、饱和、时序异常时锁存，输出 0 电流。

VOFA+ 调试帧第 16 通道为诊断码：`state*100 + fault*10 + valid`。
state：0=DISABLED，1=DRIVE，2=BRAKE，3=HOLD，4=FAULT；
fault：0=无，1=反馈丢失，2=超速，3=震荡，4=饱和，5=时序。

## 2. 控制器参数

全部默认值位于 `Chassis_ControlManager_DefaultConfig()`（`Components/Controller/Src/chassis_control_manager.c`）。

### 2.1 DRIVE（`config.drive`）

| 参数 | 默认 | 含义与调节 |
| --- | --- | --- |
| `position_kp_a_per_deg` | 0.005 | 位置跟踪刚度（A/deg）。越大跟得越紧，过大会震荡 |
| `speed_kd_a_per_rpm` | 0.008 | 速度阻尼（A/rpm）。抑制超调；过大会发闷、响应慢 |
| `speed_ki_a_per_rpm_s` | 0.01 | 速度积分（A/(rpm·s)）。消除稳态静差；过大引起抖动/振荡 |
| `speed_integral_limit_a` | 1.0 | 积分项上限，防积分饱和 |
| `speed_error_current_limit_a` | 1.5 | **速度误差通道（kd+积分）独立限幅（A）**，与摩擦/破静摩擦前馈分开限，防止多路前馈在拖滞轮上叠加成 2.7A 级别的高电流 |
| `friction_current_a` | 1.0 | 滚动摩擦前馈（A），按目标转速 tanh 平滑建立，用于匀速段补偿 |
| `friction_rpm_scale` | 100 | 摩擦前馈达到饱和的目标转速尺度（rpm） |
| `position_error_limit_deg` | 5.0 | 位置误差限幅，防止急停/急转产生电流尖峰 |
| `current_limit_a` | 3.5 | 单轮驱动电流上限（A） |

### 2.2 BRAKE（`config.brake`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `speed_gain_a_per_rpm` | 0.010 | 制动速度阻尼 |
| `current_limit_a` | 3.0 | 制动电流上限 |

### 2.3 HOLD（`config.hold`，驻坡核心）

| 参数 | 默认 | 含义与调节 |
| --- | --- | --- |
| `position_kp_a_per_deg` | 0.010 | 每轮位置闭环刚度，防止单轮在 HOLD 中空转 |
| `speed_kd_a_per_rpm` | 0.010 | HOLD 速度阻尼 |
| `pitch_feedforward_a` | -8.0 | 重力前馈系数：`电流 = k * sin(修正 pitch)`。上坡 pitch<0 得正向电流，用于坡上防止溜车 |
| `pitch_zero_offset_rad` | 3.7° | 平地零偏。**v3.0 换轴后需在平地上重新标定**：调到此值使平地修正 pitch ≈ 0 |
| `current_limit_a` | 6.5 | HOLD 电流上限（驻坡时需要大于 DRIVE 上限） |

### 2.4 斜坡位置补偿（`brake_position_comp_*`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `brake_position_comp_off_pitch_rad` | 3° | 坡度低于此值不启用位置补偿 |
| `brake_position_comp_full_pitch_rad` | 5° | 坡度高于此值全量补偿 |

### 2.5 转向破静摩擦（`turn_breakaway_*`）

| 参数 | 默认 | 含义与调节 |
| --- | --- | --- |
| `turn_breakaway_current_a` | 2.5 | 纯转向且该轮尚未跟上时，额外注入的破静摩擦电流（A） |
| `turn_breakaway_target_rpm` | 30 | 补偿随目标转速平滑建立的速度尺度（rpm） |
| `turn_breakaway_enter_rpm` | 15 | 轮速低于该值才介入（起转阶段） |
| `turn_breakaway_release_ratio` | 0.90 | 轮速达到目标转速的该比例后释放，配合进入阈值形成滞回，**避免补偿阈值贴着工作点（如 77 vs 80）造成极限环** |
| `turn_breakaway_hold_time_s` | 0.50 | 持续介入超过该时间仍未跟上目标，开始衰减 |
| `turn_breakaway_taper_time_s` | 0.25 | 超时后电流线性衰减到 0 的时间，防止拖滞轮被长期加压 |

### 2.5.1 纯转向同步降速（`turn_sync_*`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `turn_sync_enabled` | 1 | 纯转向时以最慢轮为基准，四轮目标同步缩放 |
| `turn_sync_min_ratio` | 0.25 | 同步比例下限（目标最多降到此比例） |
| `turn_sync_time_s` | 0.10 | 同步比例低通时间常数，防止速度波动导致目标抖动 |

### 2.5.2 长期拖滞轮限制（`drive_slip_*`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `drive_slip_error_ratio` | 0.40 | 轮速落后目标超过该比例（<60%）即累计拖滞时间 |
| `drive_slip_recover_ratio` | 0.20 | 轮速回到目标的 80% 以上即恢复（滞回） |
| `drive_slip_confirm_time_s` | 0.50 | 拖滞持续该时间后，该轮停止跟踪（积分清零、速度/摩擦前馈置 0，仅保留 pitch 重力前馈） |

### 2.6 限幅与保护

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `current_rise_a_per_s` | 30 | 电流上升斜率限幅（A/s），防冲击 |
| `current_release_a_per_s` | 60 | 电流释放斜率限幅（A/s） |
| `command_deadband_rpm` | 3.0 | 指令死区，低于此转速视为 0 |
| `hold_enter_speed_rpm` | 5.0 | 进入 HOLD 的四轮速度阈值 |
| `hold_enter_time_s` | 0.100 | 低速持续该时间才进入 HOLD |
| `hold_overspeed_rpm` / `_time_s` | 200 / 0.5 | HOLD 中无指令但轮速过高 → 超速故障 |
| `oscillation_window_s` | 0.100 | 电流换向统计窗口 |
| `oscillation_min_current_a` | 0.30 | 换向统计的最小电流 |
| `oscillation_reversal_limit` | 4 | 窗口内换向次数上限 → 震荡故障 |
| `saturation_time_s` | 0.100 | 持续顶电流上限 → 饱和故障 |
| `dt_min_s` / `dt_max_s` | 3 / 8 ms | 控制周期合法范围 |
| `timing_fault_count_limit` | 3 | 周期异常连续次数 → 时序故障 |

## 3. 底盘任务参数（`Application/Tasks/Inc/Chassis_Task.h`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `NORMAL_MAX_CHASSIS_SPEED_X` | 0.5 m/s | ROS/手动目标 vx 限幅 |
| `NORMAL_MAX_CHASSIS_SPEED_WZ` | 15.0 rad/s | 目标 wz 限幅 |
| `MAX_WHEEL_SPEED` | 3.6 m/s | 四轮速度等比限幅 |
| `WHEEL_R` | 0.025 m | 轮半径（运动学 + 里程计共用） |
| `CHASSIS_WHEEL_TRACK` | 0.081 m | 左右轮距（差速 yaw 计算，**务必实测**） |
| `CHASSIS_WHEEL_BASE` | 0.085 m | 前后轴距 |
| `CHASSIS_MOTOR_*_FORWARD_SIGN` | ±1 | 电机接线方向；改接线或改这里，不能两者都改 |
| `CHASSIS_CONTROL_TIME_MS` | 2 ms | 底盘任务周期 |
| `CHASSIS_TORQUE_CONTROL_PERIOD_MS` | 5 ms | 实际电流控制/发布周期 |

## 4. 里程计参数（`Components/Algorithm/Src/odometry.c`，`odom_default_config`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `wheel_radius_m` | 0.025 | 轮半径，直行距离精度主要来源 |
| `wheel_track_m` | 0.081 | 轮距，差速 yaw 计算 |
| `imu_yaw_turn_weight` | **1.0** | 转弯时 yaw 纯 IMU，不混编码器 |
| `imu_yaw_straight_weight` | 0.20 | 直行时 IMU yaw 权重（其余为编码器） |
| `straight_wz_threshold` | 0.08 | 判定转弯的 wz 阈值 |
| `slope_pitch_threshold` | 0.10 rad | 斜坡模式：后轴优先、前轮降权 |
| `wheel_speed_outlier_ratio` / `_offset` | 2.0 / 0.08 | 异常轮速权重削减 |
| `max_wheel_delta_m` | 0.08 | 单周期轮位移上限，超限弃用该轮 |
| `still_vx_threshold` / `still_wz_threshold` | 0.004 / 0.015 | 静止判定 |
| `straight_pair` | REAR | 直行距离优先采用后轴 |

## 5. 观测任务参数（`Application/Tasks/Src/observe_task.c`）

| 参数 | 位置 | 含义 |
| --- | --- | --- |
| `vaEstimateKF_Q` / `vaEstimateKF_R` | 文件顶部 | 速度卡尔曼：Q 大跟踪快噪声大，R 大平滑滞后 |
| `ODOM_STILL_*` | 文件顶部 | 静止判定阈值（vx/wz/加速度） |
| `magnetic_heading_config` | 文件顶部 | 磁航向：**校准为零（bias=0、soft-iron=单位阵），尚未标定**；innovation 限 45°，修正增益 0.25，最大修正速率 2°/s |

## 6. INS 参数（`Application/Tasks/Src/INS_Task.c`）

| 参数 | 位置 | 含义 |
| --- | --- | --- |
| IMU 安装映射 | 主循环角度转换处 | 板载 IMU 相对车体绕 Z 逆时针 90°：`angle[1]=+Euler[2]`（车体 pitch），`angle[2]=−Euler[1]`（车体 roll），`angle[0]=Euler[0]`（yaw 增量不受常数偏移影响） |
| `INS_LPF2p_Alpha` | 文件顶部 | 加速度二阶低通 |
| `yaw_low_filter` / `yaw_gyro_low_filter` | 文件顶部 | yaw / yaw 角速度一阶低通（num=0.2） |
| yaw 冻结阈值 | 主循环（0.039 rad/s） | 角速度低于阈值时冻结 `yaw_angle`（抑制静止零漂；odom 用 Euler 增量不受影响） |
| `QuaternionEKF_Init` 参数 | `INSTask_Init` | 过程噪声 10 / 0.001、观测噪声 1000000 |
| `BMI088_Temp_Control` | 主循环 | IMU 恒温 40°C |

## 7. VOFA+ 在线调参

通过 USART6 向底盘发送文本命令（User_Task 中开启 `Vofa_Process_RxCommand`）：

| 命令 | 实际作用 | 上限 |
| --- | --- | --- |
| `KP=xx` | `drive.position_kp`（mA/deg） | 50 |
| `KI=xx` | `drive.speed_kd`（mA/rpm） | 50 |
| `KD=xx` | `drive.friction_current`（mA） | 500 |

> 注意：命令名沿用了 kp/ki/kd，但实际映射到的是位置增益、速度阻尼、摩擦前馈，不是真正的 PID 三项。在线调参只改 DRIVE 组，掉电丢失。

## 8. VOFA+ 调试通道（`Vofa_Send_ChassisPipeline_Debug`，20 通道）

| 通道 | 内容 |
| --- | --- |
| 0-3 | 四轮目标转速 rpm |
| 4-7 | 四轮反馈转速 rpm |
| 8-11 | CAN 快照电流 A（控制器输出） |
| 12-15 | VESC 反馈电流 A |
| 16 | 诊断码 `state*100+fault*10+valid` |
| 17 | CAN 发送丢帧计数 |
| 18 | CAN 队列高水位 |
| 19 | 修正后 pitch（deg） |

## 9. 实车调参顺序建议

1. **先修机械**：导轮螺丝紧固、整车无松旷，否则数据不可信。
2. 平地低速直行：确认四轮反馈转速一致、无抖动；`friction_current_a`/`speed_ki` 负责稳态静差，`speed_kd` 负责超调。
3. 原地旋转：确认 odom yaw 方向与比例（v3.0 转弯为纯 IMU）；`turn_breakaway_current_a` 只影响起步破静摩擦，不应长期压在高转速轮上（超过 `turn_breakaway_speed_rpm` 自动退出）。
4. 纵向驻坡：HOLD 进入后 pitch 为负、前馈为正电流、四轮各自锁位；重新标定 `pitch_zero_offset_rad`。
5. 分段清零：上坡/过路口后发 `0x33` 帧，等待 `segment_id` 确认，再开始新段累计。
