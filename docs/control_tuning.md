# 控制模块与调参指南（safe-mit-control 分支）

本文档说明 `codex/safe-mit-control` 固件的控制架构、每个参数的位置与含义。该分支是 MIT 控制基础版，参数与 `codex/v3.0` 不同。

## 1. 控制架构

`Chassis_ControlManager_Update()`（`Components/Controller/Src/chassis_control_manager.c`）每个控制周期执行一次，状态机：

```
DISABLED --(有指令)--> DRIVE --(指令归零)--> BRAKE --(低速持续)--> HOLD
   |                      |                    |                    |
   +------ 任何状态下触发保护条件 ------> FAULT（锁存，需重新上电/重新使能）
```

- **DRIVE**：有速度指令，按轮执行有界 MIT 电流控制。
- **BRAKE**：指令归零后立即制动（速度阻尼）。
- **HOLD**：四轮速度低于阈值持续 200 ms 后进入，**四轮中值位置**闭环（单轮转动会被中值算法忽略）。
- **FAULT**：反馈丢失、超速、震荡、饱和、时序异常时锁存，输出 0 电流。

诊断码（`Vofa_Send_Brake_Debug_Info` 或状态观测）：`state*100 + fault*10 + valid`。
state：0=DISABLED，1=DRIVE，2=BRAKE，3=HOLD，4=FAULT；
fault：0=无，1=反馈丢失，2=超速，3=震荡，4=饱和，5=时序。

## 2. 控制器参数

全部默认值位于 `Chassis_ControlManager_DefaultConfig()`（`Components/Controller/Src/chassis_control_manager.c`）。

### 2.1 DRIVE（`config.drive`）

| 参数 | 默认 | 含义与调节 |
| --- | --- | --- |
| `position_kp_a_per_deg` | 0.0105 | 位置跟踪刚度（A/deg）。越大跟得越紧，过大会震荡 |
| `speed_kd_a_per_rpm` | 0.0105 | 速度阻尼（A/rpm）。抑制超调；过大会发闷 |
| `friction_current_a` | 0.25 | 滚动摩擦前馈（A），按目标转速 tanh 平滑建立 |
| `friction_rpm_scale` | 30 | 摩擦前馈达到饱和的目标转速尺度（rpm） |
| `position_error_limit_deg` | 5.0 | 位置误差限幅，防止电流尖峰 |
| `current_limit_a` | **0.5** | 单轮驱动电流上限（A）。**实测偏小，破静摩擦不足时可上调** |

### 2.2 BRAKE（`config.brake`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `speed_gain_a_per_rpm` | 0.010 | 制动速度阻尼 |
| `current_limit_a` | 0.5 | 制动电流上限 |

### 2.3 HOLD（`config.hold`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `position_kp_a_per_deg` | 0.010 | 四轮中值位置闭环刚度 |
| `speed_kd_a_per_rpm` | 0.010 | HOLD 速度阻尼 |
| `pitch_feedforward_a` | **0.0** | 重力前馈**默认关闭**（坡上驻车不完整，v3.0 已启用并加入零偏） |
| `current_limit_a` | 0.5 | HOLD 电流上限 |

### 2.4 状态机与保护

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `command_deadband_rpm` | 3.0 | 指令死区 |
| `hold_enter_speed_rpm` | 5.0 | 进入 HOLD 的四轮速度阈值 |
| `hold_enter_time_s` | 0.200 | 低速持续该时间才进入 HOLD |
| `current_slew_a_per_s` | 10 | 电流斜率限幅（A/s），防冲击 |
| `hold_overspeed_rpm` | 50 | HOLD 中无指令但轮速过高 → 超速故障 |
| `oscillation_window_s` | 0.100 | 电流换向统计窗口 |
| `oscillation_reversal_limit` | 4 | 窗口内换向次数上限 → 震荡故障 |
| `saturation_time_s` | 0.100 | 持续顶电流上限 → 饱和故障 |
| `dt_min_s` / `dt_max_s` | 3 / 8 ms | 控制周期合法范围 |
| `timing_fault_count_limit` | 3 | 周期异常连续次数 → 时序故障 |

> 注意：本分支**没有** v3.0 的转向破静摩擦（`turn_breakaway_*`）、斜坡位置补偿（`brake_position_comp_*`）和 `pitch_zero_offset`。

## 3. 底盘任务参数（`Application/Tasks/Inc/Chassis_Task.h`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `NORMAL_MAX_CHASSIS_SPEED_X` | 2.0 m/s | ROS/手动目标 vx 限幅（v3.0 已降为 0.5） |
| `NORMAL_MAX_CHASSIS_SPEED_WZ` | 15.0 rad/s | 目标 wz 限幅 |
| `MAX_WHEEL_SPEED` | 3.6 m/s | 四轮速度等比限幅 |
| `CHASSIS_TURN_MIN_RPM` | 120 | **纯转向最小目标转速**：转向指令时目标转速低于 120 rpm 会被强制抬升（v3.0 已移除，改用平滑破静摩擦） |
| `WHEEL_R` | 0.025 m | 轮半径（运动学 + 里程计共用） |
| `CHASSIS_WHEEL_TRACK` | 0.081 m | 左右轮距（差速 yaw 计算，**务必实测**） |
| `CHASSIS_MOTOR_*_FORWARD_SIGN` | ±1 | 电机接线方向；改接线或改这里，不能两者都改 |
| `CHASSIS_CONTROL_TIME_MS` | 2 ms | 底盘任务周期 |
| `CHASSIS_TORQUE_CONTROL_PERIOD_MS` | 5 ms | 实际电流控制/发布周期 |

## 4. 里程计参数（`Components/Algorithm/Src/odometry.c`，`odom_default_config`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `wheel_radius_m` | 0.025 | 轮半径，直行距离精度主要来源 |
| `wheel_track_m` | 0.081 | 轮距，差速 yaw 计算 |
| `imu_yaw_turn_weight` | **0.75** | 转弯时 yaw = IMU 75% + 编码器 25%（v3.0 已改为纯 IMU 1.0） |
| `imu_yaw_straight_weight` | 0.20 | 直行时 IMU yaw 权重 |
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
| 角度索引 | 主循环 | 本分支仍为传感器系映射：`pit_angle=Euler[2]`、`rol_angle=Euler[1]`（对应板载 IMU 绕 Z 逆时针 90° 的安装；v3.0 已在源头统一转换为车体系） |
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

> 注意：命令名沿用了 kp/ki/kd，但实际映射到的是位置增益、速度阻尼、摩擦前馈，不是真正的 PID 三项。在线调参只改 DRIVE 组，且 `friction_current` 会被钳位到 `drive.current_limit_a`（0.5 A），掉电丢失。

## 8. 常用 VOFA+ 调试帧

本分支没有 v3.0 的 20 通道管线帧，常用调试帧：

| 函数 | 通道 | 内容 |
| --- | --- | --- |
| `Vofa_Send_Odom_Debug_Info` | 12 | 0 distance，1 odom_yaw(deg)，2 INS_yaw(deg)，3 gyro_z(deg/s)，4-7 四轮连续角度(deg)，8 valid，9 motion_mode，10-11 前后轴距离 |
| `Vofa_Send_Speed_Control_Info` | 12 | 0-3 目标转速 rpm，4-7 反馈转速 rpm，8-11 电流指令 A |
| `Vofa_Send_Brake_Debug_Info` | 12 | 0-3 原始电流 A，4-7 轮速 rpm，8-11 限幅后电流 A |

## 9. 实车调参顺序建议

1. **先修机械**：导轮螺丝紧固、整车无松旷，否则数据不可信。
2. 平地低速直行：确认四轮反馈转速一致、无抖动；`friction_current_a` 负责稳态静差，`speed_kd` 负责超调。
3. 原地旋转：确认 odom yaw 方向与比例（本分支转弯 IMU 权重 0.75，抖动时 yaw 会被低估）。
4. 驻坡测试：本分支 HOLD 只锁中值位置且无重力前馈，坡上驻车不完整属已知问题，请使用 `codex/v3.0` 分支。
5. 分段清零：上坡/过路口后发 `0x33` 帧，等待 `segment_id` 确认，再开始新段累计。
