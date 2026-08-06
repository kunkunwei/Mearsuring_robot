# 管道测量机器人底盘控制（STM32F407 / FreeRTOS）

四轮独立驱动（4WD）差速转向底盘控制工程，用于狭窄方形管道内的距离测量。底盘负责稳定行驶、里程计估计、超声波测距回传，并与树莓派上位机通过串口通信。

> 当前分支为 `codex/safe-mit-control`：有界 MIT 电流控制基础版，仍在实车测试阶段，**不要合入 `main`**。后续增强（转向破静摩擦、每轮驻坡位置闭环、59 字节通信帧、INS 车体系角度、转弯纯 IMU yaw）在 `codex/v3.0` 分支。

## 硬件与通信

- 主控：RoboMaster C 开发板，STM32F407，FreeRTOS。
- 底盘：四轮独立驱动，差速转向；M3508 直驱轮（减速箱已拆除）。
- 电调：VESC（兼容 N630），CAN1 @ 500 kbit/s。
- IMU：板载 BMI088（陀螺/加速度）+ IST8310（磁力计，**未标定**）。
- 超声波：左右各一个 SR09，I2C 读取，测距随里程计帧回传。
- 上位机：树莓派。当前通过 **USART6（115200 8N1，经 CP2102N USB-UART 转接）** 通信；`MiniPC_SendChassisOdomUSB()` 的 USB CDC 打包接口保留。
- 调试：USART6 同口可用于 VOFA+ 调试输出（通过 User_Task 开关）。

## 控制模式（遥控器状态机）

| 拨杆 | 模式 | 说明 |
| --- | --- | --- |
| 右拨杆下 | 无力模式 | 底盘不输出电流 |
| 右拨杆中 | 手动模式 | 摇杆控制前进/后退/转向 |
| 右拨杆上 | ROS 控制 | 接收上位机 `vx`/`wz` 指令，500 ms 超时归零 |
| 左拨杆上 | 驻坡/锁定（HOLD_TEST） | 优先级最高，锁定四轮中值位置 |
| 左拨杆下沿 | 清零里程计 | 切换瞬间触发 `chassis_odom_reset()` |

控制量约定：`vx` 单位 m/s，车体前进为正；`wz` 单位 rad/s，逆时针为正。

## 通信协议速览

详见 [docs/chassis_protocol.md](docs/chassis_protocol.md)：

| 帧 | 方向 | 地址 | 长度 | 内容 |
| --- | --- | --- | --- | --- |
| 速度指令 | 上位机 → 底盘 | `0x31` | 12 | `vx`、`wz` |
| 分段清零 | 上位机 → 底盘 | `0x33` | 6 | `segment_id`，等待确认后可重复发送 |
| 里程计反馈 | 底盘 → 上位机 | `0x32` | **51** | `x/y/yaw/distance/vx/wz/四轮连续角度/左右超声波/segment_id`，100 Hz |

> 该分支里程计帧为 51 字节（无 pitch/roll）。`codex/v3.0` 已扩展为 59 字节并新增 pitch/roll，上位机对接不同固件版本时需按帧长度字段自适应。

## 主要控制模块

### RTOS 任务（`Core/Src/freertos.c`）

| 任务 | 周期 | 优先级 | 职责 |
| --- | --- | --- | --- |
| `INS_Task` | 1 ms | Realtime | BMI088 采集、四元数 EKF、传感器系欧拉角、IMU 温度控制 |
| `Can_Task` | 5 ms | High | VESC 电流指令发送、电机状态解析 |
| `Ultrasonic_Task` | — | High | SR09 超声波 I2C 轮询 |
| `Chassis_Task` | 2 ms | AboveNormal | 模式状态机、运动学分解、控制状态机、5 ms 扭矩发布 |
| `Ros_Task` | 10 ms | Normal | 组装并发送 51 字节里程计帧（USART6） |
| `ObserveTask` | 5 ms | Idle | 卡尔曼速度估计、里程计融合、分段清零、磁航向修正 |
| `User_Task` | 30 ms | Idle | 蜂鸣器/指示灯、可选 VOFA 调试输出 |

### 控制器（`Components/Controller/`）

- `chassis_control_manager.c`：**核心状态机**，DISABLED → DRIVE → BRAKE → HOLD → FAULT，统一限幅与故障保护。
- `chassis_mit_ctrl.c`：DRIVE 模式下的有界 MIT 电流控制（位置跟踪 + 速度阻尼 + 摩擦前馈）。
- `chassis_hold_ctrl.c`：HOLD/BRAKE 的**四轮中值位置**保持与速度阻尼（默认无重力前馈）。
- `chassis_brake.c`：制动阻尼与限幅。

本分支与 `codex/v3.0` 的主要差异：

- HOLD 是四轮中值位置闭环，不是每轮独立位置闭环（单轮转动会被中值算法忽略）。
- 无转向破静摩擦补偿（`turn_breakaway_*` 不存在）；纯转向目标转速会被 `CHASSIS_TURN_MIN_RPM`（120 rpm）强制抬升。
- HOLD 重力前馈默认关闭（`pitch_feedforward_a = 0`），无 `pitch_zero_offset`。
- 电流斜率限幅为单一 `current_slew_a_per_s`（10 A/s），且 DRIVE/BRAKE/HOLD 电流上限均为 0.5 A。
- 转弯 odom yaw 权重 0.75（IMU 75% + 编码器 25%）。

### 算法 / 设备 / BSP

- `Components/Algorithm/odometry.c`：四轮编码器 + IMU yaw 融合里程计。
- `Components/Algorithm/magnetic_heading.c`：磁航向估计（未标定，修正速率受限）。
- `Components/Device/minipc.c`：协议解析/打包。
- `Components/Device/mymotor.c`：VESC 状态解析与电流指令。
- `Bsp/bsp_can.c` / `Bsp/vofa.c`：CAN 收发、VOFA+ 调试与在线调参。

## 调参入口速查

完整说明见 [docs/control_tuning.md](docs/control_tuning.md)：

- 控制器默认参数：`Chassis_ControlManager_DefaultConfig()`（`Components/Controller/Src/chassis_control_manager.c`）。
- 底盘限速/运动学参数：`Application/Tasks/Inc/Chassis_Task.h`（含 `CHASSIS_TURN_MIN_RPM`）。
- 里程计参数：`OdomConfig_t`（`Components/Algorithm/Src/odometry.c`）。
- INS 滤波/EKF 噪声/安装映射：`Application/Tasks/Src/INS_Task.c`。
- 观测任务卡尔曼与磁力计：`Application/Tasks/Src/observe_task.c`。
- VOFA+ 在线调参：发送 `KP=xx` / `KI=xx` / `KD=xx`（映射说明见调参文档）。

## VESC / CAN 配置建议

- CAN 波特率：500 kbit/s。
- Status Rate 1：500 Hz，勾选 Status 1（RPM、电流、占空比）。
- Status Rate 2：建议 100~250 Hz，勾选 Status 4（PID-position Now，用于连续角度）。
- Status 5 当前不作为里程计输入。

## 构建说明

CLion + STM32CubeCLT/CMake。核心工程文件：`CMakeLists.txt`、`Mearsuring_robot.ioc`、`STM32F407IGHX_FLASH.ld`。

若用 STM32CubeMX 重新生成工程，需检查 `Core/Src/can.c` 的 CAN1 波特率仍为 500 kbit/s，并确认 `CMakeLists.txt` 未被覆盖。

## 已知问题

1. **纯转向四轮响应不一致**：vofa55 实测 M2 目标 74 rpm 反馈仅约 29 rpm 且电流顶限幅，M1 超速至 123 rpm。导轮缺螺丝、整车松动会直接放大此问题，先紧固机械再调参。
2. **HOLD 中单轮空转**：本分支 HOLD 只锁四轮中值位置，单轮转动会被忽略，且无重力前馈，坡上驻车不完整。
3. **odom yaw 误差**：转弯时 IMU 权重 0.75 且混入编码器，旋转抖动时 yaw 会明显低估（110° 实测仅回报约 16° 的场景即出自该分支阶段）。
4. **电流上限偏小**：DRIVE/BRAKE/HOLD 均为 0.5 A，破静摩擦不足，实测需要更大电流时请按调参文档上调。

以上问题已在 `codex/v3.0` 分支针对性重构，本分支仅保留作为 MIT 控制基础版本。

## 备注

仓库不提交本地构建产物、VOFA/CAN 测试 CSV、PPT 输出和 IDE 缓存文件。上位机代码（`PC/`）单独管理，不随本固件仓库提交。
