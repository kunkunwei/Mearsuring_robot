# 管道测量机器人底盘控制（STM32F407 / FreeRTOS）

四轮独立驱动（4WD）差速转向底盘控制工程，用于狭窄方形管道内的距离测量。底盘负责稳定行驶、里程计估计、超声波测距回传，并与树莓派上位机通过串口通信。

> 当前分支为 `codex/v3.0`（有界 MIT 电流控制 + 59 字节通信帧 + INS 车体系角度 + 转弯纯 IMU yaw）。该版本仍在实车测试阶段，**不要合入 `main`**。

## 硬件与通信

- 主控：RoboMaster C 开发板，STM32F407，FreeRTOS。
- 底盘：四轮独立驱动，差速转向；M3508 直驱轮（减速箱已拆除）。
- 电调：VESC（兼容 N630），CAN1 @ 500 kbit/s。
- IMU：板载 BMI088（陀螺/加速度）+ IST8310（磁力计，**未标定**）。
- 超声波：左右各一个 SR09，I2C 读取，测距随里程计帧回传。
- 上位机：树莓派。当前通过 **USART6（115200 8N1，经 CP2102N USB-UART 转接）** 通信；`MiniPC_SendChassisOdomUSB()` 的 USB CDC 打包接口保留，后续可直接切换。
- 调试：USART6 同口可用于 VOFA+ 调试输出（通过 User_Task 开关）。

## 控制模式（遥控器状态机）

| 拨杆 | 模式 | 说明 |
| --- | --- | --- |
| 右拨杆下 | 无力模式 | 底盘不输出电流 |
| 右拨杆中 | 手动模式 | 摇杆控制前进/后退/转向 |
| 右拨杆上 | ROS 控制 | 接收上位机 `vx`/`wz` 指令，500 ms 超时归零 |
| 左拨杆上 | 驻坡/锁定（HOLD_TEST） | 优先级最高，锁定当前轮位置并加重力前馈 |
| 左拨杆下沿 | 清零里程计 | 切换瞬间触发 `chassis_odom_reset()` |

控制量约定：`vx` 单位 m/s，车体前进为正；`wz` 单位 rad/s，逆时针为正。

## 通信协议速览

详见 [docs/chassis_protocol.md](docs/chassis_protocol.md)：

| 帧 | 方向 | 地址 | 长度 | 内容 |
| --- | --- | --- | --- | --- |
| 速度指令 | 上位机 → 底盘 | `0x31` | 12 | `vx`、`wz` |
| 分段清零 | 上位机 → 底盘 | `0x33` | 6 | `segment_id`，等待确认后可重复发送 |
| 里程计反馈 | 底盘 → 上位机 | `0x32` | **59** | `x/y/yaw/distance/vx/wz/pitch/roll/四轮连续角度/左右超声波/segment_id`，100 Hz |

> 注意：上位机解析必须按 59 字节（长度字段 `0x3B`）实现，旧版 51 字节解析会丢帧。pitch 为修正后车体俯仰角（平地≈0，上坡为负），可直接用于斜坡判断。

## 主要控制模块

### RTOS 任务（`Core/Src/freertos.c`）

| 任务 | 周期 | 优先级 | 职责 |
| --- | --- | --- | --- |
| `INS_Task` | 1 ms | Realtime | BMI088 采集、四元数 EKF、**传感器系→车体系角度转换**、IMU 温度控制 |
| `Can_Task` | 5 ms | High | VESC 电流指令发送、电机状态解析 |
| `Ultrasonic_Task` | — | High | SR09 超声波 I2C 轮询 |
| `Chassis_Task` | 2 ms | AboveNormal | 模式状态机、运动学分解、控制状态机、5 ms 扭矩发布 |
| `Ros_Task` | 10 ms | Normal | 组装并发送 59 字节里程计帧（USART6） |
| `ObserveTask` | 5 ms | Idle | 卡尔曼速度估计、里程计融合、分段清零、磁航向修正 |
| `User_Task` | 30 ms | Idle | 蜂鸣器/指示灯、可选 VOFA 调试输出 |

### 控制器（`Components/Controller/`）

- `chassis_control_manager.c`：**核心状态机**，DISABLED → DRIVE → BRAKE → HOLD → FAULT，统一限幅与故障保护。
- `chassis_mit_ctrl.c`：DRIVE 模式下的有界 MIT 电流控制（位置跟踪 + 速度阻尼 + 积分 + 摩擦前馈）。
- `chassis_hold_ctrl.c`：HOLD/BRAKE 的位置保持、速度阻尼与 pitch 重力前馈。
- `chassis_brake.c`：制动阻尼与限幅。

### 算法 / 设备 / BSP

- `Components/Algorithm/odometry.c`：四轮编码器 + IMU yaw 融合里程计（转弯纯 IMU）。
- `Components/Algorithm/magnetic_heading.c`：磁航向估计（未标定，修正速率受限）。
- `Components/Device/minipc.c`：协议解析/打包。
- `Components/Device/mymotor.c`：VESC 状态解析与电流指令。
- `Bsp/bsp_can.c` / `Bsp/vofa.c`：CAN 收发、VOFA+ 调试与在线调参。

## 调参入口速查

完整说明见 [docs/control_tuning.md](docs/control_tuning.md)：

- 控制器默认参数：`Chassis_ControlManager_DefaultConfig()`（`Components/Controller/Src/chassis_control_manager.c`）。
- 底盘限速/运动学参数：`Application/Tasks/Inc/Chassis_Task.h`。
- 里程计参数：`OdomConfig_t`（`Components/Algorithm/Src/odometry.c`）。
- INS 滤波/EKF 噪声/安装映射：`Application/Tasks/Src/INS_Task.c`。
- 观测任务卡尔曼与磁力计：`Application/Tasks/Src/observe_task.c`。
- VOFA+ 在线调参：发送 `KP=xx` / `KI=xx` / `KD=xx`，映射到位置增益/速度阻尼/摩擦前馈（注意与命名不同，见调参文档）。

## VESC / CAN 配置建议

- CAN 波特率：500 kbit/s。
- Status Rate 1：500 Hz，勾选 Status 1（RPM、电流、占空比）。
- Status Rate 2：建议 100~250 Hz，勾选 Status 4（PID-position Now，用于连续角度）。
- Status 5 当前不作为里程计输入。

## 构建说明

CLion + STM32CubeCLT/CMake。核心工程文件：`CMakeLists.txt`、`Mearsuring_robot.ioc`、`STM32F407IGHX_FLASH.ld`。

若用 STM32CubeMX 重新生成工程，需检查 `Core/Src/can.c` 的 CAN1 波特率仍为 500 kbit/s，并确认 `CMakeLists.txt` 未被覆盖。

## v3.0 已知问题与验证清单

1. **纯转向四轮响应不一致**：vofa55 实测 M2 目标 74 rpm 反馈仅约 29 rpm 且电流顶限幅，M1 超速至 123 rpm。导轮缺螺丝、整车松动会直接放大此问题，先紧固机械再调参。
2. **转弯 odom yaw**：已改为纯 IMU（`imu_yaw_turn_weight = 1.0`），需实车逆时针旋转验证符号与比例。
3. **INS 安装映射**：IMU 相对车体绕 Z 逆时针 90°，`INS_Info.angle[]` 已转成车体系（pitch=+Euler[2]，roll=−Euler[1]）。`pitch_zero_offset`（3.7°）是在旧轴上标定的，**换轴后必须在平地上重新标定**。
4. **磁力计未标定**：磁航向修正仍开启（最大 2°/s），若 yaw 有慢漂建议标定或关闭该修正。
5. **上位机协议**：PC 侧解析必须同步升级到 59 字节。

## 备注

仓库不提交本地构建产物、VOFA/CAN 测试 CSV、PPT 输出和 IDE 缓存文件。上位机代码（`PC/`）单独管理，不随本固件仓库提交。
