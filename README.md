# Mearsuring Robot

管道复尺测绘机器人底盘控制工程。项目基于 RoboMaster C 型开发板 STM32F407 和 FreeRTOS，实现四轮独立驱动差速底盘控制、N630/VESC 电调 CAN 通信、编码器/IMU 里程计估计、USB 上位机通信和调试数据输出。

## 项目目标

机器人用于狭窄、平整、可能无光照的方形管道内距离测量。底盘需要稳定沿管道中心行驶，并满足 1 m 行驶距离误差不超过 1 cm 的里程计精度目标。后续树莓派上位机会结合雷达数据进行中心循迹和路径规划。

## 硬件与通信

- 主控：RoboMaster C 板，STM32F407。
- 实时系统：FreeRTOS。
- 底盘：四轮独立驱动，差速转向。
- 电机：M3508 直驱轮，减速箱已拆除。
- 电调：N630/VESC，使用 CAN1 通信。
- 上位机：树莓派，当前通过 USB CDC 通信，后续可切换 USART6。
- 调试：USART6 当前用于 VOFA+ 调试输出。
- IMU：用于 yaw 和角速度辅助里程计与状态判断。

## 控制模式

遥控器状态机如下：

- 右拨杆下拨：无力模式，底盘不输出力矩。
- 右拨杆中位：遥控手动模式，遥控器摇杆控制前进、后退和转向。
- 右拨杆上拨：ROS 控制模式，底盘接收树莓派通过 USB 发送的 `vx`、`wz` 指令。
- 左拨杆上拨：原地固定模式，优先级最高，用于锁定当前位置。

上位机控制量单位：

- `vx`：m/s，车体前进方向为正。
- `wz`：rad/s，逆时针旋转为正。

## MiniPC 通信协议

上位机到底盘控制帧长度为 12 字节：

| 字节 | 内容 |
| --- | --- |
| 0 | 帧头 `0x42` |
| 1 | 地址 `0x31` |
| 2 | 帧长 `12` |
| 3-6 | `float vx`，小端 |
| 7-10 | `float wz`，小端 |
| 11 | checksum，前 11 字节累加 |

底盘到上位机里程计帧长度为 36 字节，包含 `x`、`y`、`yaw`、`distance`、`vx`、`wz` 和四个电机编码器值。

## 主要模块

- `Application/Tasks/Src/Chassis_Task.c`：底盘任务、模式切换、运动学分解和控制器调用。
- `Application/Tasks/Src/observe_task.c`：里程计与状态估计任务。
- `Application/Tasks/Src/Ros_Task.c`：USB/上位机里程计回传。
- `Components/Controller/Src/chassis_mit_ctrl.c`：MIT 风格电流控制。
- `Components/Controller/Src/chassis_brake.c`：停车制动、制动限幅和简化 ABS 防抱死逻辑。
- `Components/Algorithm/Src/odometry.c`：四轮编码器、IMU yaw 和运动指令融合的里程计估计。
- `Components/Device/Src/mymotor.c`：电机反馈数据结构与 VESC 状态解析。
- `Components/Device/Src/minipc.c`：上位机控制帧解析与里程计帧打包。
- `Bsp/Src/bsp_can.c`：CAN 底层收发。
- `Bsp/Src/vofa.c`：VOFA+ 调试数据发送和 PID 调参命令解析。

## VESC/CAN 配置建议

- CAN 波特率：500 kbit/s。
- Status Rate 1：500 Hz，只勾选 Status 1，用于 RPM、电流、占空比。
- Status Rate 2：建议 100-250 Hz，只勾选 Status 4，用于 PID-position Now。
- Status 5 当前不作为主里程计输入。

## 构建说明

工程使用 CLion + STM32CubeCLT/CMake 构建，核心工程文件包括：

- `CMakeLists.txt`
- `CMakeLists_template.txt`
- `Mearsuring_robot.ioc`
- `STM32F407IGHX_FLASH.ld`
- `STM32F407IGHX_RAM.ld`

如果使用 STM32CubeMX 重新生成工程，需要检查 `Core/Src/can.c` 中 CAN1 是否仍为 500 kbit/s，并确认 `CMakeLists.txt` 没有被覆盖为错误配置。

## 备注

仓库不提交本地构建产物、VOFA/CAN 测试 CSV、PPT 输出和 IDE 缓存文件。调试数据请保留在本地，不进入版本库。
