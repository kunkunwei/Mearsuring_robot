# new_hero

这是我重新搓的老麦轮英雄机器人代码，面向 STM32F407 + FreeRTOS，包含云台、底盘与自瞄接口。

## 1. 输入控制方式切换与硬件支持

### 控制源与编译开关
- 云台控制源由 `Application/Tasks/Inc/Gimbal_task.h` 中的宏控制。
  - `USE_PC_CONTROL`：图传链路遥控器（VT03）控制（当前已启用）。
  - `USE_PC_RM_CONTROL`：图传链路电脑选手端控制（当前已启用）。
  - `USE_RC_CONTROL`：传统遥控器控制（注释即禁用）。
- 若使用 SBUS 协议遥控器，启用 `USE_SBUS_PROTOCOL` 并按对应通道定义。

### 硬件与通道约定（核心）
- VT03 图传遥控器：
  - `rc.mode_sw`：模式档位（0/1/2）。
  - `rc.pause`：暂停/无力切换键。
  - `rc.fn_1`：小陀螺模式切换。
- 传统遥控器（DT7 或 SBUS）：
  - `RC_RIGHT_X_CH`、`RC_RIGHT_Y_CH`：云台 YAW/PITCH 通道。
  - `LEFT_SWITCH`/`RIGHT_SWITCH`（或 SBUS 四档开关）：模式切换。

### 底盘输入
- 底盘同时支持 VT03 与传统遥控器输入，统一由 `Application/Tasks/Src/Chassis_Task.c` 的 `chassis_set_mode()` 解析并切换状态。

## 2. 云台与底盘的关联控制

- 底盘可选择“跟随云台”或“云台跟随底盘”的耦合方式：
  - `CHASSIS_CHASSIS_FOLLOW_GIMBAL_YAW`：底盘朝向跟随云台 YAW。
  - `CHASSIS_GIMBAL_FOLLOW_CHASSIS`：云台保持与底盘对齐。
  - `CHASSIS_VECTOR_NO_FOLLOW_YAW`：底盘向量控制，方向不跟随云台。
- 云台端通过 `local_chassis_ref = get_chassis_ref_point()` 获取底盘状态，用于耦合控制与模式切换处理。

## 3. 云台与底盘状态机

### 云台状态机（`gimbal_mod_e`）
- `GIMBAL_MOD_NO_FORCE`：无力模式。
- `GIMBAL_MOD_SLOW_CALI`：慢速校准。
- `GIMBAL_MOD_NORMAL`：正常模式（底盘跟随云台）。
- `GIMBAL_MOD_AutoAim`：自动瞄准模式。
- `GIMBAL_MOD_FLOW_CHASSIS`：云台跟随底盘。

状态切换逻辑：
- 由 VT03 `pause` + `mode_sw` 组合控制，细节见 `Application/Tasks/Src/Gimbal_task.c` 的 `gimbal_set_mod()`。
- 进入/退出自瞄与小陀螺模式时，会在 `gimbal_mod_change_date_transfer()` 中重置目标位置，避免突跳。

### 底盘状态机（`chassis_mode_e`）
- `CHASSIS_FORCE_RAW`：开环/无力。
- `CHASSIS_CHASSIS_FOLLOW_GIMBAL_YAW`：底盘跟随云台。
- `CHASSIS_GIMBAL_FOLLOW_CHASSIS`：云台跟随底盘。
- `CHASSIS_VECTOR_NO_FOLLOW_YAW`：向量控制但不跟随云台。

状态切换逻辑：
- 由 VT03 `pause` 与 `mode_sw` 组合控制，见 `Application/Tasks/Src/Chassis_Task.c` 的 `chassis_set_mode()`。

## 4. 自瞄接口

- USB 小电脑数据入口：`getUsbMiniPcPtr()`，结构体类型为 `Usb_AutoAim_t`（见 `Application/Tasks/Src/Gimbal_task.c`）。
- 自瞄控制主逻辑：`minipc_control()`，直接使用 `minipc_target_yaw` 与 `minipc_target_pitch` 更新云台目标。
- ROS/上位机反馈：`Application/Tasks/Src/Ros_Task.c` 中 `MiniPC_Sendgimbal()` 周期发送云台姿态。
- 轨迹相关数据结构：`Application/API/Inc/api_trajectory.h`。

## 5. 重新生成项目的注意事项

1. 先在本地构建项目（确保当前工程可正常编译）。
2. 再使用 STM32CubeMX 重新生成工程。
3. CubeMX 生成后，恢复 `CMakeLists.txt` 为我维护的版本：
   - 方式 A：将 `CMakeLists_template.txt` 的内容回拷到 `CMakeLists.txt`。
   - 方式 B：若使用版本管理工具，直接回滚 `CMakeLists.txt` 到我写好的版本。

---

### 关键文件索引
- 云台任务：`Application/Tasks/Src/Gimbal_task.c`
- 底盘任务：`Application/Tasks/Src/Chassis_Task.c`
- 云台头文件：`Application/Tasks/Inc/Gimbal_task.h`
- 底盘头文件：`Application/Tasks/Inc/Chassis_Task.h`
- 自瞄接口：`Application/API/Inc/api_trajectory.h`
- CMake 配置：`CMakeLists.txt` / `CMakeLists_template.txt`

