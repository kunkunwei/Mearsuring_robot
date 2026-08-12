# 底盘通信协议（v3.0）

本文档描述 STM32F407 底盘与树莓派上位机之间的二进制通信协议，与 `codex/v3.0` 固件一致。

## 传输方式

- 当前：**USART6，115200 8N1，无流控**，经 CP2102N USB-UART 转接接入树莓派。
- 保留：USB CDC 虚拟串口打包接口（`MiniPC_SendChassisOdomUSB`），帧格式完全相同，后续可直接切换。

## 通用规则

- 字节序：小端（little-endian）。
- 浮点：IEEE-754 `float32`。
- 帧头：`0x42`。
- 校验和：**帧内除最后一个字节外所有字节的 8 位累加和**，存放在最后一字节。
- 无效帧直接丢弃，不影响后续解析。

```c
uint8_t checksum = 0;
for (uint8_t i = 0; i < frame_len - 1; i++) {
    checksum += frame[i];
}
```

## 坐标系约定

- 车体坐标系：X 向前、Y 向左、Z 向上。
- `vx`：m/s，前进为正。
- `wz` / `yaw`：rad/s / rad，逆时针为正。
- `pitch`：rad，车头抬起（上坡）为负；帧内为**修正后**值（平地≈0）。
- `roll`：rad，原始值。

## 帧 1：速度指令（上位机 → 底盘）

地址 `0x31`，长度 12 字节。

| 偏移 | 长度 | 类型 | 内容 |
|---:|---:|---|---|
| 0 | 1 | uint8 | 帧头 `0x42` |
| 1 | 1 | uint8 | 地址 `0x31` |
| 2 | 1 | uint8 | 长度 `12` |
| 3 | 4 | float32 | `vx`，m/s |
| 7 | 4 | float32 | `wz`，rad/s |
| 11 | 1 | uint8 | 校验和（字节 0..10 累加） |

底盘侧 500 ms 未收到有效指令则强制 `vx=0`、`wz=0`。

## 帧 2：分段清零（上位机 → 底盘）

地址 `0x33`，长度 6 字节。

| 偏移 | 长度 | 类型 | 内容 |
|---:|---:|---|---|
| 0 | 1 | uint8 | 帧头 `0x42` |
| 1 | 1 | uint8 | 地址 `0x33` |
| 2 | 1 | uint8 | 长度 `6` |
| 3 | 1 | uint8 | 命令 `0x01` |
| 4 | 1 | uint8 | `segment_id` |
| 5 | 1 | uint8 | 校验和 |

### 分段清零流程（应对上坡打滑、过路口拐弯的累计误差）

1. 导航记录当前段的最终 `x`、`y`、`distance`。
2. 导航生成新的 `segment_id`，发送 `0x33` 清零帧。
3. 在 `0x32` 里程计反馈帧中等待 `segment_id` 等于新 ID。
4. 确认后，将反馈中的 `x`、`y`、`distance` 作为新段起点，导航自行累计各段里程。
5. **确认前可重复发送同一个 ID**，底盘对同一个新 ID 只应用一次，不会重复清零。

清零只重置位置/距离累计和轮位置原点，INS yaw、磁航向和航向滤波器保持连续。

## 帧 3：里程计反馈（底盘 → 上位机）

地址 `0x32`，长度 **59 字节（0x3B）**，发送周期 10 ms（100 Hz）。

| 偏移 | 长度 | 类型 | 内容 |
|---:|---:|---|---|
| 0 | 1 | uint8 | 帧头 `0x42` |
| 1 | 1 | uint8 | 地址 `0x32` |
| 2 | 1 | uint8 | 长度 `59` |
| 3 | 4 | float32 | `x`，m |
| 7 | 4 | float32 | `y`，m |
| 11 | 4 | float32 | `yaw`，rad |
| 15 | 4 | float32 | `distance`，有符号前进距离，m |
| 19 | 4 | float32 | `vx`，估计前进速度，m/s |
| 23 | 4 | float32 | `wz`，估计 yaw 角速度，rad/s |
| 27 | 4 | float32 | `pitch`，修正后俯仰角，rad（平地≈0，上坡为负） |
| 31 | 4 | float32 | `roll`，横滚角，rad |
| 35 | 4 | float32 | 电机 1 连续角度，deg |
| 39 | 4 | float32 | 电机 2 连续角度，deg |
| 43 | 4 | float32 | 电机 3 连续角度，deg |
| 47 | 4 | float32 | 电机 4 连续角度，deg |
| 51 | 2 | uint16 | 左超声波距离，mm |
| 53 | 2 | uint16 | 右超声波距离，mm |
| 55 | 1 | uint8 | 左超声波在线状态 |
| 56 | 1 | uint8 | 右超声波在线状态 |
| 57 | 1 | uint8 | 已执行的 `segment_id` |
| 58 | 1 | uint8 | 校验和（字节 0..57 累加） |

### pitch 的斜坡判断用法

`pitch` 在 STM32 侧使用与底盘斜坡补偿**同一个零偏**（`hold.pitch_zero_offset_rad`），因此：

- 平地读数 ≈ 0 rad；
- 上坡（车头抬起）为负，与控制器重力前馈符号一致；
- 导航可用 `|pitch| > 5°` 判定需要位置补偿的大坡，`|pitch| < 3°` 判定平地。

> 注意：v3.0 将 INS 角度统一转换为车体系后，该零偏需要在平地上重新标定（见调参文档）。

## Python 参考实现（59 字节）

```python
import struct

HEADER = 0x42
ADDR_CMD = 0x31
ADDR_ODOM = 0x32
ADDR_RESET = 0x33
LEN_CMD = 12
LEN_ODOM = 59
LEN_RESET = 6
ODOM_FORMAT = "<BBBffffffffffffHHBBBB"

def checksum_u8(data: bytes) -> int:
    return sum(data) & 0xFF

def pack_command(vx: float, wz: float) -> bytes:
    frame = struct.pack("<BBBff", HEADER, ADDR_CMD, LEN_CMD, vx, wz)
    return frame + bytes([checksum_u8(frame)])

def pack_odom_reset(segment_id: int) -> bytes:
    frame = bytes((HEADER, ADDR_RESET, LEN_RESET, 0x01, segment_id))
    return frame + bytes([checksum_u8(frame)])

def unpack_odom(frame: bytes):
    if len(frame) != LEN_ODOM:
        raise ValueError("bad odom frame length")
    if frame[0] != HEADER or frame[1] != ADDR_ODOM or frame[2] != LEN_ODOM:
        raise ValueError("bad odom frame header")
    if checksum_u8(frame[:-1]) != frame[-1]:
        raise ValueError("bad odom checksum")
    v = struct.unpack(ODOM_FORMAT, frame)
    return {
        "x": v[3], "y": v[4], "yaw": v[5], "distance": v[6],
        "vx": v[7], "wz": v[8], "pitch_rad": v[9], "roll_rad": v[10],
        "motor_pos_deg": v[11:15],
        "us_left_mm": v[15], "us_right_mm": v[16],
        "us_left_online": v[17], "us_right_online": v[18],
        "segment_id": v[19],
    }
```

## ROS 映射建议

- 订阅 `/cmd_vel`：`linear.x -> vx`，`angular.z -> wz`。
- 发布 `/odom`：`x/y/yaw` → 位姿，`vx/wz` → 速度，`yaw` → 四元数。
- 发布倾斜话题（如 `imu_tilt`）：`pitch_rad` 为减去上电零点后的实车纵向俯仰角，车头抬高为正；`roll_rad` 为实车横滚角。
- 分段清零：发布 `odom_reset_segment`（UInt8），等待 `odom_segment_id` 与请求一致。
