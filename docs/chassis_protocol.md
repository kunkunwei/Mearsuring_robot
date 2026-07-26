# Chassis UART/USB Protocol

This document describes the binary protocol between the STM32F407 chassis controller and the Raspberry Pi/ROS host.

The transport is currently USB CDC. Later it will move to USART6 UART. The frame format is unchanged.

## Common Rules

- Byte order: little-endian
- Float format: IEEE-754 `float32`
- Header: `0x42`
- Checksum: unsigned 8-bit sum of all previous bytes in the frame
- Invalid frames are ignored

Checksum example:

```c
uint8_t checksum = 0;
for (int i = 0; i < frame_len - 1; i++) {
    checksum += frame[i];
}
```

## Host To STM32: Velocity Command

The ROS host sends desired chassis velocity in the robot local frame.

Coordinate convention:

- `+vx`: forward
- `+wz`: counter-clockwise yaw
- Unit of `vx`: m/s
- Unit of `wz`: rad/s

Frame length: `12` bytes

| Offset | Size | Type    | Name     | Description |
|--------|------|---------|----------|-------------|
| 0      | 1    | uint8   | header   | `0x42` |
| 1      | 1    | uint8   | address  | `0x31` |
| 2      | 1    | uint8   | length   | `12` |
| 3      | 4    | float32 | vx       | desired forward speed |
| 7      | 4    | float32 | wz       | desired yaw rate |
| 11     | 1    | uint8   | checksum | sum of bytes 0..10 |

The STM32 command timeout is `500 ms`. If no valid command is received within this window, `vx` and `wz` are forced to zero.

## STM32 To Host: Odometry Feedback

The STM32 sends chassis odometry and measured speed.

Frame length: `36` bytes

| Offset | Size | Type    | Name     | Description |
|--------|------|---------|----------|-------------|
| 0      | 1    | uint8   | header   | `0x42` |
| 1      | 1    | uint8   | address  | `0x32` |
| 2      | 1    | uint8   | length   | `36` |
| 3      | 4    | float32 | x        | odom X, m |
| 7      | 4    | float32 | y        | odom Y, m |
| 11     | 4    | float32 | yaw      | heading, rad |
| 15     | 4    | float32 | distance | signed forward distance, m |
| 19     | 4    | float32 | vx       | estimated forward speed, m/s |
| 23     | 4    | float32 | wz       | measured yaw rate, rad/s |
| 27     | 2    | uint16  | ecd_0    | motor 0 encoder raw value, 0..8191 |
| 29     | 2    | uint16  | ecd_1    | motor 1 encoder raw value, 0..8191 |
| 31     | 2    | uint16  | ecd_2    | motor 2 encoder raw value, 0..8191 |
| 33     | 2    | uint16  | ecd_3    | motor 3 encoder raw value, 0..8191 |
| 35     | 1    | uint8   | checksum | sum of bytes 0..34 |

Current STM32 send period: `10 ms` / `100 Hz`.

## Python Reference

### Pack Velocity Command

```python
import struct

HEADER = 0x42
ADDR_CHASSIS_CMD = 0x31
LEN_CHASSIS_CMD = 12

def checksum_u8(data: bytes) -> int:
    return sum(data) & 0xFF

def pack_cmd(vx: float, wz: float) -> bytes:
    frame = struct.pack("<BBBff", HEADER, ADDR_CHASSIS_CMD, LEN_CHASSIS_CMD, vx, wz)
    return frame + bytes([checksum_u8(frame)])
```

### Decode Odometry Feedback

```python
import struct

HEADER = 0x42
ADDR_CHASSIS_ODOM = 0x32
LEN_CHASSIS_ODOM = 36

def checksum_u8(data: bytes) -> int:
    return sum(data) & 0xFF

def unpack_odom(frame: bytes):
    if len(frame) != LEN_CHASSIS_ODOM:
        raise ValueError("bad odom frame length")
    if frame[0] != HEADER or frame[1] != ADDR_CHASSIS_ODOM or frame[2] != LEN_CHASSIS_ODOM:
        raise ValueError("bad odom frame header")
    if checksum_u8(frame[:-1]) != frame[-1]:
        raise ValueError("bad odom checksum")

    _, _, _, x, y, yaw, distance, vx, wz, ecd0, ecd1, ecd2, ecd3, _ = struct.unpack("<BBBffffffHHHHB", frame)
    return {
        "x": x,
        "y": y,
        "yaw": yaw,
        "distance": distance,
        "vx": vx,
        "wz": wz,
        "motor_ecd": [ecd0, ecd1, ecd2, ecd3],
    }
```

## Transport Notes

### Current USB CDC

Send the command frame bytes to the STM32 USB virtual serial port. Read odometry frames from the same port.

Recommended serial settings are not meaningful for USB CDC, but most host tools still accept:

- baudrate: `115200`
- data bits: `8`
- parity: none
- stop bits: `1`

### Later USART6 UART

Use the same frame bytes over UART6.

STM32 USART6 settings:

- baudrate: `115200`
- data bits: `8`
- parity: none
- stop bits: `1`
- flow control: none

## ROS Mapping

Suggested ROS topic mapping:

- Subscribe `/cmd_vel`
  - `linear.x -> vx`
  - `angular.z -> wz`
- Publish `/odom`
  - `pose.pose.position.x -> x`
  - `pose.pose.position.y -> y`
  - `yaw -> pose.pose.orientation`
  - `twist.twist.linear.x -> vx`
  - `twist.twist.angular.z -> wz`

The STM32 local frame is `X forward, Y left, Z up`, with positive yaw counter-clockwise.
