# Segment Odometry Reset And Automatic Hold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add acknowledged segment odometry reset and automatically hold wheel position whenever the final manual or ROS command is zero.

**Architecture:** MiniPC command parsing dispatches velocity and segment-reset frames. The receive callback only stores a pending reset request; `ObserveTask` applies it and resets wheel distance origins while preserving heading. `Chassis_Task` reuses its existing position controller for zero-command holding.

**Tech Stack:** STM32F407, C11, FreeRTOS/CMSIS-RTOS, UART6/USB CDC, ROS 2 Python, pytest.

## Global Constraints

- Preserve all current uncommitted project changes.
- Do not reset INS, magnetic heading, or the odometry heading during segment reset.
- Automatic hold depends only on the final `vx` and `wz` command, not communication timeout.
- Reuse the existing MIT and hold implementation; do not add another controller module.
- Keep the existing velocity frame backward compatible.

---

### Task 1: Segment Reset Protocol

**Files:**
- Modify: `Components/Device/Inc/minipc.h`
- Modify: `Components/Device/Src/minipc.c`
- Test: `tests/test_minipc_protocol.c`

**Interfaces:**
- Consumes: frame `42 33 06 01 <segment_id> <checksum>`.
- Produces: `MiniPC_TakeOdomResetRequest(uint8_t *segment_id)` and a 51-byte odometry frame containing the applied segment ID.

- [ ] Write a host C test for reset-frame parsing, duplicate-request handling, stream dispatch, and odometry packing.
- [ ] Build and run the test to confirm it fails because the new protocol is absent.
- [ ] Add the frame constants, pending-request latch, parser dispatch, and segment ID field.
- [ ] Build and run the test until it passes.

### Task 2: Translation-Only Odometry Reset

**Files:**
- Modify: `Components/Algorithm/Inc/odometry.h`
- Modify: `Components/Algorithm/Src/odometry.c`
- Modify: `Application/Tasks/Inc/observe_task.h`
- Modify: `Application/Tasks/Src/observe_task.c`
- Test: `tests/test_odometry_segment_reset.c`

**Interfaces:**
- Consumes: pending segment ID from `MiniPC_TakeOdomResetRequest`.
- Produces: `OdomEstimator_ResetDistanceOrigin(OdomEstimator_t *)`, zero segment position/distance, and preserved heading.

- [ ] Write a host C test proving wheel baselines reset while `heading_rad`, `last_yaw_rad`, and `yaw_ready` remain unchanged.
- [ ] Build and run the test to confirm the function is missing.
- [ ] Implement distance-origin reset and consume reset requests at the start of the observer loop.
- [ ] Build and run the test until it passes.

### Task 3: Zero-Command Position Hold

**Files:**
- Modify: `Application/Tasks/Inc/Chassis_Task.h`
- Modify: `Application/Tasks/Src/Chassis_Task.c`

**Interfaces:**
- Consumes: final constrained `state_set.vx` and `state_set.wz`.
- Produces: `auto_hold_active`; zero commands use existing position hold, nonzero commands immediately return to MIT control.

- [ ] Add a command-state testable predicate for manual/ROS zero commands.
- [ ] Confirm the test fails before implementation.
- [ ] Add the automatic hold transition and use the dedicated hold gains already defined in `Chassis_Task.h`.
- [ ] Verify force-raw mode clears hold references and sends zero current.

### Task 4: ROS Bridge And Protocol Documentation

**Files:**
- Modify: `Application/Tasks/Src/Ros_Task.c`
- Modify: `PC/pipe-measurement-nav/nav_ws/src/chassis_bridge/chassis_bridge/chassis_bridge_node.py`
- Modify: `PC/pipe-measurement-nav/nav_ws/src/chassis_bridge/test/test_ultrasonic_topics.py`
- Modify: `PC/pipe-measurement-nav/nav_ws/hardware/chassis_protocol.md`

**Interfaces:**
- Consumes: ROS `UInt8` segment reset requests.
- Produces: reset command frames and an `UInt8` applied-segment acknowledgement topic.

- [ ] Change the Python protocol test to require a 51-byte odometry frame and reset command packing.
- [ ] Run pytest and confirm the old implementation fails.
- [ ] Add the reset publisher/subscriber path and 51-byte parser.
- [ ] Run pytest and the host C tests.
- [ ] Document the exact byte layout and navigation acknowledgement sequence.

### Task 5: Verification

**Files:**
- Verify only; no new production files.

- [ ] Run all focused host C and Python tests.
- [ ] Inspect diffs for unrelated changes or duplicate parameters.
- [ ] Do not run the full firmware build; report that the user should build it in CLion as requested.

### Task 6: Non-Blocking Buzzer Alerts

**Files:**
- Modify: `Bsp/Inc/bsp_tim.h`
- Modify: `Bsp/Src/bsp_tim.c`
- Modify: `Bsp/Inc/buzzer_music.h`
- Modify: `Bsp/Src/buzzer_music.c`
- Modify: `Application/Tasks/Src/User_Task.c`
- Test: `tests/test_buzzer_alert.c`

**Interfaces:**
- Consumes: four-bit Status1 offline mask.
- Produces: three-note startup sound and motor-number beep groups without blocking an RTOS task.

- [ ] Verify the old blocking music implementation fails the new state-machine test.
- [ ] Replace it with a tick-driven sequencer and start TIM4 PWM once in BSP initialization.
- [ ] Report motor 1 to 4 with one to four beeps and silence immediately after recovery.
- [ ] Run the host test and ARM syntax check.
