# Safe MIT Chassis Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the unsafe per-wheel automatic position hold with a bounded 200 Hz DRIVE/BRAKE/HOLD/FAULT controller while preserving low-speed MIT torque control.

**Architecture:** Pure C controller modules consume a complete four-wheel feedback snapshot and publish one coherent four-wheel current command in amperes. `Chassis_Task` owns mode and kinematics integration, while `Can_Task` only transmits the latest valid command snapshot.

**Tech Stack:** C11, STM32F407 HAL, FreeRTOS CMSIS-RTOS v1, host-side GCC assertion tests.

## Global Constraints

- Do not change N630 configuration, odometry, navigation, IMU fusion, or MiniPC protocol.
- Control and command publication run at 5 ms / 200 Hz.
- Default validation current limit is 0.5 A per wheel and default slew is 0.05 A per 5 ms.
- A fault zeros all four currents immediately and remains latched until the chassis enters powerless mode.
- Remove replaced automatic-hold code and macros instead of leaving commented dead code.
- Keep existing unrelated worktree changes intact.

---

### Task 1: Define and test the pure controller contract

**Files:**
- Create: `Components/Controller/Inc/chassis_control_manager.h`
- Create: `tests/test_chassis_control_manager.c`
- Create: `tests/run_chassis_control_tests.ps1`

**Interfaces:**
- Consumes: four logical forward-positive wheel speed/position values, feedback validity, pitch, motion enable, and four target wheel rpm values.
- Produces: `Chassis_Control_Output_t` containing state, fault, four current commands in A, and debug components.

- [ ] **Step 1: Write failing state and safety tests**

Cover these observable behaviors with `assert`:

```c
Chassis_ControlManager_Init(&manager, NULL);
Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
assert(output.state == CHASSIS_CTRL_DISABLED);

input.enabled = 1U;
input.target_rpm[0] = 20.0f;
Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
assert(output.state == CHASSIS_CTRL_DRIVE);
assert(fabsf(output.current_a[0]) <= 0.5f);

input.target_rpm[0] = 0.0f;
input.speed_rpm[0] = 20.0f;
Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
assert(output.state == CHASSIS_CTRL_BRAKE);

input.speed_valid[0] = 0U;
Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
assert(output.state == CHASSIS_CTRL_FAULT);
assert(output.current_a[0] == 0.0f);
```

- [ ] **Step 2: Run the host test and verify RED**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_chassis_control_tests.ps1
```

Expected: compilation fails because `chassis_control_manager.h` and its functions do not exist.

- [ ] **Step 3: Add the public types and declarations**

Define:

```c
typedef enum {
    CHASSIS_CTRL_DISABLED = 0,
    CHASSIS_CTRL_DRIVE,
    CHASSIS_CTRL_BRAKE,
    CHASSIS_CTRL_HOLD,
    CHASSIS_CTRL_FAULT,
} Chassis_Control_State_e;

typedef enum {
    CHASSIS_CTRL_FAULT_NONE = 0,
    CHASSIS_CTRL_FAULT_FEEDBACK,
    CHASSIS_CTRL_FAULT_OVERSPEED,
    CHASSIS_CTRL_FAULT_OSCILLATION,
    CHASSIS_CTRL_FAULT_SATURATION,
    CHASSIS_CTRL_FAULT_TIMING,
} Chassis_Control_Fault_e;
```

Declare `Chassis_ControlManager_Init`, `Chassis_ControlManager_Update`, `Chassis_ControlManager_Disable`, and `Chassis_ControlManager_SetDriveGains`.

- [ ] **Step 4: Keep RED focused**

Run the test again. Expected: link failure for the declared manager functions, proving the test now reaches the intended API.

### Task 2: Implement bounded DRIVE, BRAKE, HOLD, and FAULT modules

**Files:**
- Replace: `Components/Controller/Inc/chassis_mit_ctrl.h`
- Replace: `Components/Controller/Src/chassis_mit_ctrl.c`
- Replace: `Components/Controller/Inc/chassis_brake.h`
- Replace: `Components/Controller/Src/chassis_brake.c`
- Create: `Components/Controller/Inc/chassis_hold_ctrl.h`
- Create: `Components/Controller/Src/chassis_hold_ctrl.c`
- Create: `Components/Controller/Src/chassis_control_manager.c`
- Modify: `tests/test_chassis_control_manager.c`

**Interfaces:**
- `Chassis_Mit_Update`: bounded position/velocity/friction current for one wheel.
- `Chassis_Brake_Update`: bounded velocity damping current for one wheel.
- `Chassis_Hold_Capture` and `Chassis_Hold_Update`: one chassis-level longitudinal hold current shared by all wheels.
- `Chassis_ControlManager_Update`: state transitions, final current slew, feedback checks, and fault latch.

- [ ] **Step 1: Add failing behavior tests**

Add tests that assert:

- DRIVE current changes by no more than 0.05 A each 5 ms.
- Positive-to-negative target changes pass through zero without a current step.
- BRAKE never uses a position error term.
- Four wheels below 5 rpm for 200 ms transition from BRAKE to HOLD.
- HOLD uses one common longitudinal current rather than four independent position references.
- Four current sign reversals in a 100 ms zero-command window latch `CHASSIS_CTRL_FAULT_OSCILLATION`.
- Any wheel speed above 50 rpm after entering HOLD latches `CHASSIS_CTRL_FAULT_OVERSPEED`; high-speed command release must remain in BRAKE.
- Disabling the manager clears the fault and output.

- [ ] **Step 2: Run tests and verify RED**

Expected: state-transition and current-bound assertions fail with the placeholder implementation.

- [ ] **Step 3: Implement minimal pure modules**

Use these equations:

```text
DRIVE: I = Kp_pos * clamp(q_ref - q, +/-5 deg)
             + Kd_speed * (rpm_ref - rpm)
             + I_friction * tanh(rpm_ref / rpm_scale)
BRAKE: I = clamp(-K_brake * rpm, +/-I_brake_max)
HOLD:  I = Kx * (q_hold - q_long) - Kv * rpm_long + Kg * sin(pitch)
```

Use a sorted middle-pair average for `q_long` and `rpm_long`. Apply final state-specific magnitude limits and a manager-level 10 A/s slew. Fault output bypasses the slew limiter and becomes zero immediately.

- [ ] **Step 4: Run tests and verify GREEN**

Run the host script. Expected: all assertions pass.

- [ ] **Step 5: Refactor names and remove duplicated limit helpers**

Keep helpers private to one module where possible. Configuration fields use physical units, such as `position_kp_a_per_deg`, `speed_kd_a_per_rpm`, and `current_limit_a`.

- [ ] **Step 6: Run tests after refactor**

Expected: all assertions still pass without warnings.

### Task 3: Integrate the controller with chassis and CAN tasks

**Files:**
- Modify: `Application/Tasks/Inc/Chassis_Task.h`
- Modify: `Application/Tasks/Src/Chassis_Task.c`
- Modify: `Application/Tasks/Src/Can_Task.c`
- Modify: `Bsp/Src/vofa.c`

**Interfaces:**
- `Chassis_Task` publishes `Chassis_Current_Command_t { float current_a[4]; uint32_t tick; uint32_t sequence; }` every 5 ms.
- `Can_Task` calls `chassis_get_current_command` and sends only a complete, non-stale snapshot.
- Legacy `target_current` remains a derived milliamp debug value; control math does not use it.

- [ ] **Step 1: Add a failing command-snapshot host test**

Extend the manager test utility to verify that a four-current publication copies all four values and rejects a command older than 15 ms.

- [ ] **Step 2: Run and verify RED**

Expected: compilation or assertion failure because the snapshot API is absent.

- [ ] **Step 3: Remove unsafe task-local hold behavior**

Delete:

- `chassis_hold_position_control`
- `chassis_hold_test_control`
- `chassis_update_auto_hold`
- `CHASSIS_HOLD_STATIC_CURRENT` and associated hold macros
- Per-motor `hold_pos_*`, `ff_breakaway_*`, and brake state fields that moved into controller modules
- `auto_hold_active`

`CHASSIS_HOLD_TEST`, manual zero command, and ROS zero command all feed the same manager state machine.

- [ ] **Step 4: Wire feedback and command snapshots**

At 5 ms intervals, build `Chassis_Control_Input_t` from normalized `speed_rpm`, continuous `pos_deg`, Status1/Status4 age checks, pitch, and wheel targets. Publish one complete four-wheel current result.

In powerless mode call `Chassis_ControlManager_Disable`. Re-entering a powered mode starts from a cleared manager state.

- [ ] **Step 5: Update CAN transmission**

Use `float current_a[4]` directly for VESC CAN transmission, applying installation direction signs only at the CAN boundary. If the snapshot is stale, send four zero-current commands.

- [ ] **Step 6: Preserve VOFA tuning compatibility safely**

Map `KP/KI/KD` receive fields to position gain, velocity gain, and continuous friction feedforward respectively, converting existing milliamp-style values to A. Clamp runtime gains to manager safety ranges. Report state, fault, current components, and final current without exceeding 27 VOFA channels.

- [ ] **Step 7: Run host tests**

Expected: manager and snapshot tests pass.

### Task 4: Verify the firmware build and regression surface

**Files:**
- Modify only files required by compiler diagnostics from Tasks 1-3.

**Interfaces:**
- Existing odometry, MiniPC, buzzer, ultrasonic, and IMU APIs remain unchanged.

- [ ] **Step 1: Run all host tests**

Run the existing odometry, MiniPC, buzzer tests plus the new chassis controller test with their host scripts/commands. Expected: all pass.

- [ ] **Step 2: Build the firmware once**

Run:

```powershell
cmake --build E:\Mearsuring_robot\cmake-build-debug --target Mearsuring_robot.elf -j 14
```

Expected: successful link with no new implicit-declaration, type, or unused-function diagnostics.

- [ ] **Step 3: Inspect the scoped diff**

Confirm that no files outside controller, chassis/CAN task, VOFA debug, tests, and this plan were changed by the implementation.

- [ ] **Step 4: Report hardware validation limits**

State clearly that software tests cannot validate motor direction, physical holding torque, or regenerative energy. First hardware image must retain the 0.5 A limit and follow the restrained test sequence from the design.
