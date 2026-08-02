# Chassis Low-Speed and Yaw-Rate Control Design

## Objective

Replace the continuous-motion pseudo-MIT controller with a predictable cascaded
controller that can:

- remain stable with the chassis lifted;
- build enough current to overcome ground scrub friction during skid steering;
- track a realistic body yaw rate using IMU gyro feedback without integrating yaw;
- retain immediate braking, position-hold test mode, odometry, and VESC safety checks;
- keep controller responsibilities outside `Chassis_Task.c`.

The initial maximum commanded yaw rate is 1.26 rad/s, approximately one full turn
in five seconds. This replaces the current 13.2 rad/s full-stick request.

## Root Cause

The current drive controller integrates a wheel position target and computes:

```text
current = position_stiffness * position_error
        + velocity_gain * velocity_error
        + friction_feedforward
```

The position error is limited to 90 degrees. With low gains, the controller cannot
build enough sustained current to overcome four-wheel scrub friction. Raising the
velocity gain produces enough ground torque, but creates a delayed high-gain loop
that oscillates when unloaded. A forced 120 rpm turn target couples torque demand
to speed demand and therefore cannot solve both conditions.

## Architecture

### Wheel Velocity Controller

Create a standalone wheel velocity PI/current controller.

Inputs:

- target wheel speed in rpm;
- measured wheel speed in rpm;
- control period in seconds;
- motor feedback validity.

Output:

- requested motor current in mA.

Behavior:

- proportional speed-error current provides damping;
- a limited conditional integrator builds load torque on the ground and on slopes;
- signed breakaway feedforward handles initial static friction;
- anti-windup freezes integration when current is saturated or the wheel exceeds
  its target speed;
- output slew limiting prevents current steps;
- zero target delegates to the existing brake module;
- reset clears all accumulated state on mode changes, feedback loss, or command
  reversal.

Continuous motion no longer depends on Status 4 position feedback. Status 4 remains
available for odometry and the dedicated position-hold test mode.

### Yaw-Rate Current Controller

Create a standalone IMU yaw-rate controller.

Inputs:

- requested body yaw rate in rad/s;
- measured gyro Z rate in rad/s;
- average left and right wheel speed tracking error;
- control period and feedback validity.

Output:

- signed differential current in mA.

Positive yaw current is subtracted from both left motors and added to both right
motors. The controller uses a limited PI law with the following safeguards:

- active only for an explicit turn command;
- integral reset at zero command and command-direction reversal;
- no integration when wheel speed has reached or exceeded its target, preventing
  a lifted chassis from winding up because the body itself cannot rotate;
- current and integral limits;
- output slew limiting;
- reset on invalid IMU, invalid motor feedback, raw-force mode, hold mode, or stale
  CAN feedback.

### Command and Output Coordination

- Remove the forced 120 rpm minimum turn speed.
- Bound the commanded yaw rate to 1.26 rad/s before kinematic decomposition.
- Keep the existing straight-line heading hold. Explicit turn commands use the
  yaw-rate current controller; straight heading correction does not use it.
- Add wheel PI current and yaw differential current, then scale all four currents
  by one common factor when any current exceeds the motor limit. This preserves
  the requested left/right current ratio instead of clipping motors independently.
- Keep CAN current transmission at its existing 200 Hz rate in this change.

## Parameter Handling

Keep VOFA runtime tuning compatible while assigning conventional meanings:

- `KP`: wheel speed proportional current gain, mA/rpm;
- `KI`: wheel speed integral current gain, mA/(rpm*s);
- `KD`: wheel breakaway feedforward current, mA.

The active values remain stored in each motor controller and can be reported by the
existing VOFA motor-info frame. Yaw-rate parameters live in one typed configuration
structure with conservative defaults rather than being distributed as macros.

## Failure Handling

- Any stale Status 1 feedback resets that motor and commands zero current.
- Invalid or non-positive `dt` resets the affected controller.
- An invalid gyro value disables and resets yaw-rate assistance while preserving
  the wheel velocity loop.
- Current output is bounded before conversion to `int16_t`.
- Hold and braking controllers remain independent and are not modified by the
  yaw-rate controller.

## Verification

Host tests cover:

- wheel PI response, anti-windup, reversal reset, output limit, and slew limit;
- yaw-rate sign convention, integral build-up under ground load, lifted-wheel
  windup prevention, command reversal, reset, and current limit;
- common-factor four-wheel current scaling.

Vehicle tests proceed in this order:

1. Lifted straight and turn commands: no current-limit oscillation or overspeed.
2. Ground pure turn: smooth start, continuous yaw, and prompt stop.
3. Ground combined forward/turn command: stable radius and no current chatter.
4. Incline hold and climb test: verify integral current limit and motor temperature.
5. VOFA capture: confirm target rpm, actual rpm, target current, gyro yaw rate, and
   commanded yaw rate remain consistent.
