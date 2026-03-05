# FRC Robot Troubleshooting Guide

Quick-reference for every tunable value in this codebase. Each section describes the symptom, the fix, and exactly where to change it.

---

## Table of Contents

1. [Drive Speed](#1-drive-speed)
2. [Turn / Rotation Speed](#2-turn--rotation-speed)
3. [Joystick Deadband](#3-joystick-deadband)
4. [Reversing a Drive Motor](#4-reversing-a-drive-motor)
5. [Reversing a Steer (Azimuth) Motor](#5-reversing-a-steer-azimuth-motor)
6. [Wheel Not Pointing Straight (Encoder Offset)](#6-wheel-not-pointing-straight-encoder-offset)
7. [Swerve Module Jittering / Oscillating](#7-swerve-module-jittering--oscillating)
8. [Drive Motors Feel Sluggish or Jerky (Drive PID)](#8-drive-motors-feel-sluggish-or-jerky-drive-pid)
9. [Current Limits](#9-current-limits)
10. [Robot Drifts or Curves When Driving Straight](#10-robot-drifts-or-curves-when-driving-straight)
11. [Limelight Alignment Too Slow or Oscillates](#11-limelight-alignment-too-slow-or-oscillates)
12. [Limelight Not Seeing Tags / Wrong Tag](#12-limelight-not-seeing-tags--wrong-tag)
13. [Autonomous Path Following Issues](#13-autonomous-path-following-issues)
14. [Shooter Speed Tuning](#14-shooter-speed-tuning)
15. [Feed / Transfer Motor Direction or Speed](#15-feed--transfer-motor-direction-or-speed)
16. [Gyro Issues](#16-gyro-issues)
17. [Wheel Radius / Odometry Drift](#17-wheel-radius--odometry-drift)
18. [Robot Tips or Wheels Slip on Fast Acceleration](#18-robot-tips-or-wheels-slip-on-fast-acceleration)
19. [CAN Bus Errors](#19-can-bus-errors)
20. [Module Motor IDs](#20-module-motor-ids)
21. [Brake vs Coast Mode](#21-brake-vs-coast-mode)
22. [Simulation Tuning](#22-simulation-tuning)

---

## 1. Drive Speed

**Symptom:** Robot drives too slow or too fast.

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`
**Line:** 83

```java
public static final double kSpeedAt12Volts = 4.53; // m/s
```

- **Increase** this value to allow faster driving (up to your motor's true free speed).
- **Decrease** to cap top speed.
- This is used as the max velocity reference throughout the drivetrain.

**Also check:** `RobotContainer.java` — the joystick inputs are multiplied by `MAX_LINEAR_SPEED` which comes from `TunerConstants.kSpeedAt12Volts`. If you want a "slow mode" button, scale the joystick input by a fraction (e.g., `* 0.5`).

---

## 2. Turn / Rotation Speed

**Symptom:** Robot rotates too slowly or too quickly when spinning.

**File:** `src/main/java/frc/robot/commands/DriveCommands.java`
**Lines:** 39-40

```java
private static final double ANGLE_MAX_VELOCITY = 8.0;       // rad/s
private static final double ANGLE_MAX_ACCELERATION = 20.0;   // rad/s²
```

- **Increase `ANGLE_MAX_VELOCITY`** for faster rotation (try 10.0-12.0).
- **Decrease** for slower, more controlled rotation (try 4.0-6.0).
- `ANGLE_MAX_ACCELERATION` controls how quickly rotation ramps up. Lower = smoother but slower response.

**Also:** The overall max angular rate is derived from `kSpeedAt12Volts` in `TunerConstants.java` line 83. The actual max rotation speed is `kSpeedAt12Volts / (drivebase radius)`.

---

## 3. Joystick Deadband

**Symptom:** Robot creeps when joysticks are released, OR robot doesn't respond to small inputs.

**File:** `src/main/java/frc/robot/commands/DriveCommands.java`
**Line:** 36

```java
private static final double DEADBAND = 0.1;
```

- **Increase** (e.g., 0.15) if robot creeps when sticks are centered.
- **Decrease** (e.g., 0.05) if you want finer low-speed control.
- Range: 0.0 to 1.0 (0.05-0.15 is typical).

---

## 4. Reversing a Drive Motor

**Symptom:** One side of the robot drives backward when it should go forward, or a single module drives the wrong way.

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`
**Lines:** 93-94

```java
public static final InvertedValue kInvertLeftSide = InvertedValue.CounterClockwise_Positive;  // false
public static final InvertedValue kInvertRightSide = InvertedValue.Clockwise_Positive;        // true
```

- Swap `CounterClockwise_Positive` and `Clockwise_Positive` for the affected side.
- Left side default = **not inverted**, Right side default = **inverted**.

**If a single module is wrong** and not the whole side, the motor wiring or CAN ID may be swapped. Check the module motor IDs in the same file (see [Section 20](#20-module-motor-ids)).

---

## 5. Reversing a Steer (Azimuth) Motor

**Symptom:** A wheel rotates the wrong direction when steering — it spins away from the target angle instead of toward it.

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`

| Module      | Line | Current Value |
|-------------|------|---------------|
| Front Left  | 142  | `true`        |
| Front Right | 153  | `true`        |
| Back Left   | 164  | `true`        |
| Back Right  | 175  | `true`        |

```java
// Example: Front Left
private static final boolean kFrontLeftSteerMotorInverted = true;
```

- Flip `true` to `false` (or vice versa) for the affected module.

---

## 6. Wheel Not Pointing Straight (Encoder Offset)

**Symptom:** When the robot boots up, one or more wheels are not aligned — they point at an angle instead of straight ahead.

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`

| Module      | Line | Current Offset (rotations) |
|-------------|------|----------------------------|
| Front Left  | 141  | `0.105224609375`           |
| Front Right | 152  | `0.04052734375`            |
| Back Left   | 163  | `-0.294189453125`          |
| Back Right  | 174  | `0.384765625`              |

**How to fix:**
1. With the robot disabled, physically point all wheels straight forward.
2. Read each CANcoder's absolute position (use Phoenix Tuner X or Shuffleboard).
3. Replace the offset value with the reading for that module.
4. Deploy and verify — wheels should now point straight on boot.

**If using Phoenix Tuner X:** You can use the "Set Magnet Offset" feature in the CANcoder config page, then set the code offset to 0. Either approach works.

---

## 7. Swerve Module Jittering / Oscillating

**Symptom:** Steer motors buzz, vibrate, or oscillate back and forth rapidly when holding position.

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`
**Lines:** 25-30

```java
private static final double kSteerKP = 35;
private static final double kSteerKI = 0;
private static final double kSteerKD = 0.5;
private static final double kSteerKS = 0.1;
private static final double kSteerKV = 1.16;
private static final double kSteerKA = 0;
```

**Fixes:**
- **Oscillating rapidly:** Decrease `kSteerKP` (try 25, then 20). This is the most common fix.
- **Slow to reach target angle:** Increase `kSteerKP` (try 40-50).
- **Overshooting then correcting:** Increase `kSteerKD` (try 1.0-2.0).
- **Buzzing at rest:** Increase `kSteerKS` (static friction feedforward). Try 0.15-0.25.

**Also check Motion Magic settings in:** `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java`
**Lines:** 134-138

```java
steerConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / kSteerGearRatio;
steerConfig.MotionMagic.MotionMagicAcceleration = steerConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
steerConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * kSteerGearRatio;
steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
```

- If steering feels jerky, **decrease the acceleration** by changing `0.100` to `0.200` (halves the accel).
- If steering is too slow, decrease the divisor (e.g., `0.050`).

---

## 8. Drive Motors Feel Sluggish or Jerky (Drive PID)

**Symptom:** Driving feels unresponsive, or the robot lurches when starting to move.

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`
**Line:** 35

```java
private static final Slot0Configs driveGains = new Slot0Configs()
    .withKP(0.1).withKI(0.0).withKD(0).withKS(0).withKV(0.124);
```

- **Sluggish acceleration:** Increase `KP` (try 0.2-0.5).
- **Lurching/jerky:** Decrease `KP` (try 0.05).
- **Doesn't reach full speed:** Increase `KV` slightly (try 0.13).
- **Robot vibrates at constant speed:** Decrease `KP` or add small `KD` (try 0.01).

**For smoother driving overall**, consider the joystick input curve in `DriveCommands.java`. The raw joystick value is squared for a gentler response curve — look for where the linearMagnitude is processed.

---

## 9. Current Limits

**Symptom:** Motors brown out, breakers trip, or the robot loses power during matches.

### Drive Motor Current Limits

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`
**Line:** 57

```java
private static final double kSlipCurrent = 120; // Amps
```

This is the **peak torque current** for drive motors. Also used as the stator current limit.

**File:** `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java`
**Line:** 111

```java
driveConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
```

- **Lower slip current** (e.g., 80A) if wheels spin out or breakers trip.
- **Lower supply current** (e.g., 40A) if the robot browns out frequently.

### Steer Motor Current Limit

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`
**Line:** 69

```java
private static final double kSteerStatorCurrentLimit = 60; // Amps
```

- Usually fine at 60A. Lower to 40A if steer motors overheat.

---

## 10. Robot Drifts or Curves When Driving Straight

**Symptom:** Pushing the joystick straight forward makes the robot veer left or right.

**Possible causes and fixes:**

1. **Encoder offsets wrong** — See [Section 6](#6-wheel-not-pointing-straight-encoder-offset). If one wheel is slightly off-angle, the robot will drift.

2. **Wheel radius incorrect** — If one wheel's effective radius differs (worn tread), odometry will be slightly off.
   - **File:** `TunerConstants.java` line 91
   ```java
   public static final double kWheelRadius = Units.inchesToMeters(2);
   ```
   - Measure your actual wheel radius. Even 0.05" matters.

3. **Gyro drift** — Check if the Pigeon2 is calibrated. Vibration can cause drift.
   - Let the robot sit still for 10 seconds on boot before driving.

4. **Motor inversions wrong** — A module driving backward while others go forward will cause curving. See [Section 4](#4-reversing-a-drive-motor).

---

## 11. Limelight Alignment Too Slow or Oscillates

**Symptom:** When using the align-to-target button (Y button), the robot turns too slowly, overshoots, or wobbles around the target.

**File:** `src/main/java/frc/robot/commands/DriveCommands.java`
**Lines:** 49-50

```java
private static final double ALIGN_KP = 0.1;
private static final double ALIGN_KD = 0.005;
```

**Line:** 172 (tolerance)

```java
angleController.setTolerance(1.0); // degrees
```

**Fixes:**
- **Too slow to align:** Increase `ALIGN_KP` (try 0.15, then 0.2).
- **Overshoots / oscillates around target:** Decrease `ALIGN_KP` (try 0.07) or increase `ALIGN_KD` (try 0.01-0.02).
- **Settles but not accurately enough:** Decrease tolerance (try 0.5 degrees).
- **Good speed but wobbly:** Increase `ALIGN_KD` to dampen oscillation.

**Alignment also uses the profiled PID max velocity/acceleration.** If the alignment rotation speed is capped too low:

**Lines:** 39-40

```java
private static final double ANGLE_MAX_VELOCITY = 8.0;       // rad/s
private static final double ANGLE_MAX_ACCELERATION = 20.0;   // rad/s²
```

---

## 12. Limelight Not Seeing Tags / Wrong Tag

**Symptom:** `hasTarget()` returns false, or the robot aligns to the wrong AprilTag.

### Code-side checks

**File:** `src/main/java/frc/robot/subsystems/Limelight.java`
**Line:** 12

```java
table = NetworkTableInstance.getDefault().getTable("limelight");
```

- Make sure the NetworkTable name matches your Limelight's hostname. If you renamed it (e.g., `limelight-front`), update this string.

### Limelight Web UI Configuration (`http://limelight.local:5801`)

| Setting | Recommended Value | Notes |
|---------|-------------------|-------|
| Pipeline Type | AprilTag | Must be set to AprilTag, not Retroreflective |
| Tag Family | 36h11 | FRC standard |
| Sort Mode | **Closest** | Reports nearest tag as primary target |
| LED Mode | Off | Not needed for AprilTags |
| Resolution | 1280x960 or 640x480 | Higher = more range, lower = faster |
| Detector Downscale | 2 | Balance of speed/range |

### Filtering to specific tags

If you only want to align to goal posts (and ignore other field tags), use the **Tag ID Filter** in the Limelight pipeline settings to whitelist only the goal post tag IDs.

### Common issues

- **No target at all:** Check if the camera is plugged in, has power (green light), and the pipeline is set to AprilTag mode.
- **Intermittent targets:** Increase exposure, reduce resolution, or reduce downscale factor.
- **Wrong tag selected:** Set Sort Mode to "Closest" to prioritize nearest tag. With two tags on a goal post, both give similar `tx` values, so either one works for alignment.

---

## 13. Autonomous Path Following Issues

**Symptom:** Robot doesn't follow PathPlanner paths accurately, overshoots turns, or drifts off course.

### Rotation PID

**File:** `src/main/java/frc/robot/subsystems/drive/Drive.java`
**Line:** 126

```java
new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID for path following
```

- **Overshoots turns:** Decrease KP (try 3.0) or add KD (try 0.1).
- **Doesn't rotate enough:** Increase KP (try 7.0).

### Robot Physical Parameters

**File:** `src/main/java/frc/robot/subsystems/drive/Drive.java`
**Lines:** 64-66

```java
.robotMass(74.088)           // kg — weigh your robot!
.robotMOI(6.883)             // kg*m² — moment of inertia
.wheelCOF(1.2)               // coefficient of friction
```

- **Robot overshoots paths:** Your `robotMass` may be too low. Weigh the robot and update.
- **Wheels slip in auto:** Lower `wheelCOF` (try 1.0 for worn treads, 0.8 for slick carpet).
- **Rotation is wrong in auto:** `robotMOI` may be off. This is hard to measure — try values between 4.0 and 10.0 and see what tracks best.

### Odometry frequency

**File:** `src/main/java/frc/robot/subsystems/drive/Drive.java`
**Line:** 53

```
250 Hz for CAN FD, 100 Hz for CAN 2.0
```

If you're on CAN 2.0 (not CANivore), you only get 100 Hz odometry. This is less accurate at high speeds. Consider upgrading to a CANivore for CAN FD.

---

## 14. Shooter Speed Tuning

**Symptom:** Game pieces fly too far, not far enough, or inconsistently.

**File:** `src/main/java/frc/robot/RobotContainer.java`

| Function | Line | Motor | Current Speed |
|----------|------|-------|---------------|
| Auto shoot | 121 | Launch | 0.8 |
| Auto shoot | 121 | Transfer | 0.6 |
| Teleop shoot (right trigger) | 207 | Launch | 0.5 |
| Teleop shoot | 207 | Transfer | 0.4 |

- **Shoots too far:** Decrease launch speed.
- **Doesn't reach target:** Increase launch speed.
- **Inconsistent shots:** Make sure transfer and launch speeds have a good ratio. Launch should typically be higher than transfer to avoid jamming.

### Shooter Motor PID

**File:** `src/main/java/frc/robot/subsystems/Shooter.java`
**Lines:** 33-35 (Transfer motor), 40-41 (Launch motor)

```java
transferConfig.Slot0.kP = 0.1;
transferConfig.Slot0.kI = 0.0;
transferConfig.Slot0.kD = 0.0;
```

- These are only relevant if you switch to closed-loop velocity control. Currently using open-loop (percent output).

---

## 15. Feed / Transfer Motor Direction or Speed

**Symptom:** Feed motor runs the wrong way, or game pieces don't transfer properly.

### Feed Speeds

**File:** `src/main/java/frc/robot/RobotContainer.java`

| Function | Line | Speed |
|----------|------|-------|
| Feed Move Forward (Left Bumper) | 189 | 0.2 |
| Feed Move Backward (Right Bumper) | 196 | -0.3 |
| Feed Motor (Left Trigger) | 201 | 0.3 |

- Flip the sign to reverse direction.
- Increase magnitude for faster feed.

### Motor Inversion

**File:** `src/main/java/frc/robot/subsystems/Shooter.java`
**Lines:** 18, 26

```java
feedMotor.setInverted(false);
feedMoveMotor.setInverted(false);
```

- Change to `true` if the motor runs the wrong direction.

---

## 16. Gyro Issues

**Symptom:** Robot heading is wrong, field-relative driving is off, or the robot spins unexpectedly.

### Pigeon2 CAN ID

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`
**Line:** 96

```java
private static final int kPigeonId = 0;
```

- Make sure this matches the actual CAN ID of your Pigeon2 in Phoenix Tuner X.

### Gyro is reversed

**File:** `src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java`

The Pigeon2 yaw should increase when the robot turns counter-clockwise (left). If field-relative driving is mirrored, the gyro may need to be inverted. Check the `getYaw()` signal and negate it if needed.

### NavX alternative

**File:** `src/main/java/frc/robot/subsystems/drive/GyroIONavX.java`

If using a NavX instead of Pigeon2, make sure you're instantiating the correct gyro IO class in `RobotContainer.java`.

### Reset heading

If field-relative driving is "off" after a match restart, you likely need to reset the gyro heading. Check if you have a button binding to zero the gyro (e.g., `drivetrain.seedFieldCentric()`).

---

## 17. Wheel Radius / Odometry Drift

**Symptom:** The robot's reported position drifts from reality over time, or autonomous distances are consistently too long or too short.

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`
**Line:** 91

```java
public static final double kWheelRadius = Units.inchesToMeters(2); // inches input
```

- Measure your actual wheel radius precisely. Worn wheels will be smaller.
- If auto paths consistently overshoot by 5%, your wheel radius is likely ~5% too small — increase it.
- If auto paths undershoot, decrease the wheel radius value.

### Gear ratio

**Line:** 89

```java
public static final double kDriveGearRatio = 6.821052631578947;
```

- This must match your module's actual gear ratio. Check your swerve module documentation (SDS MK4i, etc.).

---

## 18. Robot Tips or Wheels Slip on Fast Acceleration

**Symptom:** Robot wheelies, tips forward on hard braking, or wheels spin out when accelerating.

### Reduce current limits

See [Section 9](#9-current-limits). Lowering `kSlipCurrent` is the most direct fix.

### Reduce max speed for autonomous

In PathPlanner path settings, reduce the max velocity and max acceleration constraints.

### Add acceleration limiting to teleop

In `DriveCommands.java`, you can add a `SlewRateLimiter` to the joystick inputs to limit how quickly the driver can change speed. This isn't currently implemented but is a common addition:

```java
// Add as class field
private static final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0); // m/s per second
private static final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
```

---

## 19. CAN Bus Errors

**Symptom:** Devices show as missing, intermittent motor control, red LEDs on motor controllers.

### CAN Bus Name

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`
**Line:** 79

```java
public static final CANBus kCANBus = new CANBus("");
```

- `""` = default RoboRIO CAN bus.
- If using a CANivore, set this to your CANivore's name (e.g., `"canivore"`).

### Check all device IDs are unique

See [Section 20](#20-module-motor-ids). Every device on the same CAN bus must have a unique ID.

### Common CAN fixes

- Check wiring termination (120 ohm resistor at end of bus).
- Reduce CAN bus length.
- Check for damaged wires or loose Weidmuller connectors.
- Use Phoenix Tuner X to verify all devices are visible.

---

## 20. Module Motor IDs

Reference for all CAN device IDs. Every ID must be unique on the CAN bus.

**File:** `src/main/java/frc/robot/generated/TunerConstants.java`

| Module | Drive Motor ID | Steer Motor ID | Encoder ID | Lines |
|--------|---------------|----------------|------------|-------|
| Front Left | 29 | 26 | 31 | 138-140 |
| Front Right | 28 | 27 | 24 | 149-151 |
| Back Left | 21 | 22 | 27 | 160-162 |
| Back Right | 20 | 30 | 25 | 171-173 |

| Other Device | ID | Line |
|-------------|-----|------|
| Pigeon2 Gyro | 0 | 96 |

**File:** `src/main/java/frc/robot/RobotContainer.java`
**Line:** 46

| Shooter Motor | ID |
|--------------|-----|
| Feed Motor | 15 |
| Feed Move Motor | 16 |
| Transfer Motor | 17 |
| Launch Motor | 18 |

If a motor doesn't respond, verify its CAN ID in Phoenix Tuner X matches the code.

---

## 21. Brake vs Coast Mode

**Symptom:** Robot slides when stopping (coast), or robot is too hard to push when disabled (brake).

**File:** `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java`
**Lines:** 104, 122

```java
driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // Drive motors
steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // Steer motors
```

- **Drive motors:** Use `Brake` for competition (stops faster). Use `Coast` for testing/pushing the robot around.
- **Steer motors:** Almost always keep on `Brake` so wheels hold their angle.

---

## 22. Simulation Tuning

Only relevant when running in simulation mode.

**File:** `src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java`
**Lines:** 29-36

```java
private static final double DRIVE_KP = 0.05;
private static final double DRIVE_KD = 0.0;
private static final double DRIVE_KS = 0.0;
private static final double DRIVE_KV = 1.0 / (Units.rotationsToRadians(1.0 / 0.91035));
private static final double TURN_KP = 8.0;
private static final double TURN_KD = 0.0;
```

These are separate from the real robot gains. Tune them independently if the simulation doesn't behave like the real robot.

---

## Quick Reference: File Locations

| What | File | Key Lines |
|------|------|-----------|
| All swerve hardware config | `src/main/java/frc/robot/generated/TunerConstants.java` | 25-179 |
| Drive/steer motor config | `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java` | 96-139 |
| Teleop drive behavior | `src/main/java/frc/robot/commands/DriveCommands.java` | 36-50 |
| PathPlanner / auto config | `src/main/java/frc/robot/subsystems/drive/Drive.java` | 64-66, 126 |
| Limelight subsystem | `src/main/java/frc/robot/subsystems/Limelight.java` | 12 |
| Shooter speeds | `src/main/java/frc/robot/RobotContainer.java` | 121, 189-207 |
| Shooter motor config | `src/main/java/frc/robot/subsystems/Shooter.java` | 17-41 |
| Button bindings | `src/main/java/frc/robot/RobotContainer.java` | 180-230 |
