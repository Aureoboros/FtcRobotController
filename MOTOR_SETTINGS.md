# Motor Settings Reference

This document records the current motor configuration and control mappings for `SVTestTeleop12nov.java`. **Refer to this file before making any changes to drive code.**

## Motor Hardware Configuration

### Drive Motors
- **Front Left Motor**: `frontLeftMotor`
- **Back Left Motor**: `backLeftMotor`
- **Front Right Motor**: `frontRightMotor`
- **Back Right Motor**: `backRightMotor`

### Motor Directions (Lines 107-110)
```java
frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
```
**All drive motors are set to FORWARD direction.**

---

## Control Mapping

### Joystick Controls
- **Left Joystick Y** (Up/Down):
  - Maps to `x` axis (swapped from standard)
  - Up (negative Y) = Forward (negative x)
  - Down (positive Y) = Backward (positive x)
  
- **Right Joystick X** (Left/Right):
  - Maps to `y` axis (swapped from standard, inverted)
  - Left (negative X) = Strafe Left (positive y)
  - Right (positive X) = Strafe Right (negative y)

### Trigger Controls (Rotation)
- **Rotation control is DISABLED** - All triggers (LT and RT) are now used for wheel control

### Button Overrides (Individual Wheel Control)
- **Left Bumper (LB)**: Control all 4 wheels simultaneously (DEDICATED)
- **Left Trigger (LT)**: Control all 4 wheels simultaneously (SAME AS LB)
  - **Both LB and LT** activate the same wheel pattern when held/pressed
  - **While held/pressed**: 
    - **Left side wheels** (both backward):
      - Front left wheel: backward at full power (1.0) - positive = backward
      - Back left wheel: backward at full power (-1.0) - negative = backward
    - **Right side wheels** (both forward):
      - Front right wheel: forward at full power (1.0) - positive = forward
      - Back right wheel: forward at full power (-1.0) - negative = forward
  - **Notes**: 
    - Front left motor: positive power (1.0) = backward (motor wiring specific)
    - Back left motor: negative power (-1.0) = backward (motor wiring specific)
    - Front right motor: positive power (1.0) = forward (motor wiring specific)
    - Back right motor: negative power (-1.0) = forward (standard)
    - This creates a specific rotation/movement pattern with all wheels active
  - Overrides normal mecanum drive calculations
  - **LB is dedicated to this function only** (alliance selection removed from LB)
  - **LT is dedicated to this function** (rotation control removed from LT)

- **Right Trigger (RT)**: Control all 4 wheels with different pattern (DEDICATED)
  - **While held/pressed**: 
    - **Left side wheels** (both forward):
      - Front left wheel: forward at full power (-1.0) - negative = forward
      - Back left wheel: forward at full power (1.0) - positive = forward
    - **Right side wheels** (both backward):
      - Front right wheel: backward at full power (-1.0) - negative = backward
      - Back right wheel: backward at full power (1.0) - positive = backward
  - **Notes**: 
    - Front left motor: negative power (-1.0) = forward
    - Back left motor: positive power (1.0) = forward (motor wiring specific)
    - Front right motor: negative power (-1.0) = backward
    - Back right motor: positive power (1.0) = backward
    - This creates a different rotation/movement pattern with all wheels active
  - Overrides normal mecanum drive calculations
  - **RT is dedicated to this function** (rotation control removed from RT)

---

## Mecanum Wheel Formula

### Current Formula (Lines 673-676)
```java
double frontLeftPower = y + x + rx;
double backLeftPower = -(y + x + rx);  // Inverted sign for forward movement (positive = forward)
double frontRightPower = y - x - rx;
double backRightPower = -(y - x - rx);  // Inverted sign for forward movement (negative = forward)
```

### Key Points
1. **Back wheels have inverted signs**: Back wheels use the negative of the front wheel formula to match motor wiring for forward movement.
2. **Forward movement fix**: Back wheels were going backward during forward movement, so signs were inverted.
3. **Side alignment maintained**: Both left wheels still use the same base formula (with back inverted), both right wheels use the same base formula (with back inverted).

### Axis Mapping (Swapped Motor Connection)
- `x` = Forward/Backward movement (negative = forward, positive = backward)
- `y` = Strafe Left/Right (positive = left, negative = right)
- `rx` = Rotation (negative = left/counter-clockwise, positive = right/clockwise)

---

## Rotation Behavior

### Left Rotation (LT pressed, rx negative)
- **Left side wheels**: Both backward (negative power)
- **Right side wheels**: Both forward (positive power)
- **Result**: Counter-clockwise rotation (robot turns left)

### Right Rotation (RT pressed, rx positive)
- **Left side wheels**: Both forward (positive power)
- **Right side wheels**: Both backward (negative power)
- **Result**: Clockwise rotation (robot turns right)

### Verification
For pure rotation (x=0, y=0):
- **Left rotation** (rx = -0.5):
  - `frontLeftPower = 0 + 0 + (-0.5) = -0.5` ✓
  - `backLeftPower = 0 + 0 + (-0.5) = -0.5` ✓
  - `frontRightPower = 0 - 0 - (-0.5) = 0.5` ✓
  - `backRightPower = 0 - 0 - (-0.5) = 0.5` ✓

- **Right rotation** (rx = 0.5):
  - `frontLeftPower = 0 + 0 + 0.5 = 0.5` ✓
  - `backLeftPower = 0 + 0 + 0.5 = 0.5` ✓
  - `frontRightPower = 0 - 0 - 0.5 = -0.5` ✓
  - `backRightPower = 0 - 0 - 0.5 = -0.5` ✓

---

## Important Notes

### "Swapped Motor Connection"
The code comments refer to a "swapped motor connection" which means:
- The standard x/y axes are swapped in the input mapping
- The signs are inverted to match the physical robot orientation
- This was discovered through iterative testing with dpad controls

### Power Scaling
Motor powers are scaled proportionally if they exceed `maxDrivePower`:
- Gamepad1: `GAMEPAD1_MAX_POWER = 1.0`
- Gamepad2: `GAMEPAD2_MAX_POWER = 0.5`

### Deadzone
- `JOYSTICK_DEADZONE = 0.1` applied to all joystick and trigger inputs

---

## History of Changes

### Latest Change (Rotation Fix)
- **Issue**: During rotation, front wheels were aligned and back wheels were aligned, but same-side wheels were not moving together.
- **Fix**: Modified mecanum formula so both left wheels use `y + x + rx` and both right wheels use `y - x - rx` (instead of standard `backLeft = y - x + rx` and `backRight = y + x - rx`).
- **Result**: Same-side wheels now rotate together correctly.

---

## Testing Checklist

When making changes, verify:
- [ ] Left joystick up moves robot forward
- [ ] Left joystick down moves robot backward
- [ ] Right joystick left strafes robot left
- [ ] Right joystick right strafes robot right
- [ ] LT rotates robot left (both left wheels backward, both right wheels forward)
- [ ] RT rotates robot right (both left wheels forward, both right wheels backward)
- [ ] Rotation happens in-place without translation
- [ ] Movement and rotation can be combined

---

**Last Updated**: Based on `SVTestTeleop12nov.java` state after rotation alignment fix.

