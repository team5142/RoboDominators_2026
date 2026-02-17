# SysId Characterization Setup Guide

## Quick Start

### Switching Between Normal Operation and SysId Mode

**RobotContainer.java** has two clearly marked sections:

1. **NORMAL OPERATION BUTTONS** (lines ~200-245)
   - Currently active
   - Comment out this entire section when running SysId

2. **SYSID CHARACTERIZATION BUTTONS** (lines ~247-305)
   - Currently commented out with `/* ... */`
   - Uncomment this section for SysId testing

### How to Switch Modes

**For SysId Mode:**
```java
// In RobotContainer.java configureButtonBindings():

// ========== NORMAL OPERATION BUTTONS (COMMENT OUT FOR SYSID) ==========
/*
// Y/B/X: SmartDrive to tags
new JoystickButton(driverController, XboxController.Button.kX.value)
... (comment out all normal buttons)
*/
// ========== END NORMAL OPERATION BUTTONS ==========

// ========== SYSID CHARACTERIZATION BUTTONS (COMMENT OUT FOR NORMAL OPERATION) ==========
// (Remove the /* and */ to uncomment)
// SIGNAL LOGGER CONTROL
new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
... (all SysId buttons now active)
```

---

## Button Mappings for SysId

### Signal Logger Control
- **LEFT BUMPER**: Start logging (do this FIRST before any tests)
- **RIGHT BUMPER**: Stop logging (do this LAST after all tests)

### Translation Tests (Drive Motors)
Tests the drive wheel motors for feedforward gains (kS, kV, kA)

- **Y Button**: Quasistatic Forward (slow voltage ramp, drives forward)
- **A Button**: Quasistatic Reverse (slow voltage ramp, drives backward)
- **B Button**: Dynamic Forward (voltage step, quick acceleration forward)
- **X Button**: Dynamic Reverse (voltage step, quick acceleration backward)

### Steer Tests (Module Steering Motors)
Tests the azimuth/steering motors for PID gains

⚠️ **IMPORTANT**: Before steer tests, disable CANcoder fusion (see below)

- **POV UP (D-Pad Up)**: Quasistatic Forward
- **POV DOWN (D-Pad Down)**: Quasistatic Reverse  
- **POV RIGHT (D-Pad Right)**: Dynamic Forward
- **POV LEFT (D-Pad Left)**: Dynamic Reverse

### Rotation Tests (Robot Spin)
Tests the entire robot rotation for trajectory following

- **RIGHT TRIGGER (>0.5)**: Quasistatic Forward (slow spin CW)
- **LEFT STICK BUTTON (L3)**: Quasistatic Reverse (slow spin CCW)
- **RIGHT STICK BUTTON (R3)**: Dynamic Forward (fast spin CW)

(Dynamic Reverse not mapped - typically not needed)

---

## Test Procedure

### Translation Characterization (Most Common)

1. **Setup**
   - Comment out normal operation buttons in RobotContainer.java
   - Uncomment SysId buttons section
   - Deploy code
   - Clear 15m (50ft) of straight driving space
   - Full battery recommended

2. **Run Tests** (in TeleOp mode)
   ```
   a. Press LEFT BUMPER (start logging)
   b. Press and hold Y (quasistatic forward) for 3-5 seconds
   c. Let robot coast to stop
   d. Drive back to start
   e. Press and hold A (quasistatic reverse) for 3-5 seconds
   f. Let robot coast to stop
   g. Drive back to start
   h. Press and hold B (dynamic forward) for 1-2 seconds
   i. Let robot coast to stop
   j. Drive back to start
   k. Press and hold X (dynamic reverse) for 1-2 seconds
   l. Let robot coast to stop
   m. Press RIGHT BUMPER (stop logging)
   ```

3. **Data Collection**
   - Log file saved to: `C:\Users\Public\Documents\FRC\Log Files\` on RoboRIO
   - File format: `*.hoot` (CTRE Signal Logger format)

4. **Analysis**
   - Use Phoenix Tuner X to convert `.hoot` → `.wpilog`
   - Open `.wpilog` in WPILib SysId tool
   - Use TalonFX `Position`, `Velocity`, `MotorVoltage` signals
   - Copy calculated gains to Constants.java

---

### Steer Characterization (Advanced)

⚠️ **CRITICAL**: CANcoder fusion must be disabled for accurate results

**Before Testing:**

**Option 1 - Phoenix Tuner X (Recommended)**
1. Open Phoenix Tuner X
2. Connect to robot
3. Select each CANcoder device (4 total)
4. Set update frequency to **0 Hz** for:
   - Position signal
   - Absolute Position signal
   - Velocity signal

**Option 2 - Code Change**
1. Edit `Constants.java`:
   ```java
   // Change this line:
   public static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;
   // To:
   public static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.SyncCANcoder;
   ```
2. Deploy code
3. Run tests
4. Change back to `FusedCANcoder` and redeploy

**Run Tests:**
```
1. Press LEFT BUMPER (start logging)
2. D-Pad UP (quasistatic forward) - hold 3-5 sec
3. D-Pad DOWN (quasistatic reverse) - hold 3-5 sec
4. D-Pad RIGHT (dynamic forward) - hold 1-2 sec
5. D-Pad LEFT (dynamic reverse) - hold 1-2 sec
6. Press RIGHT BUMPER (stop logging)
```

**After Testing:**
- Restore CANcoder update frequencies to 100 Hz (Option 1)
- OR restore `FusedCANcoder` in code (Option 2)
- Reboot robot

---

## CTRE Specific Tips

### Translation Tests
- Uses `SysIdSwerveTranslation` request
- Max voltage: 4V (prevents brownout)
- Ramp rate: 1 V/s (quasistatic)
- All modules point forward, drive motors spin together

### Steer Tests  
- Uses `SysIdSwerveSteerGains` request
- Max voltage: 7V (steering motors are lighter)
- **Must disable CANcoder fusion** during test
- Tests steering motor PID response without encoder feedback

### Rotation Tests
- Uses `SysIdSwerveRotation` request
- Units: rad/s (not volts)
- Max rate: π rad/s (~180°/s)
- Tests full robot spin dynamics

### TorqueCurrentFOC Mode (Advanced)
If using FOC current control instead of voltage:
- Treat current as voltage in SysId requests
- Avoid hitting 100% duty cycle
- May give more accurate low-speed characterization

---

## Common Issues

### "Robot doesn't move during test"
- Check battery voltage (should be >12V)
- Verify SignalLogger was started (LEFT BUMPER)
- Check Driver Station for errors

### "Robot curves instead of going straight"
- This is a calibration issue, not a test problem
- Continue with test - SysId analyzes motor response, not trajectory
- Fix wheel alignment after characterization

### "Accidentally ran same test twice"
- Stop SignalLogger (RIGHT BUMPER)
- Delete the log file
- Start over - SysId tool can't handle duplicate tests in one file

### "How do I know it's working?"
- Driver Station console shows "Signal Logger started/stopped"
- Robot should accelerate smoothly during quasistatic tests
- Check RoboRIO log directory for `.hoot` file after tests

---

## Data Analysis

1. **Convert Log**
   - Open Phoenix Tuner X
   - Tools → Log Extractor
   - Select `.hoot` file
   - Export to `.wpilog`

2. **Run SysId**
   - Open WPILib SysId tool (found in VS Code or Start Menu)
   - Load `.wpilog` file
   - Select "Talon FX" preset
   - Signals: Position, Velocity, MotorVoltage
   - Analyze → Calculate gains

3. **Apply Gains**
   - Copy kS, kV, kA from SysId output
   - Update `Constants.java`:
     ```java
     public static final class DriveGains {
       public static final double kS = 0.12; // From SysId
       public static final double kV = 2.85; // From SysId
       public static final double kA = 0.38; // From SysId
       ...
     }
     ```
   - Deploy and test

---

## Switching Back to Normal Operation

1. Comment out the SysId buttons section (add `/*` and `*/`)
2. Uncomment normal operation buttons (remove `/*` and `*/`)
3. Deploy code
4. Verify normal button functions work
