package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

// CTRE hardware implementation for the turret mechanism.
// All motor inversion is controlled via Constants.Turret.*_INVERTED booleans -
// flip those flags rather than changing wiring.
//
// TODO - FLYWHEEL CHECKLIST:
// [ ] 1. Both FLYWHEEL_FRONT (CAN 20) and FLYWHEEL_BACK (CAN 21) appear in TunerX.
// [ ] 2. Command a small positive output to each independently. Confirm both spin in
//        the correct shooting direction. Set FLYWHEEL_MOTOR_INVERTED = true if backwards.
// [ ] 3. Spin up to each SHOT_TABLE distance percent and check RPM via TunerX telemetry.
//        Adjust SHOT_TABLE_FLYWHEEL_FRONT_PCT and SHOT_TABLE_FLYWHEEL_BACK_PCT in Constants.
// [ ] 4. Fire a test ball at close range. If ball does not clear, raise flywheel percents.
//
// TODO - HOOD CHECKLIST:
// [ ] 1. HOOD motor (CAN 22) appears in TunerX.
// [ ] 2. Confirm DIO port HOOD_LIMIT_SWITCH_DIO (currently 2): go to Test mode, manually push the
//        hood to its bottom stop and confirm Turret/HoodLimitRaw toggles in AdvantageScope.
// [ ] 3. Command a small positive percent output. Confirm hood moves UP (away from bottom stop).
//        If it moves down toward the stop, set HOOD_MOTOR_INVERTED = true in Constants.
// [ ] 4. Run hoodHome() from TurretSubsystem. Hood should creep down, stop when limit switch fires,
//        and Turret/HoodHomed should become true. Encoder should read near 0.
// [ ] 5. Command hood to HOOD_SOFT_LIMIT_TOP_ROTATIONS. Observe where it stops physically.
//        Measure the actual angle and update HOOD_SOFT_LIMIT_TOP_ROTATIONS in Constants.
// [ ] 6. Update SHOT_TABLE_HOOD_ROTATIONS in Constants with measured motor encoder values at each
//        shot distance. Replace the placeholder degree-derived values.
// [ ] 7. Tune gravity feedforward (HOOD_KG): home the hood, command it to ~60 deg, let the P
//        controller settle, then zero the P output and find the duty cycle that holds position.
//        That value is the HOOD_KG seed. Add it to TurretSetpointGenerator as
//        kG * Math.cos(hoodAngleRadians) on top of the P term. Hood is heavy + belt-driven
//        so without this it will drift downward when the P error reaches zero.
// TODO: when a CANcoder is available for the hood, re-add it here with absolute position feedback.
//       Update TurretIOInputs, updateInputs(), zeroHoodEncoder(), and shot table accordingly.
//
// TODO - TURRET ROTATION CHECKLIST:
// [x] 1. TURRET motor (CAN 23) appears in TunerX.
// [ ] 2. Physically locate the two hard stops. The LEFT hard stop is home (encoder zero).
//        The hall sensor magnet should be mounted so it triggers just before the left hard stop.
//        Confirm at least 5-10 degrees of clearance between sensor fire and the physical stop.
//        If the magnet is on the CW side instead, remount the magnet near the CCW hard stop.
// [ ] 3. Confirm DIO port HALL_SENSOR_CCW_DIO (0):
//        go to Test mode, hold the magnet near the sensor and confirm
//        Turret/HallCCWRaw toggles in AdvantageScope.
// [ ] 4. Before commanding any motor output, manually move the turret to the center of its range.
//        This avoids slamming into a hard stop on first power-on.
// [ ] 5. Command a small positive percent output (TURRET_KP is 0.15 - very slow).
//        Positive should rotate LEFT (toward the left hard stop / hall sensor).
//        If it goes right, set TURRET_MOTOR_INVERTED = true in Constants.
// [ ] 6. Run home() from TurretSubsystem. Turret should creep left, stop when hall sensor fires,
//        and Turret/Homed should become true in AdvantageScope. Encoder should read near 0.
//        If it creeps right instead, recheck step 5.
// [ ] 7. Manually sweep the full rotation range end-to-end. Confirm the turret reaches both
//        hard stops without the hall sensor failing to trigger.
// [ ] 8. Home the turret, then slowly drive it to the right hard stop. Read the motor encoder
//        value from Turret/RotationDeg (divide by 360 to get rotations) and set
//        TURRET_SOFT_LIMIT_RIGHT_ROTATIONS in Constants to ~95% of that value.
public class TurretIOCTRE implements TurretIO {
  private final TalonFX flywheelFrontMotor;
  private final TalonFX flywheelBackMotor;
  private final TalonFX hoodMotor;
  private final TalonFX turretMotor;

  private final DigitalInput hoodLimitSwitch; // fires at bottom stop (85 deg = position 0)
  private final DigitalInput hallCCW;

  private final DutyCycleOut flywheelFrontDutyCycle = new DutyCycleOut(0.0);
  private final DutyCycleOut flywheelBackDutyCycle  = new DutyCycleOut(0.0);
  private final DutyCycleOut hoodDutyCycle          = new DutyCycleOut(0.0);
  private final DutyCycleOut turretDutyCycle        = new DutyCycleOut(0.0);
  private final VoltageOut   turretVoltageOut  = new VoltageOut(0.0);
  private final MotionMagicVoltage turretMotionMagic = new MotionMagicVoltage(0.0).withSlot(0);

  // Pre-subscribed sticky fault — latches true when motor boots while robot is enabled
  private com.ctre.phoenix6.StatusSignal<Boolean> bootDuringEnSignal;

  public TurretIOCTRE() {
    flywheelFrontMotor = new TalonFX(Constants.Turret.FLYWHEEL_FRONT_MOTOR_ID, new CANBus(Constants.Swerve.CAN_BUS_NAME));
    flywheelBackMotor  = new TalonFX(Constants.Turret.FLYWHEEL_BACK_MOTOR_ID,  new CANBus(Constants.Swerve.CAN_BUS_NAME));
    hoodMotor          = new TalonFX(Constants.Turret.HOOD_MOTOR_ID,           new CANBus(Constants.Swerve.CAN_BUS_NAME));

    // Flywheel motors: voltage control, brake mode, shared inversion flag (flip both if needed)
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted = Constants.Turret.FLYWHEEL_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    flywheelConfig.CurrentLimits.StatorCurrentLimit       = 40.0;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelFrontMotor.getConfigurator().apply(flywheelConfig);
    flywheelBackMotor.getConfigurator().apply(flywheelConfig);

    // Hood motor: duty cycle, brake mode
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = Constants.Turret.HOOD_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    hoodConfig.CurrentLimits.StatorCurrentLimit       = 20.0;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodMotor.getConfigurator().apply(hoodConfig);

    turretMotor = new TalonFX(Constants.Turret.TURRET_MOTOR_ID, new CANBus(Constants.Swerve.CAN_BUS_NAME));
    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    MotorOutputConfigs turretOutput = new MotorOutputConfigs();
    turretOutput.Inverted = Constants.Turret.TURRET_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turretOutput.NeutralMode = NeutralModeValue.Brake;
    turretConfig.MotorOutput = turretOutput;
    turretConfig.CurrentLimits.StatorCurrentLimit = 20.0;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Slot 0: MotionMagic PID + feedforward gains — tune kP in AdvantageScope
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kS = Constants.Turret.TURRET_KS;
    slot0.kV = Constants.Turret.TURRET_KV;
    slot0.kP = Constants.Turret.TURRET_KP;
    slot0.kD = Constants.Turret.TURRET_KD;
    turretConfig.Slot0 = slot0;

    // MotionMagic profile — safety speeds for commissioning; raise in Constants once confirmed
    MotionMagicConfigs mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = Constants.Turret.TURRET_CRUISE_VELOCITY_RPS;
    mm.MotionMagicAcceleration   = Constants.Turret.TURRET_ACCELERATION_RPS2;
    mm.MotionMagicJerk           = Constants.Turret.TURRET_JERK_RPS3;
    turretConfig.MotionMagic = mm;

    turretMotor.getConfigurator().apply(turretConfig);

    // Raise position and velocity signal rates for SysId data quality
    BaseStatusSignal.setUpdateFrequencyForAll(100,
        turretMotor.getPosition(), turretMotor.getVelocity());
    turretMotor.optimizeBusUtilization();

    // Subscribe sticky fault at 50Hz — checked each loop to detect mid-match motor reboot
    bootDuringEnSignal = turretMotor.getStickyFault_BootDuringEnable();
    bootDuringEnSignal.setUpdateFrequency(50);

    hoodLimitSwitch = new DigitalInput(Constants.Turret.HOOD_LIMIT_SWITCH_DIO);
    hallCCW         = new DigitalInput(Constants.Turret.HALL_SENSOR_CCW_DIO);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.hoodLimitSwitchRaw = !hoodLimitSwitch.get(); // active-low — true when switch is pressed
    inputs.hallCCWRaw         = !hallCCW.get();          // active-low — true when magnet is sensed

    if (hoodMotor != null)
      inputs.hoodMotorPositionRotations = hoodMotor.getPosition().getValueAsDouble();
    inputs.turretAbsolutePositionRotations = turretMotor.getPosition().getValueAsDouble();
    inputs.turretVelocityRps               = turretMotor.getVelocity().getValueAsDouble();
    inputs.turretMotorCurrentAmps          = turretMotor.getStatorCurrent().getValueAsDouble();
    if (flywheelFrontMotor != null)
      inputs.flywheelVelocityRpm = flywheelFrontMotor.getVelocity().getValueAsDouble() * 60.0;
    // Read and immediately clear the sticky fault so it fires for exactly one loop
    bootDuringEnSignal.refresh();
    inputs.turretBootDuringEn = bootDuringEnSignal.getValue();
    if (inputs.turretBootDuringEn) {
      turretMotor.clearStickyFault_BootDuringEnable();
    }
  }

  @Override
  public void setFlywheelPercent(double percent) {
    flywheelFrontMotor.setControl(flywheelFrontDutyCycle.withOutput(percent));
    flywheelBackMotor.setControl(flywheelBackDutyCycle.withOutput(percent));
  }

  @Override
  public void setFlywheelFrontPercent(double percent) {
    flywheelFrontMotor.setControl(flywheelFrontDutyCycle.withOutput(percent));
  }

  @Override
  public void setFlywheelBackPercent(double percent) {
    flywheelBackMotor.setControl(flywheelBackDutyCycle.withOutput(percent));
  }

  @Override
  public void setHoodPercent(double percent) {
    hoodMotor.setControl(hoodDutyCycle.withOutput(percent));
  }

  @Override
  public void setTurretPercent(double percent) {
    turretMotor.setControl(turretDutyCycle.withOutput(percent));
  }

  @Override
  public void setTurretVoltage(double volts) {
    turretMotor.setControl(turretVoltageOut.withOutput(volts));
  }

  @Override
  public void setTurretPosition(double motorRotations) {
    turretMotor.setControl(turretMotionMagic.withPosition(motorRotations));
  }

  // During SysId the normal 20A stator limit would clamp the motor before it can accelerate
  // freely, corrupting kA. Raise it to 60A for the duration of the test, then restore.
  @Override
  public void setSysIdActive(boolean active) {
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
    limits.StatorCurrentLimit       = active ? 60.0 : 20.0;
    limits.StatorCurrentLimitEnable = true;
    turretMotor.getConfigurator().apply(limits);
  }

  @Override
  public void zeroTurretEncoder() {
    // Set to the hall sensor offset so the encoder reads 0 at the CCW hard stop.
    // When mechanical relocates the sensor to the CCW stop, set TURRET_HALL_OFFSET_MOTOR_ROT = 0.
    turretMotor.setPosition(Constants.Turret.TURRET_HALL_OFFSET_MOTOR_ROT);
  }

  @Override
  public void restoreTurretEncoder(double motorRotations) {
    turretMotor.setPosition(motorRotations);
  }

  @Override
  public void zeroHoodEncoder() {
    if (hoodMotor == null) return;
    hoodMotor.setPosition(Constants.Turret.HOOD_HOME_ROTATIONS);
  }
}
