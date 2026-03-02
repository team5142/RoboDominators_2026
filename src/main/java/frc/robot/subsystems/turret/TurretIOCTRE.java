package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
//        If the magnet is on the right side instead, swap HALL_SENSOR_LEFT_DIO and
//        HALL_SENSOR_RIGHT_DIO in Constants, or remount the magnet.
// [ ] 3. Confirm DIO ports HALL_SENSOR_LEFT_DIO (0) and HALL_SENSOR_RIGHT_DIO (3):
//        go to Test mode, hold the magnet near each sensor and confirm
//        Turret/HallLeftRaw and Turret/HallRightRaw toggle in AdvantageScope.
//        Verify LEFT fires at the left physical stop and RIGHT fires at the right stop.
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
  private final DigitalInput hallRight;
  private final DigitalInput hallLeft;

  private final DutyCycleOut flywheelDutyCycle = new DutyCycleOut(0.0);
  private final DutyCycleOut hoodDutyCycle     = new DutyCycleOut(0.0);
  private final DutyCycleOut turretDutyCycle   = new DutyCycleOut(0.0);

  public TurretIOCTRE() {
    flywheelFrontMotor = new TalonFX(Constants.Turret.FLYWHEEL_FRONT_MOTOR_ID);
    flywheelBackMotor  = new TalonFX(Constants.Turret.FLYWHEEL_BACK_MOTOR_ID);

    // Cap flywheel spinup current to reduce brownout risk when drive is also accelerating.
    // X60 stall current is 483A — 40A limit keeps PDH headroom. Raise to 60A if spinup feels too slow.
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    MotorOutputConfigs flywheelOutput = new MotorOutputConfigs();
    flywheelOutput.Inverted = Constants.Turret.FLYWHEEL_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    flywheelConfig.MotorOutput = flywheelOutput;
    flywheelFrontMotor.getConfigurator().apply(flywheelConfig);
    flywheelBackMotor.getConfigurator().apply(flywheelConfig);

    hoodMotor   = new TalonFX(Constants.Turret.HOOD_MOTOR_ID);
    turretMotor = new TalonFX(Constants.Turret.TURRET_MOTOR_ID);

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    MotorOutputConfigs hoodOutput = new MotorOutputConfigs();
    hoodOutput.Inverted = Constants.Turret.HOOD_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput = hoodOutput;
    hoodConfig.CurrentLimits.StatorCurrentLimit = 20.0;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodMotor.getConfigurator().apply(hoodConfig);

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    MotorOutputConfigs turretOutput = new MotorOutputConfigs();
    turretOutput.Inverted = Constants.Turret.TURRET_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turretConfig.MotorOutput = turretOutput;
    turretConfig.CurrentLimits.StatorCurrentLimit = 20.0;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turretMotor.getConfigurator().apply(turretConfig);

    hoodLimitSwitch = new DigitalInput(Constants.Turret.HOOD_LIMIT_SWITCH_DIO);
    hallRight       = new DigitalInput(Constants.Turret.HALL_SENSOR_RIGHT_DIO);
    hallLeft        = new DigitalInput(Constants.Turret.HALL_SENSOR_LEFT_DIO);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.hoodLimitSwitchRaw = !hoodLimitSwitch.get(); // active-low — true when switch is pressed
    inputs.hallRightRaw       = hallRight.get();
    inputs.hallLeftRaw        = hallLeft.get();

    inputs.hoodMotorPositionRotations      = hoodMotor.getPosition().getValueAsDouble();
    inputs.turretAbsolutePositionRotations = turretMotor.getPosition().getValueAsDouble();
    inputs.turretMotorCurrentAmps          = turretMotor.getStatorCurrent().getValueAsDouble();
    // Velocity is in rotations/sec from CTRE — convert to RPM for dashboard readability
    inputs.flywheelVelocityRpm = flywheelFrontMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  @Override
  public void setFlywheelPercent(double percent) {
    flywheelFrontMotor.setControl(flywheelDutyCycle.withOutput(percent));
    flywheelBackMotor.setControl(flywheelDutyCycle.withOutput(percent));
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
  public void zeroTurretEncoder() {
    turretMotor.setPosition(0.0);
  }

  @Override
  public void zeroHoodEncoder() {
    hoodMotor.setPosition(Constants.Turret.HOOD_HOME_ROTATIONS);
  }
}
