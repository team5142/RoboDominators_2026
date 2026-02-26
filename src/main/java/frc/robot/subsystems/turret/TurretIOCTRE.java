package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
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
// [ ] 1. HOOD motor (CAN 22) and HOOD_CANCODER (CAN 25) appear in TunerX.
// [ ] 2. Manually move the hood and confirm CANcoder position changes in TunerX.
//        Verify the reading is in rotations (not degrees) and increases in the expected direction.
// [ ] 3. Command a small positive percent output to the hood motor (HOOD_KP is 0.15 - very slow).
//        Confirm it moves in the expected direction. Set HOOD_MOTOR_INVERTED = true if backwards.
// [ ] 4. Move hood to a known physical stop and record the CANcoder value.
//        Use that to calibrate SHOT_TABLE_HOOD_ROTATIONS placeholder values in Constants.
//
// TODO - TURRET ROTATION CHECKLIST:
// [ ] 1. TURRET motor (CAN 23) and TURRET_CANCODER (CAN 26) appear in TunerX.
// [ ] 2. Physically locate the two hard stops. The LEFT hard stop is home (encoder zero).
//        The hall sensor magnet should be mounted so it triggers just before the left hard stop.
//        Confirm at least 5-10 degrees of clearance between sensor fire and the physical stop.
//        If the magnet is on the right side instead, swap HALL_SENSOR_LEFT_DIO and
//        HALL_SENSOR_RIGHT_DIO in Constants, or remount the magnet.
// [ ] 3. Confirm DIO ports HALL_SENSOR_LEFT_DIO (4) and HALL_SENSOR_RIGHT_DIO (3):
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
//        hard stops without the hall sensor failing to trigger, and that TURRET_CANCODER
//        reads span the expected range (e.g. 0 to ~0.X rotations).
public class TurretIOCTRE implements TurretIO {
  private final TalonFX flywheelFrontMotor;
  private final TalonFX flywheelBackMotor;
  private final TalonFX hoodMotor;
  private final TalonFX turretMotor;

  private final CANcoder hoodEncoder;
  private final CANcoder turretEncoder;

  private final DigitalInput hoodBeamBreak;
  private final DigitalInput hallRight;
  private final DigitalInput hallLeft;

  private final DutyCycleOut flywheelDutyCycle = new DutyCycleOut(0.0);
  private final DutyCycleOut hoodDutyCycle     = new DutyCycleOut(0.0);
  private final DutyCycleOut turretDutyCycle   = new DutyCycleOut(0.0);

  public TurretIOCTRE() {
    flywheelFrontMotor = new TalonFX(Constants.Turret.FLYWHEEL_FRONT_MOTOR_ID);
    flywheelBackMotor  = new TalonFX(Constants.Turret.FLYWHEEL_BACK_MOTOR_ID);

    // Cap flywheel spinup current to reduce brownout risk when drive is also accelerating.
    // 40A x 2 motors = 80A max flywheel draw. Raise to 60A if spinup feels too slow.
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

    hoodEncoder   = new CANcoder(Constants.Turret.HOOD_CANCODER_ID);
    turretEncoder = new CANcoder(Constants.Turret.TURRET_CANCODER_ID);

    hoodBeamBreak = new DigitalInput(Constants.Turret.HOOD_BEAM_BREAK_DIO);
    hallRight     = new DigitalInput(Constants.Turret.HALL_SENSOR_RIGHT_DIO);
    hallLeft      = new DigitalInput(Constants.Turret.HALL_SENSOR_LEFT_DIO);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.hoodBeamBreakRaw = hoodBeamBreak.get();
    inputs.hallRightRaw     = hallRight.get();
    inputs.hallLeftRaw      = hallLeft.get();

    inputs.hoodAbsolutePositionRotations   = hoodEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.turretAbsolutePositionRotations = turretEncoder.getAbsolutePosition().getValueAsDouble();
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
}
