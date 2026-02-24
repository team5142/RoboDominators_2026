package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

// CTRE-based turret hardware implementation
public class TurretIOCTRE implements TurretIO {
  private final TalonFX flywheelFrontMotor;
  private final TalonFX flywheelBackMotor;
  private final TalonFX hoodMotor;
  private final TalonFX turretMotor;

  private final CANcoder hoodEncoder;
  private final CANcoder turretEncoder;

  private final DigitalInput singulatorBeamBreak;
  private final DigitalInput hoodBeamBreak;
  private final DigitalInput hallRight;
  private final DigitalInput hallLeft;

  private final DutyCycleOut flywheelDutyCycle = new DutyCycleOut(0.0);
  private final DutyCycleOut hoodDutyCycle = new DutyCycleOut(0.0);
  private final DutyCycleOut turretDutyCycle = new DutyCycleOut(0.0);

  public TurretIOCTRE() {
    flywheelFrontMotor = new TalonFX(Constants.Turret.FLYWHEEL_FRONT_MOTOR_ID);
    flywheelBackMotor = new TalonFX(Constants.Turret.FLYWHEEL_BACK_MOTOR_ID);

    // Cap flywheel spinup current to reduce brownout risk when drive is also accelerating.
    // 40A x 2 motors = 80A max flywheel draw. Raise to 60A if spinup feels too slow.
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // TODO (robot session): Add supply-side limit to protect breakers during long matches.
    // flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    // flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelFrontMotor.getConfigurator().apply(flywheelConfig);
    flywheelBackMotor.getConfigurator().apply(flywheelConfig);
    hoodMotor = new TalonFX(Constants.Turret.HOOD_MOTOR_ID);
    turretMotor = new TalonFX(Constants.Turret.TURRET_MOTOR_ID);

    hoodEncoder = new CANcoder(Constants.Turret.HOOD_CANCODER_ID);
    turretEncoder = new CANcoder(Constants.Turret.TURRET_CANCODER_ID);

    singulatorBeamBreak = new DigitalInput(Constants.Turret.SINGULATOR_BEAM_BREAK_DIO);
    hoodBeamBreak = new DigitalInput(Constants.Turret.HOOD_BEAM_BREAK_DIO);
    hallRight = new DigitalInput(Constants.Turret.HALL_SENSOR_RIGHT_DIO);
    hallLeft = new DigitalInput(Constants.Turret.HALL_SENSOR_LEFT_DIO);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.singulatorBeamBreakRaw = singulatorBeamBreak.get();
    inputs.hoodBeamBreakRaw = hoodBeamBreak.get();
    inputs.hallRightRaw = hallRight.get();
    inputs.hallLeftRaw = hallLeft.get();

    inputs.hoodAbsolutePositionRotations = hoodEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.turretAbsolutePositionRotations = turretEncoder.getAbsolutePosition().getValueAsDouble();
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
  public void setSingulatorPercent(double percent) {
    // Singulator motor uses REV controller and is skipped for now.
  }
}
