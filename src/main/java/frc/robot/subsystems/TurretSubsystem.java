package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

// Turret shooter mechanisms (flywheel, hood, turret rotation)
public class TurretSubsystem extends SubsystemBase {
  private final RobotState robotState;

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

  public TurretSubsystem(RobotState robotState) {
    this.robotState = robotState;

    flywheelFrontMotor = new TalonFX(Constants.Turret.FLYWHEEL_FRONT_MOTOR_ID);
    flywheelBackMotor = new TalonFX(Constants.Turret.FLYWHEEL_BACK_MOTOR_ID);
    hoodMotor = new TalonFX(Constants.Turret.HOOD_MOTOR_ID);
    turretMotor = new TalonFX(Constants.Turret.TURRET_MOTOR_ID);

    hoodEncoder = new CANcoder(Constants.Turret.HOOD_CANCODER_ID);
    turretEncoder = new CANcoder(Constants.Turret.TURRET_CANCODER_ID);

    singulatorBeamBreak = new DigitalInput(Constants.Turret.SINGULATOR_BEAM_BREAK_DIO);
    hoodBeamBreak = new DigitalInput(Constants.Turret.HOOD_BEAM_BREAK_DIO);
    hallRight = new DigitalInput(Constants.Turret.HALL_SENSOR_RIGHT_DIO);
    hallLeft = new DigitalInput(Constants.Turret.HALL_SENSOR_LEFT_DIO);

    SmartLogger.logConsole("Turret hardware initialized", "Turret");
  }

  public void setFlywheelPercent(double percent) {
    flywheelFrontMotor.setControl(flywheelDutyCycle.withOutput(percent));
    flywheelBackMotor.setControl(flywheelDutyCycle.withOutput(percent));
    robotState.setTurretFlywheelPercent(percent);
  }

  public void setHoodPercent(double percent) {
    hoodMotor.setControl(hoodDutyCycle.withOutput(percent));
    robotState.setTurretHoodPercent(percent);
  }

  public void setTurretPercent(double percent) {
    turretMotor.setControl(turretDutyCycle.withOutput(percent));
    robotState.setTurretRotationPercent(percent);
  }

  public void setSingulatorPercent(double percent) {
    // Singulator motor uses REV controller and is skipped for now.
    robotState.setTurretSingulatorPercent(percent);
  }

  public void stopAll() {
    setFlywheelPercent(0.0);
    setHoodPercent(0.0);
    setTurretPercent(0.0);
    setSingulatorPercent(0.0);
  }

  @Override
  public void periodic() {
    robotState.setTurretSingulatorBeamBreakRaw(singulatorBeamBreak.get());
    robotState.setTurretHoodBeamBreakRaw(hoodBeamBreak.get());
    robotState.setTurretHallRightRaw(hallRight.get());
    robotState.setTurretHallLeftRaw(hallLeft.get());

    robotState.setTurretHoodAbsolutePositionRotations(
        hoodEncoder.getAbsolutePosition().getValueAsDouble());
    robotState.setTurretRotationAbsolutePositionRotations(
        turretEncoder.getAbsolutePosition().getValueAsDouble());
  }
}
