package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

// Climber pull and rotation motors.
// Pull motor retracts the climbing hook; rotation motor pivots the arm.
//
// TODO - COMMISSIONING CHECKLIST (complete in order before enabling in RobotContainer):
// [ ] 1. Confirm CAN IDs: PULL_MOTOR_ID (50) and ROTATION_MOTOR_ID (51) in TunerX.
// [ ] 2. Check pull motor direction: hold left trigger. If arm extends instead of pulling, set
//        PULL_MOTOR_INVERTED = true in Constants.
// [ ] 3. Check rotation motor direction: hold left bumper up. Set ROTATION_MOTOR_INVERTED if wrong.
// [ ] 4. Tune PULL_SPEED and ROTATION_SPEED in Constants - start at 0.15, increase slowly.
// [ ] 5. Tune PULL_STATOR_LIMIT_AMPS - increase if pull stalls under full climbing load.
public class ClimberSubsystem extends SubsystemBase {
  private final RobotState robotState;

  private final TalonFX pullMotor;
  private final TalonFX rotationMotor;

  private final DutyCycleOut pullDutyCycle = new DutyCycleOut(0.0);
  private final DutyCycleOut rotationDutyCycle = new DutyCycleOut(0.0);

  public ClimberSubsystem(RobotState robotState) {
    this.robotState = robotState;

    pullMotor     = new TalonFX(Constants.Climber.PULL_MOTOR_ID);
    rotationMotor = new TalonFX(Constants.Climber.ROTATION_MOTOR_ID);

    TalonFXConfiguration pullConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs pullLimits = pullConfig.CurrentLimits;
    pullLimits.StatorCurrentLimit       = Constants.Climber.PULL_STATOR_LIMIT_AMPS;
    pullLimits.StatorCurrentLimitEnable = true;
    pullLimits.SupplyCurrentLimit       = Constants.Climber.PULL_SUPPLY_LIMIT_AMPS;
    pullLimits.SupplyCurrentLimitEnable = true;
    pullConfig.MotorOutput.Inverted = Constants.Climber.PULL_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    pullMotor.getConfigurator().apply(pullConfig);

    TalonFXConfiguration rotConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs rotLimits = rotConfig.CurrentLimits;
    rotLimits.StatorCurrentLimit       = Constants.Climber.ROTATION_STATOR_LIMIT_AMPS;
    rotLimits.StatorCurrentLimitEnable = true;
    rotLimits.SupplyCurrentLimit       = Constants.Climber.ROTATION_SUPPLY_LIMIT_AMPS;
    rotLimits.SupplyCurrentLimitEnable = true;
    rotConfig.MotorOutput.Inverted = Constants.Climber.ROTATION_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    rotationMotor.getConfigurator().apply(rotConfig);

    SmartLogger.logConsole("Climber hardware initialized", "Climber");
  }

  public void setPullPercent(double percent) {
    pullMotor.setControl(pullDutyCycle.withOutput(percent));
    robotState.setClimberPullPercent(percent);
  }

  public void setRotationPercent(double percent) {
    rotationMotor.setControl(rotationDutyCycle.withOutput(percent));
    robotState.setClimberRotationPercent(percent);
  }

  public void stopAll() {
    setPullPercent(0.0);
    setRotationPercent(0.0);
  }

  @Override
  public void periodic() {
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Climber/PullPercent", robotState.getClimberPullPercent());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Climber/RotationPercent", robotState.getClimberRotationPercent());
  }
}
