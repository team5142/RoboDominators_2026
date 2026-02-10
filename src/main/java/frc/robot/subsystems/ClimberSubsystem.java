package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

// Climber pull and rotation motors
public class ClimberSubsystem extends SubsystemBase {
  private final RobotState robotState;

  private final TalonFX pullMotor;
  private final TalonFX rotationMotor;

  private final DutyCycleOut pullDutyCycle = new DutyCycleOut(0.0);
  private final DutyCycleOut rotationDutyCycle = new DutyCycleOut(0.0);

  public ClimberSubsystem(RobotState robotState) {
    this.robotState = robotState;

    pullMotor = new TalonFX(Constants.Climber.PULL_MOTOR_ID);
    rotationMotor = new TalonFX(Constants.Climber.ROTATION_MOTOR_ID);

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
    // No periodic actions yet
  }
}
