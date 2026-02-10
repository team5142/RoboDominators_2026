package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

// Intake rollers and extension
public class IntakeSubsystem extends SubsystemBase {
  private final RobotState robotState;

  private final TalonFX extensionMotor;

  private final DutyCycleOut extensionDutyCycle = new DutyCycleOut(0.0);

  public IntakeSubsystem(RobotState robotState) {
    this.robotState = robotState;

    extensionMotor = new TalonFX(Constants.Intake.INTAKE_EXTENSION_MOTOR_ID);

    SmartLogger.logConsole("Intake motors initialized", "Intake");
  }

  public void setIntakePercent(double percent) {
    // Intake roller motor uses REV controller and is skipped for now.
    robotState.setIntakePercent(percent);
  }

  public void setExtensionPercent(double percent) {
    extensionMotor.setControl(extensionDutyCycle.withOutput(percent));
    robotState.setIntakeExtensionPercent(percent);
  }

  public void stopAll() {
    setIntakePercent(0.0);
    setExtensionPercent(0.0);
  }

  @Override
  public void periodic() {
    // No periodic actions yet
  }
}
