package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final TagVisionSubsystem tagVisionSubsystem;

  public LEDSubsystem(RobotState robotState, TagVisionSubsystem tagVisionSubsystem) {
    this.robotState = robotState;
    this.tagVisionSubsystem = tagVisionSubsystem;
  }

  @Override
  public void periodic() {
    // For now, we just log what we'd like to communicate via LEDs.
    Logger.recordOutput("LED/Mode", robotState.getMode().toString());
    Logger.recordOutput("LED/AssistMode", robotState.getAssistMode().toString());
    Logger.recordOutput("LED/TagVisionHasPose", tagVisionSubsystem.hasRecentTagPose());
  }
}
