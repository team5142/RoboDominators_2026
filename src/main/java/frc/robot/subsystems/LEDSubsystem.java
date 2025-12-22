package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

// LED subsystem - currently a stub (no physical LEDs installed)
// TODO 2026: Add LED hardware and patterns for robot state visualization
public class LEDSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final TagVisionSubsystem tagVisionSubsystem;

  public LEDSubsystem(RobotState robotState, TagVisionSubsystem tagVisionSubsystem) {
    this.robotState = robotState;
    this.tagVisionSubsystem = tagVisionSubsystem;
  }

  @Override
  public void periodic() {
    // Log what we'd display on LEDs (for future reference)
    SmartLogger.logReplay("LED/Mode", robotState.getMode().toString());
    SmartLogger.logReplay("LED/TagVisionHasPose", tagVisionSubsystem.hasRecentTagPose());
  }
}