package frc.robot.commands.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.SmartLogger;

// Logs current robot pose as copy-paste ready Java code for Constants.java
// Useful for recording scoring positions during testing
public class LogCurrentPoseCommand extends InstantCommand {
  
  private final PoseEstimatorSubsystem poseEstimator;
  private final String poseName;
  
  // Currently tied to holding down both Left and Right trigger at the same time.
  public LogCurrentPoseCommand(PoseEstimatorSubsystem poseEstimator, String poseName) {
    this.poseEstimator = poseEstimator;
    this.poseName = poseName;
  }
  
  public LogCurrentPoseCommand(PoseEstimatorSubsystem poseEstimator) {
    this(poseEstimator, "CUSTOM_POSITION");
  }
  
  @Override
  public void initialize() {
    Pose2d pose = poseEstimator.getEstimatedPose();
    
    // Generate copy-paste ready Java code
    String code = String.format(
        "public static final Pose2d %s = new Pose2d(\n" +
        "    %.2f, %.2f, Rotation2d.fromDegrees(%.1f));",
        poseName,
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
    
    String fullMsg = "COPY THIS TO Constants.StartingPositions:\n\n" + code;
    SmartLogger.logConsole(fullMsg, "Save Pose: " + poseName, 15);
    
    String summary = String.format("Position: %.2fm, %.2fm, %.1f°",
        pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    SmartLogger.logConsole(summary);
    
    SmartLogger.logReplay("SavedPose/" + poseName, pose);
  }
}
