package frc.robot.commands.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.SmartLogger;

// Logs current robot pose as copy-paste ready Java code for Constants.java.
// Useful for recording positions during practice. Needs a button binding in RobotContainer to activate.
public class LogCurrentPoseCommand extends InstantCommand {

  private final PoseEstimatorSubsystem poseEstimator;
  private final String poseName;

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

    String code = String.format(
        "public static final Pose2d %s = new Pose2d(\n" +
        "    %.2f, %.2f, Rotation2d.fromDegrees(%.1f));",
        poseName,
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());

    SmartLogger.logConsole("COPY THIS TO Constants.StartingPositions:\n\n" + code, "Save Pose: " + poseName, 15);
    SmartLogger.logConsole(SmartLogger.formatPose(pose), "Save Pose: " + poseName);
    SmartLogger.logReplay("SavedPose/" + poseName, pose);
  }
}
