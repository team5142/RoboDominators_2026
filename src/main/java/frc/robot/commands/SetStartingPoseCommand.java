package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Sets the robot's starting pose for match initialization.
 * Resets gyro, pose estimator, and QuestNav to a known position.
 */
public class SetStartingPoseCommand extends Command {
  private final Pose2d targetPose;
  private final String positionName;
  private final GyroSubsystem gyro;
  private final DriveSubsystem drive;
  private final PoseEstimatorSubsystem poseEstimator;

  public SetStartingPoseCommand(
      Pose2d targetPose,
      String positionName,
      GyroSubsystem gyro,
      DriveSubsystem drive,
      PoseEstimatorSubsystem poseEstimator) {
    this.targetPose = targetPose;
    this.positionName = positionName;
    this.gyro = gyro;
    this.drive = drive;
    this.poseEstimator = poseEstimator;
    
    // No requirements - this is a one-shot initialization
  }

  @Override
  public void initialize() {
    // Get target angle
    double targetAngleDegrees = targetPose.getRotation().getDegrees();
    
    // Reset gyro to target angle
    gyro.setHeading(targetAngleDegrees);
    
    // Reset pose estimator
    poseEstimator.resetPose(
        targetPose,
        drive.getGyroRotation(),
        drive.getModulePositions());
    
    // CRITICAL: Reset QuestNav to match the new pose
    // Convert Pose2d to Pose3d (QuestNav uses 3D poses)
    Pose3d targetPose3d = new Pose3d(
        targetPose.getX(),
        targetPose.getY(),
        0.0, // Z height (assume on ground)
        new Rotation3d(0.0, 0.0, targetPose.getRotation().getRadians()));
    
    gyro.setQuestNavPose(targetPose3d);
    
    // Log to AdvantageKit
    Logger.recordOutput("StartingPose/Name", positionName);
    Logger.recordOutput("StartingPose/TargetPose", targetPose);
    Logger.recordOutput("StartingPose/TargetAngle", targetAngleDegrees);
    
    // SmartDashboard feedback
    SmartDashboard.putString("Starting Position", positionName);
    SmartDashboard.putString("Starting Pose", String.format(
        "X: %.2fm, Y: %.2fm, Θ: %.1f°",
        targetPose.getX(),
        targetPose.getY(),
        targetAngleDegrees));
    
    // Immediate verification
    Pose2d actualPose = poseEstimator.getEstimatedPose();
    double actualAngle = drive.getGyroRotation().getDegrees();
    
    SmartDashboard.putString("Actual Pose", String.format(
        "X: %.2fm, Y: %.2fm, Θ: %.1f°",
        actualPose.getX(),
        actualPose.getY(),
        actualAngle));
    
    System.out.println("=== STARTING POSE SET ===");
    System.out.println("Position: " + positionName);
    System.out.println("Target: " + targetPose);
    System.out.println("Actual: " + actualPose);
    System.out.println("QuestNav synced to: " + targetPose3d);
    System.out.println("========================");
  }

  @Override
  public boolean isFinished() {
    return true; // Instant command
  }
}