package frc.robot.commands.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.QuestNavSubsystem; // NEW
import frc.robot.subsystems.PoseEstimatorSubsystem;
import org.littletonrobotics.junction.Logger;

// Sets robot's starting pose for match initialization - resets all localization systems
// Critical: Must sync Pigeon2, PoseEstimator, AND QuestNav or they will fight each other
// Use this before autonomous or after placing robot at known position
public class SetStartingPoseCommand extends Command {
  private final Pose2d targetPose; // Known starting position on field
  private final String positionName; // Human-readable name (e.g. "Blue Reef Tag 17")
  private final GyroSubsystem gyro; // Manages Pigeon2 + QuestNav
  private final QuestNavSubsystem questNav; // NEW
  private final DriveSubsystem drive; // Provides module positions
  private final PoseEstimatorSubsystem poseEstimator; // Kalman filter

  public SetStartingPoseCommand(
      Pose2d targetPose,
      String positionName,
      GyroSubsystem gyro,
      QuestNavSubsystem questNav, // NEW parameter
      DriveSubsystem drive,
      PoseEstimatorSubsystem poseEstimator) {
    this.targetPose = targetPose;
    this.positionName = positionName;
    this.gyro = gyro;
    this.questNav = questNav; // NEW
    this.drive = drive;
    this.poseEstimator = poseEstimator;
    
    // No requirements - instant command that just sets state
  }

  @Override
  public void initialize() {
    double targetAngleDegrees = targetPose.getRotation().getDegrees();
    
    // Step 1: Reset Pigeon2 gyro to target heading
    gyro.setHeading(targetAngleDegrees);
    
    // Step 2: Reset pose estimator (Kalman filter)
    poseEstimator.resetPose(
        targetPose,
        drive.getGyroRotation(), // Use newly-set gyro angle
        drive.getModulePositions()); // Current wheel positions
    
    // Step 3: CRITICAL - Reset QuestNav to match
    // (PoseEstimator.resetPose() already calls questNav.initialize(), but we log here for clarity)
    
    // Log to AdvantageKit for analysis
    Logger.recordOutput("StartingPose/Name", positionName);
    Logger.recordOutput("StartingPose/TargetPose", targetPose);
    Logger.recordOutput("StartingPose/TargetAngle", targetAngleDegrees);
    
    // SmartDashboard feedback for driver/operator
    SmartDashboard.putString("Starting Position", positionName);
    SmartDashboard.putString("Starting Pose", String.format(
        "X: %.2fm, Y: %.2fm, Θ: %.1f°",
        targetPose.getX(),
        targetPose.getY(),
        targetAngleDegrees));
    
    // Verify reset worked (debug)
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
    System.out.println("QuestNav synced via PoseEstimator.resetPose()");
    System.out.println("========================");
  }

  @Override
  public boolean isFinished() {
    return true; // Instant command - finishes immediately
  }
}