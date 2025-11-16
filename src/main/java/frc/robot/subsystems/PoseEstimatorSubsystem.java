package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class PoseEstimatorSubsystem extends SubsystemBase {
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final RobotState robotState;
  private final DriveSubsystem driveSubsystem;

  public PoseEstimatorSubsystem(
      DriveSubsystem driveSubsystem,
      RobotState robotState) {
    this.driveSubsystem = driveSubsystem;
    this.kinematics = driveSubsystem.getKinematics();
    this.robotState = robotState;

    // Create pose estimator with initial state
    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        driveSubsystem.getGyroRotation(),
        driveSubsystem.getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(ODOMETRY_STD_DEVS[0], ODOMETRY_STD_DEVS[1], ODOMETRY_STD_DEVS[2]),
        VecBuilder.fill(VISION_STD_DEVS_SINGLE_TAG[0], VISION_STD_DEVS_SINGLE_TAG[1], VISION_STD_DEVS_SINGLE_TAG[2]));
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, int tagCount) {
    // Adjust standard deviations based on number of tags
    double[] stdDevs = tagCount >= MIN_TAG_COUNT_FOR_MULTI 
        ? VISION_STD_DEVS_MULTI_TAG 
        : VISION_STD_DEVS_SINGLE_TAG;

    poseEstimator.addVisionMeasurement(
        visionPose,
        timestampSeconds,
        VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]));

    Logger.recordOutput("PoseEstimator/VisionUpdate", visionPose);
    Logger.recordOutput("PoseEstimator/VisionTagCount", tagCount);
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
  }

  @Override
  public void periodic() {
    // Update with latest wheel odometry
    poseEstimator.update(driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    
    // Push to RobotState
    Pose2d currentPose = getEstimatedPose();
    robotState.setRobotPose(currentPose);
    
    Logger.recordOutput("PoseEstimator/EstimatedPose", currentPose);
  }
}
