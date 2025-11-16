package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Vision.*;
import static frc.robot.Constants.StartingPositions.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class PoseEstimatorSubsystem extends SubsystemBase {
  public enum InitializationState {
    WAITING_FOR_VISION,     // Haven't seen any vision yet
    VISION_INITIALIZED,     // Got first vision update
    FALLBACK_USED           // Timeout - using default position
  }

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final RobotState robotState;
  private final DriveSubsystem driveSubsystem;

  private InitializationState initState = InitializationState.WAITING_FOR_VISION;
  private final Timer visionWaitTimer = new Timer();
  private static final double VISION_TIMEOUT_SECONDS = 7.0; // Match old code

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

    // Start timer for vision initialization
    visionWaitTimer.start();
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, int tagCount) {
    // First vision update - we're initialized!
    if (initState == InitializationState.WAITING_FOR_VISION) {
      initState = InitializationState.VISION_INITIALIZED;
      Logger.recordOutput("PoseEstimator/InitializedViaVision", true);
      System.out.println("=== POSE INITIALIZED VIA VISION ===");
      System.out.println("First vision pose: " + visionPose);
      System.out.println("Tags seen: " + tagCount);
      System.out.println("Time to initialize: " + visionWaitTimer.get() + "s");
    }

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
    
    // Manual reset counts as initialized
    if (initState == InitializationState.WAITING_FOR_VISION) {
      initState = InitializationState.VISION_INITIALIZED;
      Logger.recordOutput("PoseEstimator/InitializedManually", true);
    }
  }

  public InitializationState getInitializationState() {
    return initState;
  }

  public boolean isInitialized() {
    return initState != InitializationState.WAITING_FOR_VISION;
  }

  @Override
  public void periodic() {
    // Update with latest wheel odometry
    poseEstimator.update(driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    
    // Check for vision timeout
    if (initState == InitializationState.WAITING_FOR_VISION && 
        visionWaitTimer.hasElapsed(VISION_TIMEOUT_SECONDS)) {
      
      // Timeout - use fallback starting position
      initState = InitializationState.FALLBACK_USED;
      
      // Use Blue Reef Tag 17 as default (could make this smarter based on alliance)
      Pose2d fallbackPose = BLUE_REEF_TAG_17;
      resetPose(fallbackPose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
      
      System.err.println("=== VISION TIMEOUT - USING FALLBACK POSE ===");
      System.err.println("No vision detected after " + VISION_TIMEOUT_SECONDS + "s");
      System.err.println("Fallback pose: " + fallbackPose);
      
      Logger.recordOutput("PoseEstimator/FallbackUsed", true);
      Logger.recordOutput("PoseEstimator/FallbackPose", fallbackPose);
    }
    
    // Push to RobotState
    Pose2d currentPose = getEstimatedPose();
    robotState.setRobotPose(currentPose);
    
    // Logging
    Logger.recordOutput("PoseEstimator/EstimatedPose", currentPose);
    Logger.recordOutput("PoseEstimator/InitState", initState.toString());
    Logger.recordOutput("PoseEstimator/WaitingForVisionTime", 
        initState == InitializationState.WAITING_FOR_VISION ? visionWaitTimer.get() : 0.0);
  }
}
