package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Vision.*;
import static frc.robot.Constants.StartingPositions.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
  private final GyroSubsystem gyroSubsystem; // NEW: Need this to sync QuestNav

  private InitializationState initState = InitializationState.WAITING_FOR_VISION;
  private final Timer visionWaitTimer = new Timer();
  private static final double VISION_TIMEOUT_SECONDS = 7.0; // Match old code

  private Pose2d lastPose = new Pose2d(); // Track last pose to detect vision corrections
  private int visionUpdateCount = 0; // Count total vision updates

  // NEW: Throttle QuestNav syncing
  private double lastQuestNavSyncTime = 0.0;
  private static final double QUESTNAV_SYNC_INTERVAL_SECONDS = 1.0; // Only sync once per second

  public PoseEstimatorSubsystem(
      DriveSubsystem driveSubsystem,
      RobotState robotState,
      GyroSubsystem gyroSubsystem) { // NEW: Add gyro parameter
    this.driveSubsystem = driveSubsystem;
    this.robotState = robotState;
    this.gyroSubsystem = gyroSubsystem; // NEW: Store reference

    this.kinematics = driveSubsystem.getKinematics();

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
      
      // Sync QuestNav to initial vision pose (no throttling on first init)
      syncQuestNavToPose(visionPose);
      lastQuestNavSyncTime = Timer.getFPGATimestamp();
    }

    // Calculate correction magnitude (how much vision is adjusting odometry)
    Pose2d currentOdometry = poseEstimator.getEstimatedPosition();
    double correctionDistance = currentOdometry.getTranslation()
        .getDistance(visionPose.getTranslation());
    double correctionAngle = Math.abs(
        currentOdometry.getRotation().minus(visionPose.getRotation()).getDegrees());
    
    // Log correction magnitude
    Logger.recordOutput("PoseEstimator/VisionCorrectionDistance", correctionDistance);
    Logger.recordOutput("PoseEstimator/VisionCorrectionAngle", correctionAngle);
    
    // THROTTLED: Only sync QuestNav if significant correction AND enough time has passed
    double currentTime = Timer.getFPGATimestamp();
    boolean shouldSync = (correctionDistance > 0.05 || correctionAngle > 2.0) &&
                         (currentTime - lastQuestNavSyncTime >= QUESTNAV_SYNC_INTERVAL_SECONDS);
    
    if (shouldSync) {
      Logger.recordOutput("PoseEstimator/SignificantVisionCorrection", true);
      
      // Sync QuestNav to vision-corrected pose (silent)
      syncQuestNavToPose(visionPose);
      lastQuestNavSyncTime = currentTime;
    }

    // Adjust standard deviations based on number of tags
    double[] stdDevs = tagCount >= MIN_TAG_COUNT_FOR_MULTI 
        ? VISION_STD_DEVS_MULTI_TAG 
        : VISION_STD_DEVS_SINGLE_TAG;

    poseEstimator.addVisionMeasurement(
        visionPose,
        timestampSeconds,
        VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]));

    visionUpdateCount++;
    
    Logger.recordOutput("PoseEstimator/VisionUpdate", visionPose);
    Logger.recordOutput("PoseEstimator/VisionTagCount", tagCount);
    Logger.recordOutput("PoseEstimator/TotalVisionUpdates", visionUpdateCount);
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
      
      initState = InitializationState.FALLBACK_USED;
      
      Pose2d fallbackPose = BLUE_REEF_TAG_17;
      resetPose(fallbackPose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
      
      syncQuestNavToPose(fallbackPose);
      
      System.err.println("=== VISION TIMEOUT - USING FALLBACK POSE ===");
      System.err.println("No vision detected after " + VISION_TIMEOUT_SECONDS + "s");
      System.err.println("Fallback pose: " + fallbackPose);
      System.err.println("QuestNav synced to fallback");
      
      Logger.recordOutput("PoseEstimator/FallbackUsed", true);
      Logger.recordOutput("PoseEstimator/FallbackPose", fallbackPose);
    }
    
    // Push to RobotState
    Pose2d currentPose = getEstimatedPose();
    robotState.setRobotPose(currentPose);
    
    // Detect pose changes (odometry drift or vision corrections)
    double poseChange = currentPose.getTranslation().getDistance(lastPose.getTranslation());
    Logger.recordOutput("PoseEstimator/PoseChangeMeters", poseChange);
    lastPose = currentPose;
    
    // Logging
    Logger.recordOutput("PoseEstimator/EstimatedPose", currentPose);
    Logger.recordOutput("PoseEstimator/InitState", initState.toString());
    Logger.recordOutput("PoseEstimator/WaitingForVisionTime", 
        initState == InitializationState.WAITING_FOR_VISION ? visionWaitTimer.get() : 0.0);
    Logger.recordOutput("PoseEstimator/TotalVisionUpdates", visionUpdateCount);
  }
  
  /**
   * Sync QuestNav to match the robot's current pose estimate
   */
  private void syncQuestNavToPose(Pose2d pose) {
    Pose3d pose3d = new Pose3d(
        pose.getX(),
        pose.getY(),
        0.0, // Z = 0 (on ground)
        new Rotation3d(0.0, 0.0, pose.getRotation().getRadians()));
    
    gyroSubsystem.setQuestNavPose(pose3d);
    Logger.recordOutput("PoseEstimator/QuestNavSynced", true);
    Logger.recordOutput("PoseEstimator/QuestNavSyncPose", pose);
  }
}
