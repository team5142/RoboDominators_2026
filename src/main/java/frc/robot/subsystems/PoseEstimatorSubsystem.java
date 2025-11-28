package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Vision.*;
import static frc.robot.Constants.StartingPositions.*;
import static frc.robot.Constants.QuestNav.*;

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

// Kalman filter-based pose estimator - fuses wheel odometry, vision, and QuestNav SLAM
// Provides best estimate of robot position on field by weighting all sensor sources
// More trusted sources (multi-tag vision) pull estimate more than less trusted (wheel odometry)
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

  // NEW: Temporary flag to disable ALL vision updates (for testing/calibration)
  private static final boolean VISION_UPDATES_ENABLED = true; // RE-ENABLED: Test multi-camera fusion!

  // NEW: Camera quality multipliers (higher = more trust)
  private static final double LIMELIGHT_QUALITY = 1.0;    // Highest quality (baseline)
  private static final double RR_TAG_PV_QUALITY = 1.5;    // Medium quality (50% less trust than Limelight)
  private static final double RL_TAG_PV_QUALITY = 2.0;    // Lowest quality (2x less trust than Limelight)

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
    addVisionMeasurement(visionPose, timestampSeconds, tagCount, "unknown");
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, int tagCount, String cameraName) {
    // TESTING MODE: Ignore all vision updates
    if (!VISION_UPDATES_ENABLED) {
      Logger.recordOutput("PoseEstimator/VisionUpdatesDisabled", true);
      Logger.recordOutput("PoseEstimator/VisionMeasurementIgnored", visionPose);
      Logger.recordOutput("PoseEstimator/VisionTagCountIgnored", tagCount);
      return; // Don't add vision measurement
    }
    
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

    // Base standard deviations based on number of tags
    double[] baseStdDevs = tagCount >= MIN_TAG_COUNT_FOR_MULTI 
        ? VISION_STD_DEVS_MULTI_TAG  // [0.1, 0.1, 0.2] - Multi-tag baseline
        : VISION_STD_DEVS_SINGLE_TAG; // [0.3, 0.3, 0.5] - Single-tag baseline

    // NEW: Apply camera-specific quality multiplier
    double qualityMultiplier = getCameraQualityMultiplier(cameraName);
    double[] adjustedStdDevs = new double[] {
        baseStdDevs[0] * qualityMultiplier, // X std dev
        baseStdDevs[1] * qualityMultiplier, // Y std dev
        baseStdDevs[2] * qualityMultiplier  // Theta std dev
    };

    poseEstimator.addVisionMeasurement(
        visionPose,
        timestampSeconds,
        VecBuilder.fill(adjustedStdDevs[0], adjustedStdDevs[1], adjustedStdDevs[2]));

    visionUpdateCount++;
    
    Logger.recordOutput("PoseEstimator/VisionUpdate", visionPose);
    Logger.recordOutput("PoseEstimator/VisionTagCount", tagCount);
    Logger.recordOutput("PoseEstimator/VisionCamera", cameraName);
    Logger.recordOutput("PoseEstimator/VisionQualityMultiplier", qualityMultiplier);
    Logger.recordOutput("PoseEstimator/VisionStdDevX", adjustedStdDevs[0]);
    Logger.recordOutput("PoseEstimator/TotalVisionUpdates", visionUpdateCount);
  }

  /**
   * Get camera quality multiplier (higher = less trust)
   * Limelight = 1.0 (best), RRTagPV = 1.5 (medium), RLTagPV = 2.0 (weakest)
   */
  private double getCameraQualityMultiplier(String cameraName) {
    switch (cameraName) {
      case "limelight-front":
        return LIMELIGHT_QUALITY; // Highest quality
      case "RRTagPV":
        return RR_TAG_PV_QUALITY; // Medium quality
      case "RLTagPV":
        return RL_TAG_PV_QUALITY; // Lowest quality
      default:
        return 2.0; // Unknown camera = low trust
    }
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
    
    // CRITICAL: Sync QuestNav to match the new pose
    // This prevents QuestNav from immediately "correcting" the pose back to where it was
    syncQuestNavToPose(pose);
    
    // Manual reset counts as initialized
    if (initState == InitializationState.WAITING_FOR_VISION) {
      initState = InitializationState.VISION_INITIALIZED;
      Logger.recordOutput("PoseEstimator/InitializedManually", true);
    }
    
    Logger.recordOutput("PoseEstimator/PoseReset", pose);
    Logger.recordOutput("PoseEstimator/QuestNavSyncedOnReset", true);
  }

  public InitializationState getInitializationState() {
    return initState;
  }

  public boolean isInitialized() {
    return initState != InitializationState.WAITING_FOR_VISION;
  }

  @Override
  public void periodic() {
    // Skip ALL vision updates during SysID
    if (robotState.isSysIdMode()) {
      Logger.recordOutput("PoseEstimator/SysIdMode", true);
      return; // Don't update pose estimator at all
    }
    
    // Update with latest wheel odometry (ALWAYS - this is the baseline)
    poseEstimator.update(driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    
    // TESTING MODE: Log that vision is disabled
    if (!VISION_UPDATES_ENABLED) {
      Logger.recordOutput("PoseEstimator/VisionUpdatesDisabled", true);
    }
    
    // CHANGED: Only use QuestNav position during TELEOP, not AUTO
    // QuestNav gets confused when we "teleport" it to a new pose at auto start
    if (VISION_UPDATES_ENABLED && robotState.getMode() != RobotState.Mode.AUTO && gyroSubsystem.hasNewQuestNavPose()) {
      Pose2d questNavPose = gyroSubsystem.getQuestNavPose2d();
      
      if (questNavPose != null) {
        poseEstimator.addVisionMeasurement(
          questNavPose,
          Timer.getFPGATimestamp(),
          VecBuilder.fill(
            QUESTNAV_STD_DEVS[0],
            QUESTNAV_STD_DEVS[1],
            QUESTNAV_STD_DEVS[2]
          )
        );
        
        Logger.recordOutput("PoseEstimator/QuestNavUpdate", questNavPose);
        Logger.recordOutput("PoseEstimator/QuestNavUsed", true);
      }
    } else {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
    }
    
    // FIXED: Only use fallback pose during TELEOP mode, not auto or disabled
    // Auto should use PathPlanner's starting pose, not our fallback
    if (initState == InitializationState.WAITING_FOR_VISION && 
        visionWaitTimer.hasElapsed(VISION_TIMEOUT_SECONDS) &&
        robotState.getMode() == RobotState.Mode.TELEOP) { // CHANGED: TELEOP only
      
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
