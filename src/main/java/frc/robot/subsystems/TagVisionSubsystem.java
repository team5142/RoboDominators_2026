package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightCamera;
import frc.robot.subsystems.vision.PhotonVisionCamera;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionCamera.VisionResult;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

// AprilTag vision processing - provides current tag detection status to PoseEstimator
// Exposes hasMultiTagDetection() and hasSingleTagDetection() for initialization checks
public class TagVisionSubsystem extends SubsystemBase {
  
  // ===== FEATURE TOGGLE: LIMELIGHT VISION =====
  // Set to false to disable Limelight (QuestNav-only mode)
  // Set to true to re-enable Limelight fusion
  private static final boolean LIMELIGHT_ENABLED = false; // CHANGED: Disabled for now
  // ============================================
  
  private final PoseEstimatorSubsystem poseEstimator;
  private final GyroSubsystem gyroSubsystem;
  private final AprilTagFieldLayout fieldLayout;
  private final List<VisionCamera> cameras;

  private boolean hasRecentPose = false;
  
  // NEW: Frame skipping to reduce CPU load
  private int visionLoopCounter = 0;
  private static final int VISION_SKIP_CYCLES = 2; // Process every 3rd loop (50Hz → 16Hz)
  
  // NEW: Track current detection state for PoseEstimator to check
  private boolean currentlyHasMultiTag = false;
  private boolean currentlyHasSingleTag = false;
  private int currentBestTagCount = 0;
  private double currentBestDistance = Double.MAX_VALUE;

  private final Map<String, Double> lastConsoleOutputTime = new HashMap<>();
  private static final double CONSOLE_OUTPUT_INTERVAL_SECONDS = 3.0;

  private static final Pose2d EXPECTED_TEST_POSE = new Pose2d(3.57, 2.44, Rotation2d.fromDegrees(240.0));
  private static final double POSITION_TOLERANCE_METERS = 0.15;
  private static final double ROTATION_TOLERANCE_DEGREES = 5.0;
  
  public TagVisionSubsystem(PoseEstimatorSubsystem poseEstimator, GyroSubsystem gyroSubsystem) {
    this.poseEstimator = poseEstimator;
    this.gyroSubsystem = gyroSubsystem;

    try {
      fieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load AprilTag field layout", e);
    }

    cameras = new ArrayList<>();
    
    // Limelight 3 - Front camera (ACTIVE)
    cameras.add(new LimelightCamera(LL_FRONT_NAME, 0));
    
    // DISABLED: PhotonVision cameras - too inaccurate currently
    // TODO: Re-enable after calibration improves
    /*
    cameras.add(new PhotonVisionCamera(
        PV_BACK_LEFT_NAME,
        "RLCalibratedAT",
        new Transform3d(
            new Translation3d(
                BACK_LEFT_PV_X_METERS,
                BACK_LEFT_PV_Y_METERS,
                BACK_LEFT_PV_Z_METERS),
            new Rotation3d(
                Units.degreesToRadians(BACK_LEFT_PV_ROLL_DEG),
                Units.degreesToRadians(BACK_LEFT_PV_PITCH_DEG),
                Units.degreesToRadians(BACK_LEFT_PV_YAW_DEG))),
        fieldLayout));
    
    cameras.add(new PhotonVisionCamera(
        PV_BACK_RIGHT_NAME,
        "RRCalibratedAT",
        new Transform3d(
            new Translation3d(
                BACK_RIGHT_PV_X_METERS,
                BACK_RIGHT_PV_Y_METERS,
                BACK_RIGHT_PV_Z_METERS),
            new Rotation3d(
                Units.degreesToRadians(BACK_RIGHT_PV_ROLL_DEG),
                Units.degreesToRadians(BACK_RIGHT_PV_PITCH_DEG),
                Units.degreesToRadians(BACK_RIGHT_PV_YAW_DEG))),
        fieldLayout));
    */
    
    System.out.println("TagVisionSubsystem initialized:");
    System.out.println("  - " + LL_FRONT_NAME + " (Limelight 3) - ACTIVE");
    System.out.println("  - " + PV_BACK_LEFT_NAME + " (PhotonVision) - DISABLED (inaccurate)");
    System.out.println("  - " + PV_BACK_RIGHT_NAME + " (PhotonVision) - DISABLED (inaccurate)");
    System.out.println("  - QuestNav SLAM - ACTIVE");
  }

  @Override
  public void periodic() {
    // EARLY RETURN: Limelight disabled via feature flag
    if (!LIMELIGHT_ENABLED) {
      Logger.recordOutput("TagVision/LimelightEnabled", false);
      Logger.recordOutput("TagVision/PhotonVisionEnabled", false);
      Logger.recordOutput("TagVision/DisabledViaFeatureFlag", true);
      return;
    }
    
    // LIMELIGHT ONLY - PhotonVision disabled for QuestNav-only testing
    
    // Process Limelight camera only
    for (VisionCamera camera : cameras) {
      // Skip PhotonVision cameras
      if (!camera.getName().equals(LL_FRONT_NAME)) {
        continue;
      }
      
      Optional<VisionResult> result = camera.getLatestResult();
      
      if (result.isEmpty()) {
        continue; // No tags visible
      }
      
      VisionResult visionResult = result.get();
      
      // Add measurement to pose estimator
      poseEstimator.addVisionMeasurement(
          visionResult.estimatedPose,
          visionResult.timestampSeconds,  // Now using latency-compensated timestamp!
          visionResult.tagCount,
          camera.getName());
      
      // Update state tracking
      hasRecentPose = true;
      if (visionResult.tagCount >= MIN_TAG_COUNT_FOR_MULTI) {
        currentlyHasMultiTag = true;
      } else {
        currentlyHasSingleTag = true;
      }
      currentBestTagCount = Math.max(currentBestTagCount, visionResult.tagCount);
      
      // Logging
      Logger.recordOutput("TagVision/Camera/" + camera.getName() + "/Pose", visionResult.estimatedPose);
      Logger.recordOutput("TagVision/Camera/" + camera.getName() + "/TagCount", visionResult.tagCount);
      Logger.recordOutput("TagVision/Camera/" + camera.getName() + "/Timestamp", visionResult.timestampSeconds);
    }
    
    // Overall status
    Logger.recordOutput("TagVision/LimelightEnabled", true);
    Logger.recordOutput("TagVision/PhotonVisionEnabled", false);
    Logger.recordOutput("TagVision/HasAnyTarget", hasRecentPose);
    Logger.recordOutput("TagVision/CurrentlyHasMultiTag", currentlyHasMultiTag);
    Logger.recordOutput("TagVision/CurrentlyHasSingleTag", currentlyHasSingleTag);
  }

  // NEW: Always return false - no vision available
  public boolean hasMultiTagDetection() {
    return false; // Vision disabled
  }
  
  public boolean hasSingleTagDetection() {
    return false; // Vision disabled
  }

  public boolean hasRecentTagPose() {
    return hasRecentPose;
  }

  public boolean hasTarget(String cameraName) {
    return cameras.stream()
        .filter(cam -> cam.getName().equals(cameraName))
        .findFirst()
        .map(VisionCamera::hasTarget)
        .orElse(false);
  }
  
  public int getCameraCount() {
    return cameras.size();
  }
  
  public int getActiveCameraCount() {
    return (int) cameras.stream()
        .filter(VisionCamera::hasTarget)
        .count();
  }
}