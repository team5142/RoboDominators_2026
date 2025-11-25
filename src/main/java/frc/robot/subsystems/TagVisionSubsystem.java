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

public class TagVisionSubsystem extends SubsystemBase {
  private final PoseEstimatorSubsystem poseEstimator;
  private final AprilTagFieldLayout fieldLayout;
  private final List<VisionCamera> cameras;

  private boolean hasRecentPose = false;

  // NEW: Throttle console output per camera
  private final Map<String, Double> lastConsoleOutputTime = new HashMap<>();
  private static final double CONSOLE_OUTPUT_INTERVAL_SECONDS = 3.0;

  // NEW: Expected test pose for validation (1m from Tag 17)
  private static final Pose2d EXPECTED_TEST_POSE = new Pose2d(3.57, 2.44, Rotation2d.fromDegrees(240.0)); // Updated based on actual position
  private static final double POSITION_TOLERANCE_METERS = 0.15;
  private static final double ROTATION_TOLERANCE_DEGREES = 5.0;

  public TagVisionSubsystem(PoseEstimatorSubsystem poseEstimator) {
    this.poseEstimator = poseEstimator;

    // Load 2025 field layout
    try {
      fieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load AprilTag field layout", e);
    }

    // Initialize cameras
    cameras = new ArrayList<>();
    
    // Front Limelight 3
    cameras.add(new LimelightCamera(LL_FRONT_NAME, 0));
    
    // Back Left PhotonVision camera (RLTagPV with RLCalibratedAT pipeline)
    cameras.add(new PhotonVisionCamera(
        PV_BACK_LEFT_NAME,
        "RLCalibratedAT",  // Pipeline name
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
    
    // Back Right PhotonVision camera (RRTagPV with RRCalibratedAT pipeline)
    cameras.add(new PhotonVisionCamera(
        PV_BACK_RIGHT_NAME,
        "RRCalibratedAT",  // Pipeline name
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
    
    System.out.println("TagVisionSubsystem initialized with 3 cameras:");
    System.out.println("  - " + LL_FRONT_NAME + " (Limelight 3)");
    System.out.println("  - " + PV_BACK_LEFT_NAME + " (PhotonVision, Pipeline: RLCalibratedAT)");
    System.out.println("  - " + PV_BACK_RIGHT_NAME + " (PhotonVision, Pipeline: RRCalibratedAT)");
  }

  @Override
  public void periodic() {
    hasRecentPose = false;
    int totalAccepted = 0;
    int totalRejected = 0;
    int totalErrors = 0;

    // Process all cameras
    for (VisionCamera camera : cameras) {
      try {
        boolean accepted = processVisionUpdate(camera);
        if (accepted) {
          totalAccepted++;
          hasRecentPose = true;
        } else if (camera.hasTarget()) {
          totalRejected++; // Had target but rejected
        }
      } catch (Exception e) {
        // Camera error - log but continue with other cameras
        totalErrors++;
        Logger.recordOutput("TagVision/" + camera.getName() + "/Error", e.getMessage());
        System.err.println("[WARNING] Vision camera '" + camera.getName() + "' error: " + e.getMessage());
      }
    }

    Logger.recordOutput("TagVision/HasAnyTarget", hasRecentPose);
    Logger.recordOutput("TagVision/AcceptedUpdates", totalAccepted);
    Logger.recordOutput("TagVision/RejectedUpdates", totalRejected);
    Logger.recordOutput("TagVision/ErrorCount", totalErrors);
    Logger.recordOutput("TagVision/ActiveCameras", getCameraCount() - totalErrors);
  }

  private boolean processVisionUpdate(VisionCamera camera) {
    // === Process all cameras normally ===
    Optional<VisionResult> result = camera.getLatestResult();

    if (result.isEmpty()) {
      Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", false);
      Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason", "No result from camera");
      return false;
    }

    VisionResult visionResult = result.get();

    // NEW: Log raw vision result details BEFORE validation
    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/TagCount", visionResult.tagCount);
    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/EstimatedPose", visionResult.estimatedPose);
    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/AvgDistance", visionResult.averageTagDistance);
    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/Timestamp", visionResult.timestampSeconds);

    // Validation: Check tag count
    if (visionResult.tagCount == 0) {
      Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason", "No tags");
      Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", false);
      return false;
    }

    // === DETAILED CAMERA TESTING LOGS ===
    Pose2d estimatedPose = visionResult.estimatedPose;
    
    // Calculate error from expected test pose
    double positionError = estimatedPose.getTranslation().getDistance(EXPECTED_TEST_POSE.getTranslation());
    double rotationError = Math.abs(
        estimatedPose.getRotation().minus(EXPECTED_TEST_POSE.getRotation()).getDegrees());
    
    // Log detailed camera estimate (always log to AdvantageScope)
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/EstimatedX", estimatedPose.getX());
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/EstimatedY", estimatedPose.getY());
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/EstimatedRotation", 
        estimatedPose.getRotation().getDegrees());
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/PositionErrorMeters", positionError);
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/RotationErrorDegrees", rotationError);
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/WithinTolerance", 
        positionError < POSITION_TOLERANCE_METERS && rotationError < ROTATION_TOLERANCE_DEGREES);
    
    // THROTTLED console output (every 3 seconds per camera)
    double currentTime = Timer.getFPGATimestamp();
    double lastOutputTime = lastConsoleOutputTime.getOrDefault(camera.getName(), 0.0);
    
    if (currentTime - lastOutputTime >= CONSOLE_OUTPUT_INTERVAL_SECONDS) {
      // Format tag IDs for display
      String tagIdsStr = visionResult.tagIds.stream()
          .map(String::valueOf)
          .collect(java.util.stream.Collectors.joining(","));
      
      System.out.println(String.format(
          "[%s] ENABLED - Estimate: (%.3f, %.3f, %.1f°) | Tags: %d [%s] | AvgDist: %.2fm",
          camera.getName(),
          estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getRotation().getDegrees(),
          visionResult.tagCount,
          tagIdsStr,
          visionResult.averageTagDistance));
      
      lastConsoleOutputTime.put(camera.getName(), currentTime);
    }

    // Validation: For single-tag detections, check quality
    if (visionResult.tagCount == 1) {
      if (visionResult.averageTagDistance > 20.0) {
        Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason", 
            "Too far: " + visionResult.averageTagDistance + "m");
        Logger.recordOutput("TagVision/" + camera.getName() + "/RejectedDistance",
            visionResult.averageTagDistance);
        Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", false);
        return false;
      }
    }

    // Sanity check: reject if pose is too far from current estimate
    Pose2d currentPose = poseEstimator.getEstimatedPose();
    double distance = currentPose.getTranslation()
        .getDistance(visionResult.estimatedPose.getTranslation());

    if (distance > MAX_POSE_DIFFERENCE_METERS) {
      Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason",
          "Pose diff too large: " + distance + "m");
      Logger.recordOutput("TagVision/" + camera.getName() + "/RejectedPoseDiff", distance);
      Logger.recordOutput("TagVision/" + camera.getName() + "/CurrentPose", currentPose);
      Logger.recordOutput("TagVision/" + camera.getName() + "/VisionPose", visionResult.estimatedPose);
      Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", false);
      return false;
    }

    // ACCEPTED - Add measurement to pose estimator WITH CAMERA-SPECIFIC TRUST
    poseEstimator.addVisionMeasurement(
        visionResult.estimatedPose,
        visionResult.timestampSeconds,
        visionResult.tagCount,
        camera.getName()); // NEW: Pass camera name for quality weighting

    // Log acceptance
    Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", true);
    Logger.recordOutput("TagVision/" + camera.getName() + "/EstimatedPose", visionResult.estimatedPose);
    Logger.recordOutput("TagVision/" + camera.getName() + "/TagCount", visionResult.tagCount);
    Logger.recordOutput("TagVision/" + camera.getName() + "/AvgDistance", visionResult.averageTagDistance);
    Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason", "ACCEPTED");

    return true;
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
  
  /**
   * Get total number of cameras configured
   */
  public int getCameraCount() {
    return cameras.size();
  }
  
  /**
   * Get number of cameras currently seeing targets
   */
  public int getActiveCameraCount() {
    return (int) cameras.stream()
        .filter(VisionCamera::hasTarget)
        .count();
  }
}
