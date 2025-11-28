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

// AprilTag vision processing - manages 3 cameras (Limelight + 2 PhotonVision) for robot localization
// Filters bad measurements and sends good ones to PoseEstimator with quality-weighted trust levels
public class TagVisionSubsystem extends SubsystemBase {
  private final PoseEstimatorSubsystem poseEstimator; // Where we send vision measurements
  private final AprilTagFieldLayout fieldLayout; // Official field tag positions (from WPILib)
  private final List<VisionCamera> cameras; // All configured cameras

  private boolean hasRecentPose = false; // True if any camera accepted this loop

  private final Map<String, Double> lastConsoleOutputTime = new HashMap<>(); // Throttle console spam
  private static final double CONSOLE_OUTPUT_INTERVAL_SECONDS = 3.0; // Print camera updates every 3s

  // Test position validation - compares camera estimates to known test location
  private static final Pose2d EXPECTED_TEST_POSE = new Pose2d(3.57, 2.44, Rotation2d.fromDegrees(240.0));
  private static final double POSITION_TOLERANCE_METERS = 0.15; // Accept if within 15cm
  private static final double ROTATION_TOLERANCE_DEGREES = 5.0; // Accept if within 5 degrees

  public TagVisionSubsystem(PoseEstimatorSubsystem poseEstimator) {
    this.poseEstimator = poseEstimator;

    // Load 2025 Reefscape field layout - contains all AprilTag positions
    try {
      fieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load AprilTag field layout", e);
    }

    cameras = new ArrayList<>();
    
    // Front Limelight 3 - front center of robot (no transform needed, configured in camera)
    cameras.add(new LimelightCamera(LL_FRONT_NAME, 0)); // Pipeline 0
    
    // Back Left PhotonVision camera - back-left corner, angled forward-left
    cameras.add(new PhotonVisionCamera(
        PV_BACK_LEFT_NAME,
        "RLCalibratedAT", // Pipeline name must match PhotonVision web UI
        new Transform3d( // Camera position relative to robot center
            new Translation3d(
                BACK_LEFT_PV_X_METERS, // 11.5" behind center
                BACK_LEFT_PV_Y_METERS, // 10" left of center
                BACK_LEFT_PV_Z_METERS), // 8" above ground
            new Rotation3d( // Camera orientation (roll, pitch, yaw)
                Units.degreesToRadians(BACK_LEFT_PV_ROLL_DEG),
                Units.degreesToRadians(BACK_LEFT_PV_PITCH_DEG), // 15deg up to see high tags
                Units.degreesToRadians(BACK_LEFT_PV_YAW_DEG))), // 135deg (forward-left)
        fieldLayout));
    
    // Back Right PhotonVision camera - back-right corner, angled forward-right
    cameras.add(new PhotonVisionCamera(
        PV_BACK_RIGHT_NAME,
        "RRCalibratedAT", // Pipeline name must match PhotonVision web UI
        new Transform3d(
            new Translation3d(
                BACK_RIGHT_PV_X_METERS, // 11.5" behind center
                BACK_RIGHT_PV_Y_METERS, // 10" right of center
                BACK_RIGHT_PV_Z_METERS), // 8" above ground
            new Rotation3d(
                Units.degreesToRadians(BACK_RIGHT_PV_ROLL_DEG),
                Units.degreesToRadians(BACK_RIGHT_PV_PITCH_DEG), // 15deg up to see high tags
                Units.degreesToRadians(BACK_RIGHT_PV_YAW_DEG))), // 225deg (forward-right)
        fieldLayout));
    
    System.out.println("TagVisionSubsystem initialized with 3 cameras:");
    System.out.println("  - " + LL_FRONT_NAME + " (Limelight 3)");
    System.out.println("  - " + PV_BACK_LEFT_NAME + " (PhotonVision, Pipeline: RLCalibratedAT)");
    System.out.println("  - " + PV_BACK_RIGHT_NAME + " (PhotonVision, Pipeline: RRCalibratedAT)");
  }

  @Override
  public void periodic() {
    hasRecentPose = false;
    int totalAccepted = 0; // How many cameras successfully updated pose this loop
    int totalRejected = 0; // How many cameras saw tags but failed validation
    int totalErrors = 0; // How many cameras threw exceptions

    // Process each camera independently - failures don't affect other cameras
    for (VisionCamera camera : cameras) {
      try {
        boolean accepted = processVisionUpdate(camera);
        if (accepted) {
          totalAccepted++;
          hasRecentPose = true;
        } else if (camera.hasTarget()) {
          totalRejected++; // Camera saw tags but measurement was rejected
        }
      } catch (Exception e) {
        totalErrors++; // Camera error - log but continue with other cameras
        Logger.recordOutput("TagVision/" + camera.getName() + "/Error", e.getMessage());
        System.err.println("[WARNING] Vision camera '" + camera.getName() + "' error: " + e.getMessage());
      }
    }

    // Summary logging - shows multi-camera health at a glance
    Logger.recordOutput("TagVision/HasAnyTarget", hasRecentPose);
    Logger.recordOutput("TagVision/AcceptedUpdates", totalAccepted);
    Logger.recordOutput("TagVision/RejectedUpdates", totalRejected);
    Logger.recordOutput("TagVision/ErrorCount", totalErrors);
    Logger.recordOutput("TagVision/ActiveCameras", getCameraCount() - totalErrors);
  }

  // Process vision measurement from one camera - returns true if accepted by pose estimator
  private boolean processVisionUpdate(VisionCamera camera) {
    Optional<VisionResult> result = camera.getLatestResult();

    if (result.isEmpty()) {
      Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", false);
      Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason", "No result from camera");
      return false;
    }

    VisionResult visionResult = result.get();

    // Log raw result BEFORE any filtering - useful for debugging rejected measurements
    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/TagCount", visionResult.tagCount);
    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/EstimatedPose", visionResult.estimatedPose);
    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/AvgDistance", visionResult.averageTagDistance);
    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/Timestamp", visionResult.timestampSeconds);

    // Reject if no tags detected
    if (visionResult.tagCount == 0) {
      Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason", "No tags");
      Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", false);
      return false;
    }

    Pose2d estimatedPose = visionResult.estimatedPose;
    
    // Calculate error from known test position - helps validate camera calibration
    double positionError = estimatedPose.getTranslation().getDistance(EXPECTED_TEST_POSE.getTranslation());
    double rotationError = Math.abs(
        estimatedPose.getRotation().minus(EXPECTED_TEST_POSE.getRotation()).getDegrees());
    
    // Log camera estimate with test validation metrics
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/EstimatedX", estimatedPose.getX());
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/EstimatedY", estimatedPose.getY());
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/EstimatedRotation", 
        estimatedPose.getRotation().getDegrees());
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/PositionErrorMeters", positionError);
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/RotationErrorDegrees", rotationError);
    Logger.recordOutput("TagVision/" + camera.getName() + "/Test/WithinTolerance", 
        positionError < POSITION_TOLERANCE_METERS && rotationError < ROTATION_TOLERANCE_DEGREES);
    
    // Throttled console output - prevents spam while still showing periodic updates
    double currentTime = Timer.getFPGATimestamp();
    double lastOutputTime = lastConsoleOutputTime.getOrDefault(camera.getName(), 0.0);
    
    if (currentTime - lastOutputTime >= CONSOLE_OUTPUT_INTERVAL_SECONDS) {
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

    // Quality check for single-tag measurements - reject distant/unreliable detections
    if (visionResult.tagCount == 1) {
      if (visionResult.averageTagDistance > 20.0) { // 20m is unrealistically far
        Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason", 
            "Too far: " + visionResult.averageTagDistance + "m");
        Logger.recordOutput("TagVision/" + camera.getName() + "/RejectedDistance",
            visionResult.averageTagDistance);
        Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", false);
        return false;
      }
    }

    // Sanity check - reject if vision disagrees too much with current pose estimate
    // This prevents bad measurements from corrupting odometry
    Pose2d currentPose = poseEstimator.getEstimatedPose();
    double distance = currentPose.getTranslation()
        .getDistance(visionResult.estimatedPose.getTranslation());

    if (distance > MAX_POSE_DIFFERENCE_METERS) { // 2m threshold
      Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason",
          "Pose diff too large: " + distance + "m");
      Logger.recordOutput("TagVision/" + camera.getName() + "/RejectedPoseDiff", distance);
      Logger.recordOutput("TagVision/" + camera.getName() + "/CurrentPose", currentPose);
      Logger.recordOutput("TagVision/" + camera.getName() + "/VisionPose", visionResult.estimatedPose);
      Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", false);
      return false;
    }

    // ACCEPTED - send to pose estimator with camera-specific quality weighting
    poseEstimator.addVisionMeasurement(
        visionResult.estimatedPose,
        visionResult.timestampSeconds,
        visionResult.tagCount,
        camera.getName()); // Camera name used to apply quality multiplier

    // Log successful measurement
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
