package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.util.SmartLogger;

// PhotonVision camera wrapper - reads from PhotonVision coprocessor (OrangePi/RaspberryPi)
// Uses PhotonLib to get pose estimates from camera's onboard processing
// Supports multi-tag fusion (better than Limelight for multiple tags in view)
public class PhotonVisionCamera implements VisionCamera {
  private final String name; // Camera name in PhotonVision web UI
  private final PhotonCamera camera; // PhotonLib camera object
  private final PhotonPoseEstimator poseEstimator; // Handles 3D pose math
  private PhotonPipelineResult latestResult; // Cached result for hasTarget() check
  private boolean connectionWarningShown = false; // Throttle error messages

  // Create PhotonVision camera with robot-to-camera transform for pose estimation.
  // pipelineName is informational only - there is currently only one pipeline (index 0).
  public PhotonVisionCamera(
      String cameraName,
      String pipelineName, // Pipeline name in PhotonVision UI (e.g. "RLCalibratedAT")
      Transform3d robotToCamera, // Camera position/orientation relative to robot center
      AprilTagFieldLayout fieldLayout) { // Field AprilTag layout for pose estimation
    this.name = cameraName;
    this.camera = new PhotonCamera(cameraName);
    
    camera.setPipelineIndex(getPipelineIndex(pipelineName)); // Set active pipeline
    
    // Create pose estimator with strategy based on camera name
    // RLTagPV uses single-tag only (Tag 22 multi-tag causes issues)
    // Others use multi-tag for better accuracy
    this.poseEstimator = new PhotonPoseEstimator(
        fieldLayout,
        cameraName.equals("RLTagPV") 
            ? PoseStrategy.LOWEST_AMBIGUITY  // Single-tag only (best fit)
            : PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  // Multi-tag (most accurate)
        robotToCamera);
    
    // Enable fallback to single-tag if multi-tag fails
    if (!cameraName.equals("RLTagPV")) {
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    
    SmartLogger.logConsole("PhotonVision camera initialized: " + cameraName
        + " (Strategy: " + (cameraName.equals("RLTagPV") ? "SINGLE_TAG" : "MULTI_TAG") + ")",
        "Vision");
  }

  // Always returns pipeline 0 - only one pipeline is configured.
  // If multiple pipelines are ever needed, map names to indices here.
  private int getPipelineIndex(String pipelineName) {
    return 0;
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public Optional<VisionResult> getLatestResult() {
    try {
      List<PhotonPipelineResult> unread = camera.getAllUnreadResults();
      if (unread.isEmpty()) {
        return Optional.empty();
      }
      latestResult = unread.get(unread.size() - 1); // Use the newest frame
      
      if (!latestResult.hasTargets()) {
        connectionWarningShown = false; // Reset warning when camera recovers
        return Optional.empty();
      }

      // Get pose estimate from PhotonVision's on-coprocessor processing
      Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(latestResult);
      
      if (estimatedPose.isEmpty()) {
        // PhotonVision rejected the measurement (ambiguity too high, tags too far, etc.)
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/PhotonRejected", true);
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/HasTargets", true);
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/TargetCount", latestResult.getTargets().size());
        return Optional.empty();
      }

      EstimatedRobotPose robotPose = estimatedPose.get();
      
      Pose2d pose2d = robotPose.estimatedPose.toPose2d(); // Convert 3D pose to 2D (drop Z)
      
      // Extract which tag IDs contributed to this pose
      List<Integer> usedTagIds = robotPose.targetsUsed.stream()
          .map(target -> target.getFiducialId())
          .collect(java.util.stream.Collectors.toList());
      
      // SAFETY: Reject (0,0,0) poses - these are invalid/uninitialized
      if (Math.abs(pose2d.getX()) < 0.01 && Math.abs(pose2d.getY()) < 0.01) {
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/RejectedZeroPose", true);
        SmartLogger.logConsoleError("[Vision] " + name + ": rejected zero pose (invalid data)");
        return Optional.empty();
      }
      
      // Calculate average distance to all tags used in pose estimate
      double avgDistance = robotPose.targetsUsed.stream()
          .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
          .average()
          .orElse(0.0);
      
      connectionWarningShown = false; // Reset warning
      
      org.littletonrobotics.junction.Logger.recordOutput("Vision/" + name + "/RawPose2d", pose2d);
      
      return Optional.of(new VisionResult(
          pose2d,
          robotPose.timestampSeconds,
          robotPose.targetsUsed.size(),
          avgDistance,
          usedTagIds));
      
    } catch (Exception e) {
      // Camera disconnected, coprocessor crash, or NetworkTables error
      if (!connectionWarningShown) {
        SmartLogger.logConsoleError("[Vision] PhotonVision '" + name + "' error: " + e.getMessage());
        connectionWarningShown = true;
      }
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/ConnectionError", e.getMessage());
      return Optional.empty();
    }
  }

  @Override
  public boolean hasTarget() {
    try {
      return latestResult != null && latestResult.hasTargets();
    } catch (Exception e) {
      return false;
    }
  }

  // Get raw PhotonVision camera for advanced usage (pipeline switching, LED control, etc.)
  public PhotonCamera getCamera() {
    return camera;
  }

  // Get raw PhotonPoseEstimator for advanced configuration
  public PhotonPoseEstimator getPoseEstimator() {
    return poseEstimator;
  }
}
