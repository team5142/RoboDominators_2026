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

/**
 * PhotonVision camera implementation for AprilTag detection.
 * Wraps PhotonVision API to work with our VisionCamera interface.
 */
public class PhotonVisionCamera implements VisionCamera {
  private final String name;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private PhotonPipelineResult latestResult;
  private boolean connectionWarningShown = false; // Prevent spam

  /**
   * Create a PhotonVision camera for AprilTag pose estimation.
   * 
   * @param cameraName Name of the camera in PhotonVision
   * @param pipelineName Name of the pipeline to use (e.g., "RLCalibratedAT")
   * @param robotToCamera Transform from robot center to camera
   * @param fieldLayout AprilTag field layout for 2025
   */
  public PhotonVisionCamera(
      String cameraName,
      String pipelineName,
      Transform3d robotToCamera,
      AprilTagFieldLayout fieldLayout) {
    this.name = cameraName;
    this.camera = new PhotonCamera(cameraName);
    
    // Set the pipeline by name
    camera.setPipelineIndex(getPipelineIndex(pipelineName));
    
    // Create pose estimator
    this.poseEstimator = new PhotonPoseEstimator(
        fieldLayout,
        // TESTING: Use single-tag mode for RLTagPV to avoid Tag 22 multi-tag issues
        cameraName.equals("RLTagPV") 
            ? PoseStrategy.LOWEST_AMBIGUITY  // Single-tag only for RLTagPV
            : PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  // Multi-tag for others
        robotToCamera);
    
    // Enable multi-tag fallback (only used if primary strategy is multi-tag)
    if (!cameraName.equals("RLTagPV")) {
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    
    System.out.println("PhotonVision camera initialized: " + cameraName 
        + " (Pipeline: " + pipelineName 
        + ", Strategy: " + (cameraName.equals("RLTagPV") ? "SINGLE_TAG" : "MULTI_TAG") + ")");
  }

  /**
   * Get pipeline index by name (PhotonVision API requires index, not name)
   * For now, assumes pipeline 0 - you may need to adjust this based on your PhotonVision config
   */
  private int getPipelineIndex(String pipelineName) {
    // PhotonVision pipelines are indexed 0, 1, 2, etc.
    // Since you're using "Duel Point Offset Mode", pipeline 0 should be the calibrated one
    // If you have multiple pipelines, you'll need to map names to indices
    
    // For now, default to pipeline 0 (the calibrated AprilTag pipeline)
    return 0;
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public Optional<VisionResult> getLatestResult() {
    try {
      latestResult = camera.getLatestResult();
      
      // DEBUG: Always log what we get (AdvantageScope only - no console spam)
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/Debug/ResultExists", latestResult != null);
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/Debug/HasTargets", latestResult != null && latestResult.hasTargets());
      
      if (latestResult != null && latestResult.hasTargets()) {
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/Debug/TargetCount", latestResult.getTargets().size());
        
        // Log which tag IDs we see
        var tagIds = latestResult.getTargets().stream()
            .mapToInt(target -> target.getFiducialId())
            .toArray();
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/Debug/TagIDs", 
            java.util.Arrays.stream(tagIds).mapToObj(String::valueOf).toArray(String[]::new));
      }
      
      if (!latestResult.hasTargets()) {
        connectionWarningShown = false; // Reset warning when camera recovers
        return Optional.empty();
      }

      // Get pose estimate from PhotonVision
      Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(latestResult);
      
      if (estimatedPose.isEmpty()) {
        // Log why PhotonVision rejected it
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/PhotonRejected", true);
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/HasTargets", true);
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/TargetCount", latestResult.getTargets().size());
        return Optional.empty();
      }

      EstimatedRobotPose robotPose = estimatedPose.get();
      
      // Convert to 2D pose
      Pose2d pose2d = robotPose.estimatedPose.toPose2d();
      
      // Extract which tag IDs were used
      List<Integer> usedTagIds = robotPose.targetsUsed.stream()
          .map(target -> target.getFiducialId())
          .collect(java.util.stream.Collectors.toList());
      
      // SAFETY: Reject zero poses (0,0,0) - these are invalid
      if (Math.abs(pose2d.getX()) < 0.01 && Math.abs(pose2d.getY()) < 0.01) {
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/RejectedZeroPose", true);
        System.err.println("[" + name + "] REJECTED: Zero pose (0, 0, 0°) - invalid data");
        return Optional.empty();
      }
      
      // Calculate average distance to tags
      double avgDistance = robotPose.targetsUsed.stream()
          .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
          .average()
          .orElse(0.0);
      
      connectionWarningShown = false; // Reset warning
      
      // DEBUG: Always log when PhotonVision camera gets a valid pose
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/ValidPoseReceived", true);
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/RawPose2d", pose2d);
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/UsedTagIDs", 
          usedTagIds.stream().map(String::valueOf).toArray(String[]::new));
      
      // Create VisionResult
      return Optional.of(new VisionResult(
          pose2d,
          robotPose.timestampSeconds,
          robotPose.targetsUsed.size(),
          avgDistance,
          usedTagIds)); // NEW: Pass tag IDs
      
    } catch (Exception e) {
      // Camera disconnected or communication error
      if (!connectionWarningShown) {
        System.err.println("[WARNING] PhotonVision camera '" + name + "' error: " + e.getMessage());
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

  /**
   * Get the raw PhotonVision camera for advanced usage
   */
  public PhotonCamera getCamera() {
    return camera;
  }

  /**
   * Get the raw PhotonPoseEstimator for advanced usage
   */
  public PhotonPoseEstimator getPoseEstimator() {
    return poseEstimator;
  }
}
