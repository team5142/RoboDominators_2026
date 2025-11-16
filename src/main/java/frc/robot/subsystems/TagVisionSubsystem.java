package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightCamera;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionCamera.VisionResult;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class TagVisionSubsystem extends SubsystemBase {
  private final PoseEstimatorSubsystem poseEstimator;
  private final AprilTagFieldLayout fieldLayout;
  private final List<VisionCamera> cameras;

  private boolean hasRecentPose = false;

  public TagVisionSubsystem(PoseEstimatorSubsystem poseEstimator) {
    this.poseEstimator = poseEstimator;

    // Load 2025 field layout
    try {
      fieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load AprilTag field layout", e);
    }

    // Initialize cameras - currently using Limelights
    // Easy to add PhotonVision cameras later
    cameras = new ArrayList<>();
    cameras.add(new LimelightCamera(LL_FRONT_NAME, 0));
    cameras.add(new LimelightCamera(LL_BACK_NAME, 0));

    // Future: Add PhotonVision cameras like this:
    // cameras.add(new PhotonVisionCamera(
    // PV_TAG_FRONT_NAME,
    // new Transform3d(
    // new Translation3d(FRONT_PV_X_METERS, FRONT_PV_Y_METERS, FRONT_PV_Z_METERS),
    // new Rotation3d(
    // Units.degreesToRadians(FRONT_PV_ROLL_DEG),
    // Units.degreesToRadians(FRONT_PV_PITCH_DEG),
    // Units.degreesToRadians(FRONT_PV_YAW_DEG))),
    // fieldLayout));
  }

  @Override
  public void periodic() {
    hasRecentPose = false;

    // Process all cameras
    for (VisionCamera camera : cameras) {
      boolean accepted = processVisionUpdate(camera);
      hasRecentPose = hasRecentPose || accepted;
    }

    Logger.recordOutput("TagVision/HasAnyTarget", hasRecentPose);
  }

  private boolean processVisionUpdate(VisionCamera camera) {
    Optional<VisionResult> result = camera.getLatestResult();

    if (result.isEmpty()) {
      Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", false);
      return false;
    }

    VisionResult visionResult = result.get();

    // Validation: Check tag count
    if (visionResult.tagCount == 0) {
      return false;
    }

    // Validation: For single-tag detections, check quality
    if (visionResult.tagCount == 1) {
      // Reject if too far (using inverse distance as quality metric)
      if (visionResult.averageTagDistance > 20.0) {
        Logger.recordOutput("TagVision/" + camera.getName() + "/RejectedDistance",
            visionResult.averageTagDistance);
        return false;
      }
    }

    // Sanity check: reject if pose is too far from current estimate
    Pose2d currentPose = poseEstimator.getEstimatedPose();
    double distance = currentPose.getTranslation()
        .getDistance(visionResult.estimatedPose.getTranslation());

    if (distance > MAX_POSE_DIFFERENCE_METERS) {
      Logger.recordOutput("TagVision/" + camera.getName() + "/RejectedPoseDiff", distance);
      return false;
    }

    // Add measurement to pose estimator
    poseEstimator.addVisionMeasurement(
        visionResult.estimatedPose,
        visionResult.timestampSeconds,
        visionResult.tagCount);

    // Log
    Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", true);
    Logger.recordOutput("TagVision/" + camera.getName() + "/EstimatedPose", visionResult.estimatedPose);
    Logger.recordOutput("TagVision/" + camera.getName() + "/TagCount", visionResult.tagCount);
    Logger.recordOutput("TagVision/" + camera.getName() + "/AvgDistance", visionResult.averageTagDistance);

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
}
