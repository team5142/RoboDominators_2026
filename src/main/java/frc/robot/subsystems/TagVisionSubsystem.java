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
import frc.robot.util.SmartLogger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

// AprilTag vision processing - provides tag detection status to PoseEstimator
// Currently Limelight-only (PhotonVision disabled for QuestNav-only testing)
public class TagVisionSubsystem extends SubsystemBase {
  
  private static final boolean LIMELIGHT_ENABLED = false; // Vision disabled for QuestNav-only mode
  
  private final PoseEstimatorSubsystem poseEstimator;
  private final GyroSubsystem gyroSubsystem;
  private final AprilTagFieldLayout fieldLayout;
  private final List<VisionCamera> cameras;

  private boolean hasRecentPose = false;
  private boolean currentlyHasMultiTag = false;
  private boolean currentlyHasSingleTag = false;

  public TagVisionSubsystem(PoseEstimatorSubsystem poseEstimator, GyroSubsystem gyroSubsystem) {
    this.poseEstimator = poseEstimator;
    this.gyroSubsystem = gyroSubsystem;

    try {
      fieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load AprilTag field layout", e);
    }

    cameras = new ArrayList<>();
    
    // Limelight 3 - Front camera (active when LIMELIGHT_ENABLED = true)
    cameras.add(new LimelightCamera(LL_FRONT_NAME, 0));
    
    // PhotonVision cameras (disabled - uncomment when new stable mounts installed)
    /*
    cameras.add(new PhotonVisionCamera(
        PV_BACK_LEFT_NAME,
        "RLCalibratedAT",
        new Transform3d(
            new Translation3d(BACK_LEFT_PV_X_METERS, BACK_LEFT_PV_Y_METERS, BACK_LEFT_PV_Z_METERS),
            new Rotation3d(
                Units.degreesToRadians(BACK_LEFT_PV_ROLL_DEG),
                Units.degreesToRadians(BACK_LEFT_PV_PITCH_DEG),
                Units.degreesToRadians(BACK_LEFT_PV_YAW_DEG))),
        fieldLayout));
    
    cameras.add(new PhotonVisionCamera(
        PV_BACK_RIGHT_NAME,
        "RRCalibratedAT",
        new Transform3d(
            new Translation3d(BACK_RIGHT_PV_X_METERS, BACK_RIGHT_PV_Y_METERS, BACK_RIGHT_PV_Z_METERS),
            new Rotation3d(
                Units.degreesToRadians(BACK_RIGHT_PV_ROLL_DEG),
                Units.degreesToRadians(BACK_RIGHT_PV_PITCH_DEG),
                Units.degreesToRadians(BACK_RIGHT_PV_YAW_DEG))),
        fieldLayout));
    */
    
    SmartLogger.logConsole("TagVisionSubsystem: Limelight only (PhotonVision disabled)");
  }

  @Override
  public void periodic() {
    if (!LIMELIGHT_ENABLED) {
      SmartLogger.logReplay("TagVision/Disabled", true);
      return;
    }
    
    // Process Limelight camera only
    for (VisionCamera camera : cameras) {
      if (!camera.getName().equals(LL_FRONT_NAME)) {
        continue;
      }
      
      Optional<VisionResult> result = camera.getLatestResult();
      if (result.isEmpty()) {
        continue;
      }
      
      VisionResult visionResult = result.get();
      
      poseEstimator.addVisionMeasurement(
          visionResult.estimatedPose,
          visionResult.timestampSeconds,
          visionResult.tagCount,
          camera.getName());
      
      hasRecentPose = true;
      if (visionResult.tagCount >= MIN_TAG_COUNT_FOR_MULTI) {
        currentlyHasMultiTag = true;
      } else {
        currentlyHasSingleTag = true;
      }
      
      SmartLogger.logReplay("TagVision/" + camera.getName() + "/Pose", visionResult.estimatedPose);
      SmartLogger.logReplay("TagVision/" + camera.getName() + "/TagCount", (double) visionResult.tagCount);
    }
    
    SmartLogger.logReplay("TagVision/HasTarget", hasRecentPose);
    SmartLogger.logReplay("TagVision/MultiTag", currentlyHasMultiTag);
    SmartLogger.logReplay("TagVision/SingleTag", currentlyHasSingleTag);
  }

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