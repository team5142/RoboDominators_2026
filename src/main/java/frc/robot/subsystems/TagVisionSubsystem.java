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
  private final PoseEstimatorSubsystem poseEstimator;
  private final GyroSubsystem gyroSubsystem;
  private final AprilTagFieldLayout fieldLayout;
  private final List<VisionCamera> cameras;

  private boolean hasRecentPose = false;
  
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
    
    cameras.add(new LimelightCamera(LL_FRONT_NAME, 0));
    
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
    
    // Reset detection flags each cycle
    currentlyHasMultiTag = false;
    currentlyHasSingleTag = false;
    currentBestTagCount = 0;
    currentBestDistance = Double.MAX_VALUE;

    for (VisionCamera camera : cameras) {
      try {
        boolean accepted = processVisionUpdate(camera);
        if (accepted) {
          totalAccepted++;
          hasRecentPose = true;
        } else if (camera.hasTarget()) {
          totalRejected++;
        }
      } catch (Exception e) {
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
    
    // NEW: Log current detection state
    Logger.recordOutput("TagVision/CurrentlyHasMultiTag", currentlyHasMultiTag);
    Logger.recordOutput("TagVision/CurrentlyHasSingleTag", currentlyHasSingleTag);
    Logger.recordOutput("TagVision/CurrentBestTagCount", currentBestTagCount);
  }

  private boolean processVisionUpdate(VisionCamera camera) {
    Optional<VisionResult> result = camera.getLatestResult();

    if (result.isEmpty()) {
      Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", false);
      Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason", "No result from camera");
      return false;
    }

    VisionResult visionResult = result.get();

    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/TagCount", visionResult.tagCount);
    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/EstimatedPose", visionResult.estimatedPose);
    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/AvgDistance", visionResult.averageTagDistance);
    Logger.recordOutput("TagVision/" + camera.getName() + "/RawResult/Timestamp", visionResult.timestampSeconds);

    if (visionResult.tagCount == 0) {
      Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason", "No tags");
      Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", false);
      return false;
    }

    Pose2d estimatedPose = visionResult.estimatedPose;
    
    // NEW: Update current detection state
    if (visionResult.tagCount >= 2) {
      currentlyHasMultiTag = true;
      if (visionResult.tagCount > currentBestTagCount || 
          (visionResult.tagCount == currentBestTagCount && visionResult.averageTagDistance < currentBestDistance)) {
        currentBestTagCount = visionResult.tagCount;
        currentBestDistance = visionResult.averageTagDistance;
      }
    } else if (visionResult.tagCount == 1 && visionResult.averageTagDistance < 2.0) {
      currentlyHasSingleTag = true;
      if (currentBestTagCount == 0 || visionResult.averageTagDistance < currentBestDistance) {
        currentBestTagCount = 1;
        currentBestDistance = visionResult.averageTagDistance;
      }
    }
    
    // RELAXED initialization - accept single tag if close
    boolean isMultiTag = visionResult.tagCount >= 2;
    boolean isSingleTagClose = visionResult.tagCount == 1 && visionResult.averageTagDistance < 2.0;
    boolean canInitialize = (isMultiTag || isSingleTagClose) &&
                           poseEstimator.getInitializationState() == PoseEstimatorSubsystem.InitializationState.WAITING_FOR_VISION;
    
    if (canInitialize) {
      edu.wpi.first.math.geometry.Pose3d pose3d = new edu.wpi.first.math.geometry.Pose3d(
          estimatedPose.getX(),
          estimatedPose.getY(),
          0.0,
          new edu.wpi.first.math.geometry.Rotation3d(0.0, 0.0, estimatedPose.getRotation().getRadians()));
      
      gyroSubsystem.setQuestNavPose(pose3d);
      gyroSubsystem.setHeading(estimatedPose.getRotation().getDegrees());
      
      poseEstimator.setInitializedViaVision();
      
      if (isMultiTag) {
        System.out.println("=== VISION READY: MULTI-TAG ===");
        System.out.println("Tags: " + visionResult.tagCount);
      } else {
        System.out.println("=== VISION READY: SINGLE TAG (CLOSE) ===");
        System.out.println("Distance: " + visionResult.averageTagDistance + "m");
        System.err.println("WARNING: Single tag - verify position before driving!");
      }
      
      System.out.println("QuestNav synced to: " + estimatedPose);
      System.out.println("Pigeon2 synced to: " + estimatedPose.getRotation().getDegrees() + "°");
      
      Logger.recordOutput("TagVision/QuestNavCalibrated", true);
      Logger.recordOutput("TagVision/PigeonCalibrated", true);
      Logger.recordOutput("TagVision/CalibrationTagCount", visionResult.tagCount);
      Logger.recordOutput("TagVision/CalibrationMethod", isMultiTag ? "Multi-tag" : "Single-tag");
    }

    // ...existing validation and measurement code...
    
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

    poseEstimator.addVisionMeasurement(
        visionResult.estimatedPose,
        visionResult.timestampSeconds,
        visionResult.tagCount,
        camera.getName());

    Logger.recordOutput("TagVision/" + camera.getName() + "/HasTarget", true);
    Logger.recordOutput("TagVision/" + camera.getName() + "/EstimatedPose", visionResult.estimatedPose);
    Logger.recordOutput("TagVision/" + camera.getName() + "/TagCount", visionResult.tagCount);
    Logger.recordOutput("TagVision/" + camera.getName() + "/AvgDistance", visionResult.averageTagDistance);
    Logger.recordOutput("TagVision/" + camera.getName() + "/RejectionReason", "ACCEPTED");

    return true;
  }

  // NEW: Check if currently seeing 2+ tags (for PoseEstimator initialization check)
  public boolean hasMultiTagDetection() {
    return currentlyHasMultiTag;
  }
  
  // NEW: Check if currently seeing 1 tag close enough (for PoseEstimator initialization check)
  public boolean hasSingleTagDetection() {
    return currentlyHasSingleTag && !currentlyHasMultiTag; // Only true if ONLY single tag
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