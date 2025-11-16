package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.ObjectDetection;
import frc.robot.subsystems.vision.ObjectDetection.ObjectType;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectVisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;
  private final RobotState robotState;
  
  private List<ObjectDetection> detectedObjects = new ArrayList<>();
  private Optional<ObjectDetection> closestTarget = Optional.empty();

  public ObjectVisionSubsystem(RobotState robotState) {
    this.robotState = robotState;
    camera = new PhotonCamera(OBJ_CAMERA_NAME);
    
    // Robot-to-camera transform
    robotToCamera = new Transform3d(
        new Translation3d(OBJ_CAMERA_X_METERS, OBJ_CAMERA_Y_METERS, OBJ_CAMERA_Z_METERS),
        new Rotation3d(
            Units.degreesToRadians(OBJ_CAMERA_ROLL_DEG),
            Units.degreesToRadians(OBJ_CAMERA_PITCH_DEG),
            Units.degreesToRadians(OBJ_CAMERA_YAW_DEG)));
    
    // Set to object detection pipeline (index 0)
    camera.setPipelineIndex(0);
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    
    detectedObjects.clear();
    closestTarget = Optional.empty();

    if (!result.hasTargets()) {
      Logger.recordOutput("ObjectVision/HasTargets", false);
      Logger.recordOutput("ObjectVision/TargetCount", 0);
      return;
    }

    // Get current robot pose for field-relative transforms
    Pose2d robotPose = robotState.getRobotPose();

    // Process all detected targets
    for (PhotonTrackedTarget target : result.getTargets()) {
      processTarget(target, result.getTimestampSeconds(), robotPose)
          .ifPresent(detectedObjects::add);
    }

    // Find closest target (in field coordinates)
    if (!detectedObjects.isEmpty()) {
      closestTarget = detectedObjects.stream()
          .min((a, b) -> {
            double distA = robotPose.getTranslation().getDistance(a.getPosition());
            double distB = robotPose.getTranslation().getDistance(b.getPosition());
            return Double.compare(distA, distB);
          });
    }

    // Logging
    Logger.recordOutput("ObjectVision/HasTargets", true);
    Logger.recordOutput("ObjectVision/TargetCount", detectedObjects.size());
    Logger.recordOutput("ObjectVision/DetectedPositions", 
        detectedObjects.stream()
            .map(obj -> new Pose2d(obj.getPosition(), new Rotation2d()))
            .toArray(Pose2d[]::new));
    
    if (closestTarget.isPresent()) {
      ObjectDetection closest = closestTarget.get();
      Logger.recordOutput("ObjectVision/ClosestTarget/Type", closest.getType().toString());
      Logger.recordOutput("ObjectVision/ClosestTarget/Distance", closest.getDistanceMeters());
      Logger.recordOutput("ObjectVision/ClosestTarget/Angle", closest.getAngleToTarget().getDegrees());
      Logger.recordOutput("ObjectVision/ClosestTarget/FieldPosition", 
          new Pose2d(closest.getPosition(), new Rotation2d()));
    }
  }

  private Optional<ObjectDetection> processTarget(
      PhotonTrackedTarget target, 
      double timestamp, 
      Pose2d robotPose) {
    // Filter by area
    if (target.getArea() < MIN_TARGET_AREA_PERCENT) {
      return Optional.empty();
    }

    // Determine object type based on class ID (from PhotonVision neural network)
    ObjectType type = classifyTarget(target);
    
    // Calculate distance using pitch angle and known target height
    double targetHeightMeters = getTargetHeight(type);
    double pitchRadians = Units.degreesToRadians(target.getPitch());
    double cameraHeightMeters = OBJ_CAMERA_Z_METERS;
    
    // Distance calculation: d = (h_target - h_camera) / tan(pitch)
    // Handle edge cases where pitch is near zero
    double distance;
    if (Math.abs(pitchRadians) < 0.01) {
      // Too close to horizontal, use alternative method or reject
      return Optional.empty();
    } else {
      distance = Math.abs((targetHeightMeters - cameraHeightMeters) / Math.tan(pitchRadians));
    }
    
    // Sanity check
    if (distance > MAX_TARGET_DISTANCE_METERS || distance < 0.1) {
      return Optional.empty();
    }

    // Calculate angle to target (yaw from camera)
    Rotation2d yawFromCamera = Rotation2d.fromDegrees(target.getYaw());

    // Transform to robot-relative coordinates
    // Account for camera offset and yaw
    double robotRelativeX = distance * yawFromCamera.getCos() + OBJ_CAMERA_X_METERS;
    double robotRelativeY = distance * yawFromCamera.getSin() + OBJ_CAMERA_Y_METERS;
    Translation2d robotRelativePosition = new Translation2d(robotRelativeX, robotRelativeY);

    // Transform to field coordinates
    Transform2d robotToTarget = new Transform2d(robotRelativePosition, new Rotation2d());
    Translation2d fieldPosition = robotPose.transformBy(robotToTarget).getTranslation();

    // Calculate robot-relative angle for driver assistance
    Rotation2d angleToTarget = new Rotation2d(robotRelativeX, robotRelativeY);

    return Optional.of(new ObjectDetection(
        type,
        fieldPosition,  // Now field-relative!
        target.getArea() / 100.0,  // Normalize area as confidence
        distance,
        angleToTarget,
        timestamp));
  }

  private ObjectType classifyTarget(PhotonTrackedTarget target) {
    // PhotonVision class IDs from neural network
    // Class 0 = Coral, Class 1 = Algae (update based on your pipeline)
    int classId = target.getFiducialId();  // Using getFiducialId as proxy for classId
    
    switch (classId) {
      case 0:
        return ObjectType.CORAL;
      case 1:
        return ObjectType.ALGAE;
      default:
        return ObjectType.UNKNOWN;
    }
  }

  private double getTargetHeight(ObjectType type) {
    switch (type) {
      case CORAL:
        return CORAL_HEIGHT_METERS;
      case ALGAE:
        return ALGAE_HEIGHT_METERS;
      default:
        return 0.1; // Default small height
    }
  }

  /**
   * Get all detected objects in field coordinates
   */
  public List<ObjectDetection> getDetectedObjects() {
    return new ArrayList<>(detectedObjects);
  }

  /**
   * Get the closest target (any type) in field coordinates
   */
  public Optional<ObjectDetection> getClosestTarget() {
    return closestTarget;
  }

  /**
   * Get the closest target of a specific type in field coordinates
   */
  public Optional<ObjectDetection> getClosestTargetOfType(ObjectType type) {
    Pose2d robotPose = robotState.getRobotPose();
    return detectedObjects.stream()
        .filter(obj -> obj.getType() == type)
        .min((a, b) -> {
          double distA = robotPose.getTranslation().getDistance(a.getPosition());
          double distB = robotPose.getTranslation().getDistance(b.getPosition());
          return Double.compare(distA, distB);
        });
  }

  public boolean hasTarget() {
    return !detectedObjects.isEmpty();
  }

  public boolean hasTargetOfType(ObjectType type) {
    return detectedObjects.stream().anyMatch(obj -> obj.getType() == type);
  }

  /**
   * Get field-relative Translation2d to the closest target
   */
  public Optional<Translation2d> getClosestTargetPosition() {
    return closestTarget.map(ObjectDetection::getPosition);
  }

  /**
   * Get robot-relative angle to closest target (for driver feedback)
   */
  public Optional<Rotation2d> getAngleToClosestTarget() {
    return closestTarget.map(ObjectDetection::getAngleToTarget);
  }
}
