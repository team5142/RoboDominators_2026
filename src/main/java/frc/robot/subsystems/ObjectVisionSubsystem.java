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
import frc.robot.util.SmartLogger;
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
  private boolean connectionWarningShown = false; // Prevent spam

  public ObjectVisionSubsystem(RobotState robotState) {
    this.robotState = robotState;
    
    // Initialize camera transform (position and orientation relative to robot center)
    this.robotToCamera = new Transform3d(
        new Translation3d(OBJ_CAMERA_X_METERS, OBJ_CAMERA_Y_METERS, OBJ_CAMERA_Z_METERS),
        new Rotation3d()
    );
    
    // Initialize PhotonVision camera for object detection (FObjPV with FCalibratedTag pipeline)
    camera = new PhotonCamera(OBJ_CAMERA_NAME);
    
    // Set pipeline to "FCalibratedTag" (pipeline 0 by default)
    camera.setPipelineIndex(0);  // Assumes "FCalibratedTag" is pipeline 0
    
    SmartLogger.logConsole("ObjectVisionSubsystem initialized - Camera: " + OBJ_CAMERA_NAME
        + " Pipeline: FCalibratedTag FOV: " + OBJ_CAMERA_FOV_DEG + "deg", "ObjectVision");
  }

  @Override
  public void periodic() {
    try {
      detectedObjects.clear();
      closestTarget = Optional.empty();

      List<PhotonPipelineResult> unread = camera.getAllUnreadResults();
      if (unread.isEmpty()) {
        Logger.recordOutput("ObjectVision/HasTargets", false);
        Logger.recordOutput("ObjectVision/TargetCount", 0);
        return;
      }
      PhotonPipelineResult result = unread.get(unread.size() - 1); // Use the newest frame

      if (!result.hasTargets()) {
        Logger.recordOutput("ObjectVision/HasTargets", false);
        Logger.recordOutput("ObjectVision/TargetCount", 0);
        connectionWarningShown = false; // Reset warning when camera recovers
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
      
      connectionWarningShown = false; // Reset warning on successful cycle
      
    } catch (Exception e) {
      // Camera disconnected or communication error
      if (!connectionWarningShown) {
        SmartLogger.logConsoleError("Object detection camera error: " + e.getMessage());
        connectionWarningShown = true;
      }
      Logger.recordOutput("ObjectVision/ConnectionError", e.getMessage());
      Logger.recordOutput("ObjectVision/HasTargets", false);
      Logger.recordOutput("ObjectVision/TargetCount", 0);
      
      detectedObjects.clear();
      closestTarget = Optional.empty();
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
    if (Math.abs(pitchRadians) < 0.01) { // Pitch too close to horizontal
      return Optional.empty();
    }
    
    double distance = Math.abs((targetHeightMeters - cameraHeightMeters) / Math.tan(pitchRadians));
    
    // Sanity check distance
    if (distance > MAX_TARGET_DISTANCE_METERS || distance < 0.1) {
      Logger.recordOutput("ObjectVision/Debug/RejectedDistance", distance);
      return Optional.empty();
    }

    // Calculate angle to target (yaw from camera)
    Rotation2d yawFromCamera = Rotation2d.fromDegrees(target.getYaw());

    // Transform to robot-relative coordinates
    double robotRelativeX = distance * yawFromCamera.getCos() + OBJ_CAMERA_X_METERS;
    double robotRelativeY = distance * yawFromCamera.getSin() + OBJ_CAMERA_Y_METERS;
    
    // Check magnitude before creating Rotation2d - avoids undefined angle at robot center
    double magnitude = Math.hypot(robotRelativeX, robotRelativeY);
    
    if (magnitude < 0.01) { // Less than 1cm - too close to robot center
      Logger.recordOutput("ObjectVision/Debug/RejectedZeroMagnitude", magnitude);
      return Optional.empty(); // Return BEFORE creating Rotation2d
    }
    
    Translation2d robotRelativePosition = new Translation2d(robotRelativeX, robotRelativeY);

    // Transform to field coordinates
    Transform2d robotToTarget = new Transform2d(robotRelativePosition, new Rotation2d());
    Translation2d fieldPosition = robotPose.transformBy(robotToTarget).getTranslation();

    // Magnitude already verified > 0.01 above, so Rotation2d(x, y) is safe here
    Rotation2d angleToTarget = new Rotation2d(robotRelativeX, robotRelativeY);

    return Optional.of(new ObjectDetection(
        type,
        fieldPosition,
        target.getArea() / 100.0,
        distance,
        angleToTarget,
        timestamp));
  }

  private ObjectType classifyTarget(PhotonTrackedTarget target) {
    // PhotonVision class IDs from neural network (update class IDs when 2026 pipeline is trained)
    int classId = target.getFiducialId();  // Using getFiducialId as proxy for classId
    
    switch (classId) {
      case 0:
        return ObjectType.GAME_PIECE;
      default:
        return ObjectType.UNKNOWN;
    }
  }

  private double getTargetHeight(ObjectType type) {
    switch (type) {
      case GAME_PIECE:
        return GAME_PIECE_HEIGHT_METERS;
      default:
        return 0.1; // Default small height
    }
  }

  // Returns all detected objects in field coordinates
  public List<ObjectDetection> getDetectedObjects() {
    return new ArrayList<>(detectedObjects);
  }

  // Returns the closest detected target of any type
  public Optional<ObjectDetection> getClosestTarget() {
    return closestTarget;
  }

  // Returns the closest detected target of the given type
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

  // Returns field-relative position of the closest target
  public Optional<Translation2d> getClosestTargetPosition() {
    return closestTarget.map(ObjectDetection::getPosition);
  }

  // Returns robot-relative angle to the closest target (for driver feedback)
  public Optional<Rotation2d> getAngleToClosestTarget() {
    return closestTarget.map(ObjectDetection::getAngleToTarget);
  }
}
