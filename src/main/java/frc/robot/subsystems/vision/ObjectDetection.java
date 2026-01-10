package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// Container for detected game pieces - used by ObjectVisionSubsystem (object detection camera)
// Stores field-relative position of coral/algae for autonomous game piece pickup
// Currently set up for 2025 Reefscape - update ObjectType enum for 2026 game
public class ObjectDetection {
  // Game piece types - add 2026 game pieces here when game is revealed
  public enum ObjectType {
    CORAL,    // 2025: Orange coral game piece
    ALGAE,    // 2025: Green algae game piece
    UNKNOWN   // Unrecognized/low confidence detection
  }

  private final ObjectType type; // What kind of game piece
  private final Translation2d position;  // Field-relative position (X, Y in meters)
  private final double confidence; // Neural net confidence (0-1, higher = more certain)
  private final double distanceMeters; // Distance from robot to object
  private final Rotation2d angleToTarget; // Angle from robot heading to object
  private final double timestampSeconds; // When detection was made (for staleness checks)

  public ObjectDetection(
      ObjectType type,
      Translation2d position,
      double confidence,
      double distanceMeters,
      Rotation2d angleToTarget,
      double timestampSeconds) {
    this.type = type;
    this.position = position;
    this.confidence = confidence;
    this.distanceMeters = distanceMeters;
    this.angleToTarget = angleToTarget;
    this.timestampSeconds = timestampSeconds;
  }

  public ObjectType getType() {
    return type;
  }

  public Translation2d getPosition() {
    return position;
  }

  public double getConfidence() {
    return confidence;
  }

  public double getDistanceMeters() {
    return distanceMeters;
  }

  public Rotation2d getAngleToTarget() {
    return angleToTarget;
  }

  public double getTimestampSeconds() {
    return timestampSeconds;
  }

  @Override
  public String toString() {
    return String.format("%s @ (%.2f, %.2f) dist=%.2f conf=%.2f",
        type, position.getX(), position.getY(), distanceMeters, confidence);
  }
}
