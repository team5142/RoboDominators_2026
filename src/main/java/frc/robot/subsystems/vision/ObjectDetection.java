package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a detected game piece or object in field coordinates
 */
public class ObjectDetection {
  public enum ObjectType {
    CORAL,
    ALGAE,
    UNKNOWN
  }

  private final ObjectType type;
  private final Translation2d position;  // Field-relative position
  private final double confidence;
  private final double distanceMeters;
  private final Rotation2d angleToTarget;
  private final double timestampSeconds;

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
