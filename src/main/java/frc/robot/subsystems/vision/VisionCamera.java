package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;

// Interface for vision cameras - allows TagVisionSubsystem to work with any camera type
// We support Limelight (NetworkTables) and PhotonVision (different APIs) using same interface
// This abstraction lets us mix camera types without TagVisionSubsystem caring about implementation
public interface VisionCamera {

  // Container for vision pose estimate - includes quality metrics for filtering
  public static class VisionResult {

    public final Pose2d estimatedPose; // Robot position on field from this camera
    public final double timestampSeconds; // When photo was taken (for latency compensation)
    public final int tagCount; // How many AprilTags were used (more = better)
    public final double averageTagDistance; // Average distance to tags (closer = better)
    public final List<Integer> tagIds; // Which tag IDs were detected (for debugging)

    public VisionResult(Pose2d estimatedPose, double timestampSeconds, int tagCount, double averageTagDistance, List<Integer> tagIds) {
      this.estimatedPose = estimatedPose;
      this.timestampSeconds = timestampSeconds;
      this.tagCount = tagCount;
      this.averageTagDistance = averageTagDistance;
      this.tagIds = tagIds;
    }
  }

  // Get latest pose estimate from camera - returns empty if no tags visible
  Optional<VisionResult> getLatestResult();

  // Get camera name for logging (e.g. "limelight-front", "RRTagPV")
  String getName();

  // Quick check if camera sees any targets (used for LED status)
  boolean hasTarget();
}
