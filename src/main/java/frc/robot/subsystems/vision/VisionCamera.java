package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;

public interface VisionCamera {

  public static class VisionResult {

    public final Pose2d estimatedPose;

    public final double timestampSeconds;

    public final int tagCount;

    public final double averageTagDistance;

    public final List<Integer> tagIds; // NEW: Track which tags were used

    public VisionResult(Pose2d estimatedPose, double timestampSeconds, int tagCount, double averageTagDistance, List<Integer> tagIds) {

      this.estimatedPose = estimatedPose;

      this.timestampSeconds = timestampSeconds;

      this.tagCount = tagCount;

      this.averageTagDistance = averageTagDistance;

      this.tagIds = tagIds;

    }

  }

  Optional<VisionResult> getLatestResult();

  String getName();

  boolean hasTarget();

}
