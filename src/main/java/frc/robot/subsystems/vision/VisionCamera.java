
package frc.robot.subsystems.vision;



import edu.wpi.first.math.geometry.Pose2d;

import java.util.Optional;



public interface VisionCamera {

  public static class VisionResult {

    public final Pose2d estimatedPose;

    public final double timestampSeconds;

    public final int tagCount;

    public final double averageTagDistance;



    public VisionResult(Pose2d estimatedPose, double timestampSeconds, int tagCount, double averageTagDistance) {

      this.estimatedPose = estimatedPose;

      this.timestampSeconds = timestampSeconds;

      this.tagCount = tagCount;

      this.averageTagDistance = averageTagDistance;

    }

  }



  Optional<VisionResult> getLatestResult();

  String getName();

  boolean hasTarget();

}
