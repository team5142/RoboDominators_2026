package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

// Fixed field targets for turret aim
public enum TurretFixedTarget {
  REEF_TAG_17(Constants.TurretTargets.REEF_TAG_17),
  REEF_TAG_18(Constants.TurretTargets.REEF_TAG_18),
  REEF_TAG_21(Constants.TurretTargets.REEF_TAG_21),
  REEF_TAG_22(Constants.TurretTargets.REEF_TAG_22),
  TAG_12(Constants.TurretTargets.TAG_12),
  TAG_16(Constants.TurretTargets.TAG_16);

  private final Pose2d pose;

  TurretFixedTarget(Pose2d pose) {
    this.pose = pose;
  }

  public Pose2d getPose() {
    return pose;
  }
}
