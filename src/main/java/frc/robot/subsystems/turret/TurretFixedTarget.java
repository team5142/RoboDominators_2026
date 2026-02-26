package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

// Fixed field targets for turret aim
public enum TurretFixedTarget {
  HUB_LEFT(Constants.TurretTargets.BLUE_PASS_TARGET_LEFT);
 

  private final Pose2d pose;

  TurretFixedTarget(Pose2d pose) {
    this.pose = pose;
  }

  public Pose2d getPose() {
    return pose;
  }
}
