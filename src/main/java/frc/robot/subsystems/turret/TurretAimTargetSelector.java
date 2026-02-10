package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;

// Holds the current fixed target for turret aiming
public class TurretAimTargetSelector {
  private Pose2d targetPose;

  public TurretAimTargetSelector(Pose2d initialTarget) {
    this.targetPose = initialTarget;
  }

  public void setTarget(TurretFixedTarget target) {
    if (target != null) {
      this.targetPose = target.getPose();
    }
  }

  public void setTargetPose(Pose2d targetPose) {
    this.targetPose = targetPose;
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }
}
