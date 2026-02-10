package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.function.Supplier;

// Selects hub or pass targets based on robot pose
public class TurretTargetSelector implements Supplier<Pose2d> {
  private final PoseEstimatorSubsystem poseEstimator;

  public TurretTargetSelector(PoseEstimatorSubsystem poseEstimator) {
    this.poseEstimator = poseEstimator;
  }

  @Override
  public Pose2d get() {
    Pose2d robotPose = poseEstimator.getEstimatedPose();
    boolean isRed = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);

    boolean inAllianceZone = isRed
        ? robotPose.getX() > Constants.Field.FIELD_LENGTH_METERS - Constants.Field.ALLIANCE_ZONE_LENGTH_METERS
        : robotPose.getX() < Constants.Field.ALLIANCE_ZONE_LENGTH_METERS;

    if (inAllianceZone) {
      return isRed ? Constants.HubCenters.RED_HUB_CENTER : Constants.HubCenters.BLUE_HUB_CENTER;
    }

    boolean onLeftSide = robotPose.getY() > Constants.Field.FIELD_WIDTH_METERS / 2.0;
    if (onLeftSide) {
      return isRed ? Constants.PassTargets.RED_PASS_TARGET_LEFT : Constants.PassTargets.BLUE_PASS_TARGET_LEFT;
    }

    return isRed ? Constants.PassTargets.RED_PASS_TARGET_RIGHT : Constants.PassTargets.BLUE_PASS_TARGET_RIGHT;
  }
}
