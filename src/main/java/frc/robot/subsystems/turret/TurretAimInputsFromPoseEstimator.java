package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.function.Supplier;

// Supplies aim inputs using PoseEstimatorSubsystem data.
// robotPose is adjusted to the turret pivot location (not robot center) so bearing is accurate.
public class TurretAimInputsFromPoseEstimator implements Supplier<TurretAimInputs> {
  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem driveSubsystem;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final TurretAimInputs inputs = new TurretAimInputs();

  public TurretAimInputsFromPoseEstimator(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem) {
    this(poseEstimator, driveSubsystem, null);
  }

  public TurretAimInputsFromPoseEstimator(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem,
      Supplier<Pose2d> targetPoseSupplier) {
    this.poseEstimator = poseEstimator;
    this.driveSubsystem = driveSubsystem;
    this.targetPoseSupplier = targetPoseSupplier;
  }

  @Override
  public TurretAimInputs get() {
    Pose2d pose = poseEstimator.getEstimatedPose();

    // Shift from robot center to turret pivot — bearing must come from the actual launch point.
    // Offset is robot-relative, so rotate it by the robot heading to get field-relative delta.
    double heading = pose.getRotation().getRadians();
    double dx = Constants.Turret.TURRET_PIVOT_OFFSET_X_METERS;
    double dy = Constants.Turret.TURRET_PIVOT_OFFSET_Y_METERS;
    double pivotX = pose.getX() + dx * Math.cos(heading) - dy * Math.sin(heading);
    double pivotY = pose.getY() + dx * Math.sin(heading) + dy * Math.cos(heading);
    inputs.robotPose = new Pose2d(new Translation2d(pivotX, pivotY), pose.getRotation());

    inputs.targetPose = resolveTargetPose(pose);

    double vx = driveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond;
    double vy = driveSubsystem.getRobotRelativeSpeeds().vyMetersPerSecond;
    inputs.robotSpeedMetersPerSecond = Math.hypot(vx, vy);
    inputs.targetLatencySeconds = 0.0;
    return inputs;
  }

  private Pose2d resolveTargetPose(Pose2d fallbackPose) {
    if (targetPoseSupplier == null) {
      return fallbackPose;
    }

    Pose2d targetPose = targetPoseSupplier.get();
    return targetPose != null ? targetPose : fallbackPose;
  }
}
