package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.function.Supplier;

// Supplies aim inputs using PoseEstimatorSubsystem data
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
    inputs.robotPose = pose;
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
