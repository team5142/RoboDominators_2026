package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.function.Supplier;

// Supplies aim inputs using PoseEstimatorSubsystem data.
// robotPose is adjusted to the turret pivot location (not robot center) so bearing is accurate.
// Heading is low-pass filtered before entering the solver to suppress QuestNav jitter.
// XY position is NOT filtered — acceleration compensation (Phase 4) needs raw speed from drive.
public class TurretAimInputsFromPoseEstimator implements Supplier<TurretAimInputs> {
  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem driveSubsystem;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final TurretAimInputs inputs = new TurretAimInputs();

  // Low-pass filter on heading only — smooths QuestNav angular jitter without lagging XY position.
  // alpha=0.10: ~200ms time constant. Balances jitter rejection vs response speed.
  private static final double HEADING_FILTER_ALPHA = 0.10;
  // Dead zone: ignore heading changes smaller than this — rejects QuestNav noise at the source.
  // 0.25deg = 0.00436 rad. Changes larger than this (real robot rotation) pass through normally.
  private static final double HEADING_DEADZONE_RAD = Math.toRadians(0.25);
  private double filteredHeadingRad = Double.NaN; // NaN = not yet initialized

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

    // Low-pass filter the heading to suppress QuestNav angular jitter.
    // First call: seed the filter directly so there's no startup transient.
    double rawHeadingRad = pose.getRotation().getRadians();
    if (Double.isNaN(filteredHeadingRad)) {
      filteredHeadingRad = rawHeadingRad;
    } else {
      // Wrap-aware delta — handles the -pi/+pi discontinuity correctly.
      double delta = rawHeadingRad - filteredHeadingRad;
      while (delta >  Math.PI) delta -= 2.0 * Math.PI;
      while (delta < -Math.PI) delta += 2.0 * Math.PI;
      // Dead zone: ignore changes smaller than 0.25deg — rejects QuestNav standstill noise.
      // Only update the filter when the robot is actually rotating.
      if (Math.abs(delta) >= HEADING_DEADZONE_RAD) {
        filteredHeadingRad += HEADING_FILTER_ALPHA * delta;
      }
    }

    // Shift from robot center to turret pivot — bearing must come from the actual launch point.
    // Use filtered heading for pivot rotation but raw XY from pose (XY is already Kalman-smoothed).
    double dx = Constants.Turret.TURRET_PIVOT_OFFSET_X_METERS;
    double dy = Constants.Turret.TURRET_PIVOT_OFFSET_Y_METERS;
    double pivotX = pose.getX() + dx * Math.cos(filteredHeadingRad) - dy * Math.sin(filteredHeadingRad);
    double pivotY = pose.getY() + dx * Math.sin(filteredHeadingRad) + dy * Math.cos(filteredHeadingRad);
    inputs.robotPose = new Pose2d(new Translation2d(pivotX, pivotY), Rotation2d.fromRadians(filteredHeadingRad));

    inputs.targetPose = resolveTargetPose(pose);

    double vx = driveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond;
    double vy = driveSubsystem.getRobotRelativeSpeeds().vyMetersPerSecond;
    inputs.robotSpeedMetersPerSecond = Math.hypot(vx, vy);
    // Rotate robot-relative velocity to field-relative for Phase 4 lead compensation.
    double headingRad = pose.getRotation().getRadians();
    inputs.robotFieldVxMetersPerSecond = vx * Math.cos(headingRad) - vy * Math.sin(headingRad);
    inputs.robotFieldVyMetersPerSecond = vx * Math.sin(headingRad) + vy * Math.cos(headingRad);
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
