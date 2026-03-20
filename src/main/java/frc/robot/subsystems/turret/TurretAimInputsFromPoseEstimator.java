package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  // Low-pass filter on heading — only active when nearly stopped to suppress QuestNav standstill jitter.
  // During rotation the raw heading is used directly so the turret doesn't lag behind.
  private static final double HEADING_FILTER_ALPHA = 0.10;
  // Dead zone: ignore heading changes smaller than this at standstill — rejects QuestNav noise.
  private static final double HEADING_DEADZONE_RAD = Math.toRadians(0.25);
  // Omega above which we bypass the filter entirely and use raw heading.
  private static final double HEADING_FILTER_BYPASS_RPS = 0.15; // ~9 deg/s
  private double filteredHeadingRad = Double.NaN; // NaN = not yet initialized
  private int lastSeedGeneration = -1; // tracks pose reseeds to reset heading filter + latch

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
    if (!poseEstimator.isInitialized()) {
      return null;
    }

    // Detect pose reseeds — reset heading filter so stale filtered heading doesn't persist.
    int currentGen = poseEstimator.getSeedGeneration();
    boolean reseeded = currentGen != lastSeedGeneration;
    if (reseeded) {
      lastSeedGeneration = currentGen;
      filteredHeadingRad = Double.NaN;
    }
    inputs.poseReseeded = reseeded;

    Pose2d pose = poseEstimator.getEstimatedPose();
    // Cache once — used for heading filter, speed magnitude, omega, and field-relative velocity.
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();

    // Low-pass filter the heading to suppress QuestNav standstill jitter.
    // Bypassed during rotation so the turret doesn't lag behind the actual heading.
    double rawHeadingRad = pose.getRotation().getRadians();
    double omega = Math.abs(speeds.omegaRadiansPerSecond);
    if (Double.isNaN(filteredHeadingRad)) {
      filteredHeadingRad = rawHeadingRad;
    } else if (omega >= HEADING_FILTER_BYPASS_RPS) {
      // Robot is rotating — track raw heading directly, keep filter state in sync.
      filteredHeadingRad = rawHeadingRad;
    } else {
      // Nearly stopped — apply low-pass + dead zone to suppress QuestNav jitter.
      double delta = rawHeadingRad - filteredHeadingRad;
      while (delta >  Math.PI) delta -= 2.0 * Math.PI;
      while (delta < -Math.PI) delta += 2.0 * Math.PI;
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

    inputs.targetPose = resolveTargetPose();

    // Cache speeds once — used for magnitude, omega, and field-relative rotation below.
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;
    inputs.robotSpeedMetersPerSecond = Math.hypot(vx, vy);
    inputs.robotOmegaRadPerSecond = speeds.omegaRadiansPerSecond; // signed: CCW positive
    // Rotate robot-relative velocity to field-relative for Phase 4 lead compensation.
    double headingRad = pose.getRotation().getRadians();
    inputs.robotFieldVxMetersPerSecond = vx * Math.cos(headingRad) - vy * Math.sin(headingRad);
    inputs.robotFieldVyMetersPerSecond = vx * Math.sin(headingRad) + vy * Math.cos(headingRad);
    inputs.targetLatencySeconds = 0.0;
    return inputs;
  }

  // Returns the target pose from the supplier, or null if unavailable.
  // Callers (TurretAimSolver) already handle null by disabling the goal.
  private Pose2d resolveTargetPose() {
    if (targetPoseSupplier == null) return null;
    return targetPoseSupplier.get();
  }
}
