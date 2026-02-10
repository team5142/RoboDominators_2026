package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.SmartLogger;
import java.util.ArrayList;
import java.util.List;

// Prototype command for looping intake sweeps through the neutral zone.
// Path points are placeholders and should be updated once geometry is finalized.
public class NeutralZoneIntakeSweepCommand extends Command {
  public static class SweepConfig {
    public final double fieldLengthMeters;
    public final double fieldWidthMeters;
    public final double neutralZoneLengthMeters;
    public final double edgeMarginMeters;
    public final double centerLaneOffsetMeters;
    public final double edgeLaneOffsetMeters;
    public final PathConstraints pathConstraints;

    public SweepConfig(
        double fieldLengthMeters,
        double fieldWidthMeters,
        double neutralZoneLengthMeters,
        double edgeMarginMeters,
        double centerLaneOffsetMeters,
        double edgeLaneOffsetMeters,
        PathConstraints pathConstraints) {
      this.fieldLengthMeters = fieldLengthMeters;
      this.fieldWidthMeters = fieldWidthMeters;
      this.neutralZoneLengthMeters = neutralZoneLengthMeters;
      this.edgeMarginMeters = edgeMarginMeters;
      this.centerLaneOffsetMeters = centerLaneOffsetMeters;
      this.edgeLaneOffsetMeters = edgeLaneOffsetMeters;
      this.pathConstraints = pathConstraints;
    }
  }

  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem driveSubsystem;
  private final SweepConfig config;

  private List<Pose2d> sweepLoop = new ArrayList<>();
  private int currentIndex = 0;
  private Command activeCommand = null;

  public NeutralZoneIntakeSweepCommand(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem,
      SweepConfig config) {
    this.poseEstimator = poseEstimator;
    this.driveSubsystem = driveSubsystem;
    this.config = config;
  }

  @Override
  public void initialize() {
    sweepLoop = buildSweepLoop(config); // Build the loop of sweep targets.
    currentIndex = findNearestIndex(poseEstimator.getEstimatedPose(), sweepLoop); // Start closest to current pose.
    startNextSegment(); // Begin the first segment.
  }

  @Override
  public void execute() {
    if (activeCommand == null || !activeCommand.isScheduled()) {
      currentIndex = (currentIndex + 1) % sweepLoop.size(); // Advance to the next point in the loop.
      startNextSegment(); // Keep looping until canceled.
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      activeCommand.cancel(); // Stop any active path.
    }
    driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0)); // Stop the drivetrain.
    SmartLogger.logReplay("Sweep/Interrupted", interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void startNextSegment() {
    if (sweepLoop.isEmpty()) {
      return; // No targets to follow.
    }

    Pose2d targetPose = sweepLoop.get(currentIndex); // Current sweep target.
    activeCommand = AutoBuilder.pathfindToPose(targetPose, config.pathConstraints); // Let PathPlanner handle routing.
    CommandScheduler.getInstance().schedule(activeCommand);
    SmartLogger.logReplay("Sweep/TargetPose", targetPose);
  }

  private static int findNearestIndex(Pose2d currentPose, List<Pose2d> loop) {
    int bestIndex = 0;
    double bestDistance = Double.POSITIVE_INFINITY;

    for (int i = 0; i < loop.size(); i++) {
  double distance = currentPose.getTranslation().getDistance(loop.get(i).getTranslation()); // Compare to each target.
      if (distance < bestDistance) {
        bestDistance = distance;
        bestIndex = i;
      }
    }

    return bestIndex;
  }

  private static List<Pose2d> buildSweepLoop(SweepConfig config) {
    double centerX = config.fieldLengthMeters / 2.0;
    double neutralHalfLength = config.neutralZoneLengthMeters / 2.0;

    double neutralMinX = centerX - neutralHalfLength + config.edgeMarginMeters;
    double neutralMaxX = centerX + neutralHalfLength - config.edgeMarginMeters;

  double leftLaneY = config.edgeLaneOffsetMeters; // Near the left wall.
  double rightLaneY = config.fieldWidthMeters - config.edgeLaneOffsetMeters; // Near the right wall.
  double centerLaneY = (config.fieldWidthMeters / 2.0) + config.centerLaneOffsetMeters; // Slightly off center.

    List<Pose2d> loop = new ArrayList<>();

    loop.add(new Pose2d(neutralMinX, leftLaneY, Rotation2d.fromDegrees(0.0)));
    loop.add(new Pose2d(neutralMaxX, leftLaneY, Rotation2d.fromDegrees(0.0)));
    loop.add(new Pose2d(neutralMaxX, centerLaneY, Rotation2d.fromDegrees(180.0)));
    loop.add(new Pose2d(neutralMinX, centerLaneY, Rotation2d.fromDegrees(180.0)));
    loop.add(new Pose2d(neutralMinX, rightLaneY, Rotation2d.fromDegrees(0.0)));
    loop.add(new Pose2d(neutralMaxX, rightLaneY, Rotation2d.fromDegrees(0.0)));
    loop.add(new Pose2d(neutralMaxX, centerLaneY, Rotation2d.fromDegrees(180.0)));
    loop.add(new Pose2d(neutralMinX, centerLaneY, Rotation2d.fromDegrees(180.0)));

    return applyAllianceMirroring(loop, config.fieldLengthMeters); // Flip for red alliance if needed.
  }

  private static List<Pose2d> applyAllianceMirroring(List<Pose2d> loop, double fieldLengthMeters) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.get() == DriverStation.Alliance.Blue) {
      return loop; // Blue is the default field frame.
    }

    List<Pose2d> mirrored = new ArrayList<>();
    for (Pose2d pose : loop) {
      mirrored.add(mirrorPoseForRed(pose, fieldLengthMeters));
    }
    return mirrored;
  }

  private static Pose2d mirrorPoseForRed(Pose2d bluePose, double fieldLengthMeters) {
    double mirroredX = fieldLengthMeters - bluePose.getX(); // Flip across field length.
    Rotation2d mirroredRotation = bluePose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0)); // Face downfield.
    return new Pose2d(mirroredX, bluePose.getY(), mirroredRotation);
  }

  public static SweepConfig createPrototypeConfig() {
    double fieldLengthMeters = 17.55;
    double fieldWidthMeters = 8.05;
    double neutralZoneLengthMeters = Units.inchesToMeters(240.0);
    double edgeMarginMeters = Units.inchesToMeters(3.0); // Small buffer from obstacles.
    double centerLaneOffsetMeters = Units.inchesToMeters(18.0); // Center lane spacing.
    double edgeLaneOffsetMeters = Units.inchesToMeters(18.0); // Wall lane spacing.

    PathConstraints constraints = new PathConstraints(
        3.0,
        3.0,
        Math.toRadians(360.0),
        Math.toRadians(540.0));

    return new SweepConfig(
        fieldLengthMeters,
        fieldWidthMeters,
        neutralZoneLengthMeters,
        edgeMarginMeters,
        centerLaneOffsetMeters,
        edgeLaneOffsetMeters,
        constraints);
  }
}