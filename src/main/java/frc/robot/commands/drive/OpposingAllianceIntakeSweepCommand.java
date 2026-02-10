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

// Prototype command for looping intake sweeps through the opposing alliance zone.
// Path points are placeholders and should be updated once geometry is finalized.
public class OpposingAllianceIntakeSweepCommand extends Command {
  public static class SweepConfig {
    public final double fieldLengthMeters;
    public final double fieldWidthMeters;
    public final double allianceZoneLengthMeters;
    public final double edgeMarginMeters;
    public final double laneOffsetMeters;
    public final PathConstraints pathConstraints;

    public SweepConfig(
        double fieldLengthMeters,
        double fieldWidthMeters,
        double allianceZoneLengthMeters,
        double edgeMarginMeters,
        double laneOffsetMeters,
        PathConstraints pathConstraints) {
      this.fieldLengthMeters = fieldLengthMeters;
      this.fieldWidthMeters = fieldWidthMeters;
      this.allianceZoneLengthMeters = allianceZoneLengthMeters;
      this.edgeMarginMeters = edgeMarginMeters;
      this.laneOffsetMeters = laneOffsetMeters;
      this.pathConstraints = pathConstraints;
    }
  }

  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem driveSubsystem;
  private final SweepConfig config;

  private List<Pose2d> sweepLoop = new ArrayList<>();
  private int currentIndex = 0;
  private Command activeCommand = null;

  public OpposingAllianceIntakeSweepCommand(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem,
      SweepConfig config) {
    this.poseEstimator = poseEstimator;
    this.driveSubsystem = driveSubsystem;
    this.config = config;
  }

  @Override
  public void initialize() {
    sweepLoop = buildSweepLoop(config); // Build lane targets for the opposing zone.
    currentIndex = findNearestIndex(poseEstimator.getEstimatedPose(), sweepLoop); // Start closest to the robot.
    startNextSegment(); // Begin the loop.
  }

  @Override
  public void execute() {
    if (activeCommand == null || !activeCommand.isScheduled()) {
      currentIndex = (currentIndex + 1) % sweepLoop.size(); // Move to the next lane point.
      startNextSegment(); // Continue the sweep.
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      activeCommand.cancel(); // Stop path following.
    }
    driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0)); // Stop the drivetrain.
    SmartLogger.logReplay("OpposingSweep/Interrupted", interrupted);
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
    activeCommand = AutoBuilder.pathfindToPose(targetPose, config.pathConstraints); // Use PathPlanner routing.
    CommandScheduler.getInstance().schedule(activeCommand);
    SmartLogger.logReplay("OpposingSweep/TargetPose", targetPose);
  }

  private static int findNearestIndex(Pose2d currentPose, List<Pose2d> loop) {
    int bestIndex = 0;
    double bestDistance = Double.POSITIVE_INFINITY;

    for (int i = 0; i < loop.size(); i++) {
      double distance = currentPose.getTranslation().getDistance(loop.get(i).getTranslation());
      if (distance < bestDistance) {
        bestDistance = distance;
        bestIndex = i;
      }
    }

    return bestIndex;
  }

  private static List<Pose2d> buildSweepLoop(SweepConfig config) {
    double maxX = config.fieldLengthMeters - config.edgeMarginMeters;
    double minX = config.fieldLengthMeters - config.allianceZoneLengthMeters + config.edgeMarginMeters;

  double laneLow = config.laneOffsetMeters; // Low wall lane.
  double laneMidLow = Units.inchesToMeters(118.0); // Below tower band.
  double laneMidHigh = Units.inchesToMeters(177.0); // Above tower band.
  double laneHigh = config.fieldWidthMeters - config.laneOffsetMeters; // High wall lane.

    List<Pose2d> loop = new ArrayList<>();

    loop.add(new Pose2d(maxX, laneLow, Rotation2d.fromDegrees(180.0)));
    loop.add(new Pose2d(minX, laneLow, Rotation2d.fromDegrees(180.0)));
    loop.add(new Pose2d(minX, laneMidLow, Rotation2d.fromDegrees(0.0)));
    loop.add(new Pose2d(maxX, laneMidLow, Rotation2d.fromDegrees(0.0)));
    loop.add(new Pose2d(maxX, laneMidHigh, Rotation2d.fromDegrees(180.0)));
    loop.add(new Pose2d(minX, laneMidHigh, Rotation2d.fromDegrees(180.0)));
    loop.add(new Pose2d(minX, laneHigh, Rotation2d.fromDegrees(0.0)));
    loop.add(new Pose2d(maxX, laneHigh, Rotation2d.fromDegrees(0.0)));

    return applyAllianceMirroring(loop, config.fieldLengthMeters); // Flip for red alliance.
  }

  private static List<Pose2d> applyAllianceMirroring(List<Pose2d> loop, double fieldLengthMeters) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.get() == DriverStation.Alliance.Blue) {
      return loop; // Blue frame is the default.
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
    double fieldWidthMeters = 8.043;
    double allianceZoneLengthMeters = Units.inchesToMeters(158.60);
  double edgeMarginMeters = Units.inchesToMeters(3.0); // Small buffer from walls.
  double laneOffsetMeters = Units.inchesToMeters(16.0); // Lane spacing from edges.

    PathConstraints constraints = new PathConstraints(
        3.0,
        3.0,
        Math.toRadians(360.0),
        Math.toRadians(540.0));

    return new SweepConfig(
        fieldLengthMeters,
        fieldWidthMeters,
        allianceZoneLengthMeters,
        edgeMarginMeters,
        laneOffsetMeters,
        constraints);
  }
}