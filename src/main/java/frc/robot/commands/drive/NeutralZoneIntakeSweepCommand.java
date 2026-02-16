package frc.robot.commands.drive;

import static frc.robot.Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SEC;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
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
    public final double robotHalfWidthMeters;
    public final boolean useMeasuredFieldLines;
    public final double measuredNearX;
    public final double measuredFarX;
    public final double measuredLeftY;
    public final double measuredCenterY;
    public final double measuredRightY;
    public final PathConstraints pathConstraints;

    public SweepConfig(
        double fieldLengthMeters,
        double fieldWidthMeters,
        double neutralZoneLengthMeters,
        double edgeMarginMeters,
        double centerLaneOffsetMeters,
        double edgeLaneOffsetMeters,
        double robotHalfWidthMeters,
        boolean useMeasuredFieldLines,
        double measuredNearX,
        double measuredFarX,
        double measuredLeftY,
        double measuredCenterY,
        double measuredRightY,
        PathConstraints pathConstraints) {
      this.fieldLengthMeters = fieldLengthMeters;
      this.fieldWidthMeters = fieldWidthMeters;
      this.neutralZoneLengthMeters = neutralZoneLengthMeters;
      this.edgeMarginMeters = edgeMarginMeters;
      this.centerLaneOffsetMeters = centerLaneOffsetMeters;
      this.edgeLaneOffsetMeters = edgeLaneOffsetMeters;
      this.robotHalfWidthMeters = robotHalfWidthMeters;
      this.useMeasuredFieldLines = useMeasuredFieldLines;
      this.measuredNearX = measuredNearX;
      this.measuredFarX = measuredFarX;
      this.measuredLeftY = measuredLeftY;
      this.measuredCenterY = measuredCenterY;
      this.measuredRightY = measuredRightY;
      this.pathConstraints = pathConstraints;
    }
  }

  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem driveSubsystem;
  private final SweepConfig config;

  private List<SweepTarget> sweepLoop = new ArrayList<>();
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
    Pose2d currentPose = poseEstimator.getEstimatedPose();
    currentIndex = findNearestIndex(currentPose, sweepLoop); // Start closest to current pose.
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

    SweepTarget target = sweepLoop.get(currentIndex); // Current sweep target.
    if (target.spinInPlace) {
      activeCommand = new SpinToHeadingCommand(driveSubsystem, target.pose.getRotation());
    } else {
      Pose2d currentPose = poseEstimator.getEstimatedPose();
      Rotation2d moveHeading = headingFromDelta(
          target.pose.getX() - currentPose.getX(),
          target.pose.getY() - currentPose.getY());
      Pose2d movePose = new Pose2d(target.pose.getTranslation(), moveHeading);
      Command moveCommand = AutoBuilder.pathfindToPose(movePose, config.pathConstraints); // Let PathPlanner handle routing.
      double headingErrorDeg = Math.abs(driveSubsystem.getGyroRotation().minus(moveHeading).getDegrees());
      Command alignedMove = moveCommand;
      if (headingErrorDeg > 2.0) {
        alignedMove = new SequentialCommandGroup(
            new SpinToHeadingCommand(driveSubsystem, moveHeading),
            moveCommand);
      }
      activeCommand = alignedMove;
    }
    CommandScheduler.getInstance().schedule(activeCommand);
    SmartLogger.logReplay("Sweep/TargetPose", target.pose);
    SmartLogger.logReplay("Sweep/SpinInPlace", target.spinInPlace);
  }

  private static int findNearestIndex(Pose2d currentPose, List<SweepTarget> loop) {
    int bestIndex = 0;
    double bestDistance = Double.POSITIVE_INFINITY;

    for (int i = 0; i < loop.size(); i++) {
      Pose2d targetPose = loop.get(i).pose;
  double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation()); // Compare to each target.
      if (distance < bestDistance) {
        bestDistance = distance;
        bestIndex = i;
      }
    }

    return bestIndex;
  }

  private static List<SweepTarget> buildSweepLoop(SweepConfig config) {
    double neutralMinX;
    double neutralMaxX;
    double leftLaneY;
    double rightLaneY;
    double centerLaneY;

    if (config.useMeasuredFieldLines) {
      neutralMinX = config.measuredNearX;
      neutralMaxX = config.measuredFarX;
      leftLaneY = config.measuredLeftY;
      rightLaneY = config.measuredRightY;
      centerLaneY = config.measuredCenterY;
    } else {
      double centerX = config.fieldLengthMeters / 2.0;
      double neutralHalfLength = config.neutralZoneLengthMeters / 2.0;

      double zoneMargin = config.edgeMarginMeters + config.robotHalfWidthMeters;
      double edgeLaneMargin = config.edgeLaneOffsetMeters + config.robotHalfWidthMeters;
      double centerLaneMargin = config.centerLaneOffsetMeters + config.robotHalfWidthMeters;

      neutralMinX = centerX - neutralHalfLength + zoneMargin;
      neutralMaxX = centerX + neutralHalfLength - zoneMargin;

      leftLaneY = edgeLaneMargin; // Near the left wall.
      rightLaneY = config.fieldWidthMeters - edgeLaneMargin; // Near the right wall.
      centerLaneY = (config.fieldWidthMeters / 2.0) + centerLaneMargin; // Slightly off center.
    }

  List<SweepTarget> loop = new ArrayList<>();
    List<double[]> points = new ArrayList<>();

    points.add(new double[] {neutralMinX, leftLaneY});
    points.add(new double[] {neutralMaxX, leftLaneY});
    points.add(new double[] {neutralMaxX, centerLaneY});
    points.add(new double[] {neutralMinX, centerLaneY});
    points.add(new double[] {neutralMinX, rightLaneY});
    points.add(new double[] {neutralMaxX, rightLaneY});
    points.add(new double[] {neutralMaxX, centerLaneY});
    points.add(new double[] {neutralMinX, centerLaneY});

    for (int i = 0; i < points.size(); i++) {
      double[] prev = points.get((i - 1 + points.size()) % points.size());
      double[] current = points.get(i);
      double[] next = points.get((i + 1) % points.size());
      Rotation2d incomingHeading = headingFromDelta(current[0] - prev[0], current[1] - prev[1]);
      Rotation2d outgoingHeading = headingFromDelta(next[0] - current[0], next[1] - current[1]);
      loop.add(new SweepTarget(new Pose2d(current[0], current[1], incomingHeading), false));
      if (Math.abs(incomingHeading.getDegrees() - outgoingHeading.getDegrees()) > 1e-3) {
        loop.add(new SweepTarget(new Pose2d(current[0], current[1], outgoingHeading), true));
      }
    }

    return applyAllianceMirroring(loop, config.fieldLengthMeters); // Flip for red alliance if needed.
  }

  private static List<SweepTarget> applyAllianceMirroring(List<SweepTarget> loop, double fieldLengthMeters) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.get() == DriverStation.Alliance.Blue) {
      return loop; // Blue is the default field frame.
    }

    List<SweepTarget> mirrored = new ArrayList<>();
    for (SweepTarget target : loop) {
      mirrored.add(new SweepTarget(mirrorPoseForRed(target.pose, fieldLengthMeters), target.spinInPlace));
    }
    return mirrored;
  }

  private static Pose2d mirrorPoseForRed(Pose2d bluePose, double fieldLengthMeters) {
    double mirroredX = fieldLengthMeters - bluePose.getX(); // Flip across field length.
    Rotation2d mirroredRotation = bluePose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0)); // Face downfield.
    return new Pose2d(mirroredX, bluePose.getY(), mirroredRotation);
  }

  private static Rotation2d headingFromDelta(double dx, double dy) {
    return Rotation2d.fromRadians(Math.atan2(dy, dx));
  }


  private static class SweepTarget {
    private final Pose2d pose;
    private final boolean spinInPlace;

    private SweepTarget(Pose2d pose, boolean spinInPlace) {
      this.pose = pose;
      this.spinInPlace = spinInPlace;
    }
  }

  private static class SpinToHeadingCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final Rotation2d targetHeading;
    private final ProfiledPIDController headingController;

    private SpinToHeadingCommand(DriveSubsystem driveSubsystem, Rotation2d targetHeading) {
      this.driveSubsystem = driveSubsystem;
      this.targetHeading = targetHeading;
      this.headingController = new ProfiledPIDController(
          5.0,
          0.0,
          0.1,
          new TrapezoidProfile.Constraints(
              MAX_ANGULAR_SPEED_RAD_PER_SEC,
              MAX_ANGULAR_SPEED_RAD_PER_SEC * 2.0));
      this.headingController.enableContinuousInput(-Math.PI, Math.PI);
      this.headingController.setTolerance(Math.toRadians(2.0));

      addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
      headingController.reset(driveSubsystem.getGyroRotation().getRadians());
      headingController.setGoal(targetHeading.getRadians());
    }

    @Override
    public void execute() {
      double currentHeadingRad = driveSubsystem.getGyroRotation().getRadians();
      double omegaRadPerSec = headingController.calculate(currentHeadingRad);
      omegaRadPerSec = MathUtil.clamp(
          omegaRadPerSec,
          -MAX_ANGULAR_SPEED_RAD_PER_SEC,
          MAX_ANGULAR_SPEED_RAD_PER_SEC);
      driveSubsystem.drive(0.0, 0.0, omegaRadPerSec, true);
    }

    @Override
    public void end(boolean interrupted) {
      driveSubsystem.drive(0.0, 0.0, 0.0, true);
    }

    @Override
    public boolean isFinished() {
      return headingController.atGoal();
    }
  }

  public static SweepConfig createPrototypeConfig() {
    double fullFieldLengthMeters = 17.55;
    double practiceFieldLengthMeters = (fullFieldLengthMeters / 2.0) + Units.inchesToMeters(58.0);
    double fieldLengthMeters = RobotContainer.COMPETITION_MODE
        ? fullFieldLengthMeters
        : practiceFieldLengthMeters;
    double fieldWidthMeters = 8.05;
    double neutralZoneLengthMeters = Units.inchesToMeters(240.0);
    double edgeMarginMeters = Units.inchesToMeters(3.0); // Buffer from the zone edge.
    double centerLaneOffsetMeters = Units.inchesToMeters(18.0); // Center lane spacing.
    double edgeLaneOffsetMeters = Units.inchesToMeters(18.0); // Wall lane spacing.
    double robotHalfWidthMeters = Units.inchesToMeters(16.5); // Half robot width with bumpers.
    boolean useMeasuredFieldLines = !RobotContainer.COMPETITION_MODE;
    double wallOffsetMeters = Units.inchesToMeters(3.0);
    double measuredNearX = 5.9 + wallOffsetMeters;
    double measuredFarX = 9.1 - wallOffsetMeters;
    double measuredLeftY = 7.4 - wallOffsetMeters;
    double measuredCenterY = 4.077;
    double measuredRightY = 0.700 + wallOffsetMeters;

    PathConstraints constraints = new PathConstraints(
  2.0,
  2.0,
        Math.toRadians(360.0),
        Math.toRadians(540.0));

    return new SweepConfig(
        fieldLengthMeters,
        fieldWidthMeters,
        neutralZoneLengthMeters,
        edgeMarginMeters,
        centerLaneOffsetMeters,
        edgeLaneOffsetMeters,
        robotHalfWidthMeters,
        useMeasuredFieldLines,
        measuredNearX,
        measuredFarX,
        measuredLeftY,
        measuredCenterY,
        measuredRightY,
        constraints);
  }
}