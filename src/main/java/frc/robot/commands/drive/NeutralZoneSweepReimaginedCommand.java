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

public class NeutralZoneSweepReimaginedCommand extends Command {
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
  private List<SegmentStart> segmentStarts = new ArrayList<>();
  private static int s_sweepSessionCounter = 0;
  private int sweepSessionId = 0;
  private int currentIndex = 0;
  private Command activeCommand = null;
  private Command pendingEntryCommand = null;
  private String activeLegLabel = "";

  public NeutralZoneSweepReimaginedCommand(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem,
      SweepConfig config) {
    this.poseEstimator = poseEstimator;
    this.driveSubsystem = driveSubsystem;
    this.config = config;
  }

  @Override
  public void initialize() {
    sweepSessionId = ++s_sweepSessionCounter;
    segmentStarts = new ArrayList<>();
    sweepLoop = buildSweepLoop(config, segmentStarts);
    Pose2d currentPose = poseEstimator.getEstimatedPose();
    EntryPlan entryPlan = buildEntryPlan(currentPose, segmentStarts, sweepLoop, config);
    currentIndex = entryPlan.nextIndex;
    
    double distanceToEntry = currentPose.getTranslation().getDistance(entryPlan.entryPose.getTranslation());
    double headingErrorDeg = Math.abs(driveSubsystem.getGyroRotation().minus(entryPlan.entryPose.getRotation()).getDegrees());
    
    if (distanceToEntry > 0.3 || headingErrorDeg > 5.0) {
      pendingEntryCommand = buildEntryCommand(currentPose, entryPlan.entryPose);
      SmartLogger.logConsole(
        "->NEUTRAL: Init session " + sweepSessionId
          + " | entryIndex=" + currentIndex
          + " | entryPose=" + SmartLogger.formatPose(entryPlan.entryPose)
          + " | needsEntry=true (dist=" + String.format("%.2f", distanceToEntry) + "m)",
        "NeutralSweep");
    } else {
      pendingEntryCommand = null;
      SmartLogger.logConsole(
        "->NEUTRAL: Init session " + sweepSessionId
          + " | entryIndex=" + currentIndex
          + " | entryPose=" + SmartLogger.formatPose(entryPlan.entryPose)
          + " | needsEntry=false (already at entry)",
        "NeutralSweep");
    }
    
    SmartLogger.logReplay("Sweep/EntryStartPose", entryPlan.entryPose);
    SmartLogger.logReplay("Sweep/EntryTargetIndex", entryPlan.nextIndex);
  SmartLogger.logReplay("Sweep/SessionId", sweepSessionId);
  SmartLogger.logReplay("Sweep/EntryCurrentIndex", currentIndex);
    startNextSegment();
  }

  @Override
  public void execute() {
    if (activeCommand == null || !activeCommand.isScheduled()) {
      if (!activeLegLabel.isEmpty()) {
        SmartLogger.logConsole("->NEUTRAL: Completed " + activeLegLabel, "NeutralSweep");
        activeLegLabel = "";
      }
      currentIndex = (currentIndex + 1) % sweepLoop.size();
      startNextSegment();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      activeCommand.cancel();
    }
    driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    SmartLogger.logReplay("Sweep/Interrupted", interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void startNextSegment() {
    if (sweepLoop.isEmpty()) {
      return;
    }

    if (pendingEntryCommand != null) {
      activeCommand = pendingEntryCommand;
      pendingEntryCommand = null;
      activeLegLabel = "entry move";
      SmartLogger.logConsole("->NEUTRAL: Start entry move", "NeutralSweep");
      SmartLogger.logReplay("Sweep/SessionId", sweepSessionId);
      SmartLogger.logReplay("Sweep/ScheduleIndex", -1);
      CommandScheduler.getInstance().schedule(activeCommand);
      SmartLogger.logReplay("Sweep/EntryMove", true);
      return;
    }

    int scheduledIndex = currentIndex;
    SweepTarget target = sweepLoop.get(currentIndex);
    SmartLogger.logReplay("Sweep/SessionId", sweepSessionId);
    SmartLogger.logReplay("Sweep/ScheduleIndex", scheduledIndex);
    SmartLogger.logConsole(
        "->NEUTRAL: Schedule index " + scheduledIndex
            + " | spin=" + target.spinInPlace
            + " | pose=" + SmartLogger.formatPose(target.pose),
        "NeutralSweep");
    if (target.spinInPlace) {
      activeCommand = new SpinToHeadingCommand(driveSubsystem, target.pose.getRotation());
      activeLegLabel = String.format(
          "spin to %.1f deg",
          target.pose.getRotation().getDegrees());
      SmartLogger.logConsole("->NEUTRAL: Start " + activeLegLabel, "NeutralSweep");
    } else {
      Pose2d movePose = target.pose;
      Rotation2d currentHeading = driveSubsystem.getGyroRotation();
      Pose2d moveTarget = new Pose2d(movePose.getTranslation(), currentHeading);
      Command moveCommand = AutoBuilder.pathfindToPose(moveTarget, config.pathConstraints);
      activeCommand = moveCommand;
      activeCommand = moveCommand;
      activeLegLabel = String.format(
          "move to (%.2f, %.2f) at %.1f deg",
          movePose.getX(),
          movePose.getY(),
          movePose.getRotation().getDegrees());
      SmartLogger.logConsole("->NEUTRAL: Start " + activeLegLabel, "NeutralSweep");
    }
    CommandScheduler.getInstance().schedule(activeCommand);
    SmartLogger.logReplay("Sweep/TargetPose", target.pose);
    SmartLogger.logReplay("Sweep/SpinInPlace", target.spinInPlace);
  }

  private static EntryPlan buildEntryPlan(
      Pose2d currentPose,
      List<SegmentStart> starts,
      List<SweepTarget> loop,
      SweepConfig config) {
    Pose2d nearLeftCorner = starts.get(0).startPose;
    Pose2d nearRightCorner = starts.get(4).startPose;
    
    double distToLeft = currentPose.getTranslation().getDistance(nearLeftCorner.getTranslation());
    double distToRight = currentPose.getTranslation().getDistance(nearRightCorner.getTranslation());
    
    Pose2d chosenEntryPose;
    int chosenStartIndex;
    
    if (distToLeft <= distToRight) {
      chosenEntryPose = nearLeftCorner;
      chosenStartIndex = findLoopIndexForPose(loop, nearLeftCorner);
    } else {
      chosenEntryPose = nearRightCorner;
      chosenStartIndex = findLoopIndexForPose(loop, nearRightCorner);
    }
    
    return new EntryPlan(chosenEntryPose, chosenStartIndex);
  }

  private static int findLoopIndexForPose(List<SweepTarget> loop, Pose2d targetPose) {
    for (int i = 0; i < loop.size(); i++) {
      SweepTarget target = loop.get(i);
      double distance = target.pose.getTranslation().getDistance(targetPose.getTranslation());
      if (distance < 0.1 && !target.spinInPlace) {
        return i;
      }
    }
    return 0;
  }

  private Command buildEntryCommand(Pose2d currentPose, Pose2d entryPose) {
    Rotation2d targetHeading = entryPose.getRotation();
    Rotation2d currentHeading = driveSubsystem.getGyroRotation();
    double headingErrorDeg = Math.abs(currentHeading.minus(targetHeading).getDegrees());
    
    Pose2d moveTarget = new Pose2d(entryPose.getTranslation(), currentHeading);
    Command entryMove = AutoBuilder.pathfindToPose(moveTarget, config.pathConstraints);
    
    if (headingErrorDeg > 2.0) {
      SmartLogger.logReplay("Sweep/EntrySpinHeading", targetHeading.getDegrees());
      return new SequentialCommandGroup(
          new SpinToHeadingCommand(driveSubsystem, targetHeading),
          entryMove);
    }
    SmartLogger.logReplay("Sweep/EntrySpinHeading", targetHeading.getDegrees());
    return entryMove;
  }

  private static List<SweepTarget> buildSweepLoop(
      SweepConfig config,
      List<SegmentStart> segmentStarts) {
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

      leftLaneY = edgeLaneMargin;
      rightLaneY = config.fieldWidthMeters - edgeLaneMargin;
      centerLaneY = (config.fieldWidthMeters / 2.0) + centerLaneMargin;
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
      double[] start = points.get(i);
      double[] end = points.get((i + 1) % points.size());
      double[] next = points.get((i + 2) % points.size());
      Rotation2d segmentHeading = headingFromDelta(end[0] - start[0], end[1] - start[1]);
      Rotation2d nextHeading = headingFromDelta(next[0] - end[0], next[1] - end[1]);
      int targetIndex = loop.size();
      loop.add(new SweepTarget(new Pose2d(end[0], end[1], segmentHeading), false));
      segmentStarts.add(new SegmentStart(new Pose2d(start[0], start[1], segmentHeading), targetIndex));
      if (Math.abs(segmentHeading.getDegrees() - nextHeading.getDegrees()) > 1e-3) {
        loop.add(new SweepTarget(new Pose2d(end[0], end[1], nextHeading), true));
      }
    }

    return applyAllianceMirroring(loop, segmentStarts, config.fieldLengthMeters);
  }

  private static List<SweepTarget> applyAllianceMirroring(
      List<SweepTarget> loop,
      List<SegmentStart> segmentStarts,
      double fieldLengthMeters) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.get() == DriverStation.Alliance.Blue) {
      return loop;
    }

    List<SweepTarget> mirrored = new ArrayList<>();
    for (SweepTarget target : loop) {
      mirrored.add(new SweepTarget(mirrorPoseForRed(target.pose, fieldLengthMeters), target.spinInPlace));
    }

    for (int i = 0; i < segmentStarts.size(); i++) {
      SegmentStart start = segmentStarts.get(i);
      segmentStarts.set(
          i,
          new SegmentStart(mirrorPoseForRed(start.startPose, fieldLengthMeters), start.targetIndex));
    }
    return mirrored;
  }

  private static Pose2d mirrorPoseForRed(Pose2d bluePose, double fieldLengthMeters) {
    double mirroredX = fieldLengthMeters - bluePose.getX();
    Rotation2d mirroredRotation = bluePose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0));
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

  private static class SegmentStart {
    private final Pose2d startPose;
    private final int targetIndex;

    private SegmentStart(Pose2d startPose, int targetIndex) {
      this.startPose = startPose;
      this.targetIndex = targetIndex;
    }
  }

  private static class EntryPlan {
    private final Pose2d entryPose;
    private final int nextIndex;

    private EntryPlan(Pose2d entryPose, int nextIndex) {
      this.entryPose = entryPose;
      this.nextIndex = nextIndex;
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
    double edgeMarginMeters = Units.inchesToMeters(3.0);
    double centerLaneOffsetMeters = Units.inchesToMeters(18.0);
    double edgeLaneOffsetMeters = Units.inchesToMeters(18.0);
    double robotHalfWidthMeters = Units.inchesToMeters(16.5);
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
