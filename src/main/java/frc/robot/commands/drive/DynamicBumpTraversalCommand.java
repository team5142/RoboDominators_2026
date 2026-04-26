package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.SmartLogger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// TODO (next session with robot):
// 1. Field geometry verified from 2026 AndyMark manual:
//    - Field: 650.12 x 316.64 in (corrected from 691 in length)
//    - Bump start X: ~159 in (code uses 160.0 in, <1 in off - OK)
//    - Bump depth: 44.4 in (exact match)
//    - Bump center Y: 98.36 in from side wall (code uses 99.0 in, 0.64 in off - OK)
//    - Y layout (right to left): 49.86 trench + 12 base + 73 bump + 47 hub + 73 bump + 12 + 49.86
//    - Bump peak angle: 15 deg, downhillPitchThreshold=5.0 deg should trigger reliably
// 2. Verify staging entry heading (-45 deg) on the actual robot at the bump.

// Drives the robot across the bump between alliance zones.
// Uses a 3-pose plan: staging -> bump midpoint -> exit.
// Gyro pitch detects when the robot crests the bump and switches to a softer descent profile.
public class DynamicBumpTraversalCommand extends Command {
  public enum Side {
    LEFT,
    RIGHT
  }

  public static class BumpConfig {
    public final double fieldLengthMeters;
    public final double fieldWidthMeters;
    public final double allianceZoneLengthMeters;
    public final double bumpStartXMeters;
    public final double bumpDepthMeters;
    public final double bumpCenterOffsetMeters;
    public final double stagingClearanceMeters;
    public final double robotHalfLengthMeters;
    public final double intakeExtensionMeters;
    public final PathConstraints pathConstraints;
    public final PathConstraints downhillConstraints;
    public final double downhillPitchThresholdDeg;

    public BumpConfig(
        double fieldLengthMeters,
        double fieldWidthMeters,
        double allianceZoneLengthMeters,
        double bumpStartXMeters,
        double bumpDepthMeters,
        double bumpCenterOffsetMeters,
        double stagingClearanceMeters,
        double robotHalfLengthMeters,
        double intakeExtensionMeters,
        PathConstraints pathConstraints,
        PathConstraints downhillConstraints,
        double downhillPitchThresholdDeg) {
      this.fieldLengthMeters = fieldLengthMeters;
      this.fieldWidthMeters = fieldWidthMeters;
      this.allianceZoneLengthMeters = allianceZoneLengthMeters;
      this.bumpStartXMeters = bumpStartXMeters;
      this.bumpDepthMeters = bumpDepthMeters;
      this.bumpCenterOffsetMeters = bumpCenterOffsetMeters;
      this.stagingClearanceMeters = stagingClearanceMeters;
      this.robotHalfLengthMeters = robotHalfLengthMeters;
      this.intakeExtensionMeters = intakeExtensionMeters;
      this.pathConstraints = pathConstraints;
      this.downhillConstraints = downhillConstraints;
      this.downhillPitchThresholdDeg = downhillPitchThresholdDeg;
    }
  }

  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem driveSubsystem;
  private final Side side;
  private final boolean modifierRequested;
  private final BumpConfig config;

  private boolean uphillSignCaptured = false;
  private double uphillPitchSign = 1.0;

  private Command activeCommand;

  public DynamicBumpTraversalCommand(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem,
      Side side,
      boolean modifierRequested,
      BumpConfig config,
      IntakeSubsystem intakeSubsystem) {
    this.poseEstimator = poseEstimator;
    this.driveSubsystem = driveSubsystem;
    this.side = side;
    this.modifierRequested = modifierRequested;
    this.config = config;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = poseEstimator.getEstimatedPose();
    TraversalPlan plan = buildTraversalPlan(currentPose, side, modifierRequested, config);

    uphillSignCaptured = false;
    uphillPitchSign = 1.0;

    double passThroughVelocity = 1.5;
    // Goal velocity 0: PathPlanner fully decelerates and stops at staging before crossing.
    // A short settle wait lets oscillation damp so the robot is truly centered on the bump Y.
    Command toStaging = AutoBuilder.pathfindToPose(
        plan.stagingPose,
        config.pathConstraints,
        0.0);
    Command settle = Commands.waitSeconds(0.20);
    Command toMid = AutoBuilder.pathfindToPose(
        plan.midPose,
        config.pathConstraints,
        passThroughVelocity);
    Command waitForDownhill = new WaitUntilCommand(this::hasPitchFlipped);
    Command toExit = AutoBuilder.pathfindToPose(plan.exitPose, config.downhillConstraints);

    // Lift intake slightly at uphill start and again at downhill start to clear the 15 deg slope.
    Command liftUphill  = Commands.none();
    Command liftDownhill = Commands.none();

    activeCommand = new SequentialCommandGroup(
        toStaging,
        settle,
        Commands.race(Commands.sequence(liftUphill, toMid), waitForDownhill),
        liftDownhill,
        toExit);
    CommandScheduler.getInstance().schedule(activeCommand);

    SmartLogger.logReplay("BumpTraversal/Side", side.toString());
    SmartLogger.logReplay("BumpTraversal/StagingPose", plan.stagingPose);
    SmartLogger.logReplay("BumpTraversal/MidPose", plan.midPose);
    SmartLogger.logReplay("BumpTraversal/ExitPose", plan.exitPose);
  }

  @Override
  public void execute() {
    if (activeCommand != null && !activeCommand.isScheduled()) {
      activeCommand = null;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      activeCommand.cancel();
    }
    driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    SmartLogger.logReplay("BumpTraversal/Interrupted", interrupted);
  }

  @Override
  public boolean isFinished() {
    return activeCommand == null;
  }

  private static class TraversalPlan {
    private final Pose2d stagingPose;
    private final Pose2d midPose;
    private final Pose2d exitPose;

    private TraversalPlan(Pose2d stagingPose, Pose2d midPose, Pose2d exitPose) {
      this.stagingPose = stagingPose;
      this.midPose = midPose;
      this.exitPose = exitPose;
    }
  }

  private static TraversalPlan buildTraversalPlan(
      Pose2d currentPose,
      Side side,
      boolean modifierRequested,
      BumpConfig config) {
    double fieldLength = config.fieldLengthMeters;
    double fieldWidth = config.fieldWidthMeters;
    double allianceEndX = config.allianceZoneLengthMeters;
    double opposingStartX = fieldLength - config.allianceZoneLengthMeters;

    double neutralMinX = allianceEndX;
    double neutralMaxX = opposingStartX;

    double bumpCenterY = (side == Side.LEFT)
        ? (fieldWidth - config.bumpCenterOffsetMeters)
        : config.bumpCenterOffsetMeters;

    boolean inAllianceZone = currentPose.getX() < neutralMinX;
    boolean inNeutralZone = currentPose.getX() >= neutralMinX && currentPose.getX() <= neutralMaxX;

    boolean goToOpposing = inNeutralZone && modifierRequested;
    boolean travelTowardNeutral = inAllianceZone;
    boolean travelTowardOpposing = goToOpposing;

    double bumpStartX = config.bumpStartXMeters;
    double bumpEndX = bumpStartX + config.bumpDepthMeters;
    double bumpMidX = bumpStartX + (config.bumpDepthMeters / 2.0);

    double stagingOffset = config.stagingClearanceMeters
        + config.robotHalfLengthMeters
        + config.intakeExtensionMeters;
    double exitOffset = stagingOffset;

    double stagingX;
    double exitX;
    Rotation2d stagingHeading;
    Rotation2d exitHeading;

    if (travelTowardNeutral) {
      stagingX = bumpStartX - stagingOffset;
      exitX = bumpEndX + exitOffset;
      stagingHeading = Rotation2d.fromDegrees(-45.0); // Enter at a diagonal.
      exitHeading = Rotation2d.fromDegrees(0.0); // Face downfield for blue.
    } else if (travelTowardOpposing) {
      stagingX = bumpEndX + stagingOffset;
      exitX = bumpStartX - exitOffset;
      stagingHeading = Rotation2d.fromDegrees(135.0); // Diagonal from the far side.
      exitHeading = Rotation2d.fromDegrees(180.0); // Face downfield for red.
    } else {
      stagingX = bumpEndX + stagingOffset;
      exitX = bumpStartX - exitOffset;
      stagingHeading = Rotation2d.fromDegrees(135.0); // Neutral to alliance default.
      exitHeading = Rotation2d.fromDegrees(180.0); // Face downfield for red.
    }

    Rotation2d midHeading = stagingHeading;
    Pose2d stagingPose = new Pose2d(stagingX, bumpCenterY, stagingHeading);
    Pose2d midPose = new Pose2d(bumpMidX, bumpCenterY, midHeading);
    Pose2d exitPose = new Pose2d(exitX, bumpCenterY, exitHeading);

    return applyAllianceMirroring(new TraversalPlan(stagingPose, midPose, exitPose), fieldLength, fieldWidth); // Flip for red.
  }

  private static TraversalPlan applyAllianceMirroring(TraversalPlan plan, double fieldLengthMeters, double fieldWidthMeters) {
    if (!RobotContainer.isRedAlliance()) {
      return plan; // Blue frame is the default.
    }

    Pose2d staging = mirrorPoseForRed(plan.stagingPose, fieldLengthMeters, fieldWidthMeters);
    Pose2d mid = mirrorPoseForRed(plan.midPose, fieldLengthMeters, fieldWidthMeters);
    Pose2d exit = mirrorPoseForRed(plan.exitPose, fieldLengthMeters, fieldWidthMeters);
    return new TraversalPlan(staging, mid, exit);
  }

  private static Pose2d mirrorPoseForRed(Pose2d bluePose, double fieldLengthMeters, double fieldWidthMeters) {
    double mirroredX = fieldLengthMeters - bluePose.getX();
    double mirroredY = fieldWidthMeters - bluePose.getY(); // Rotational symmetry requires Y flip too.
    Rotation2d mirroredRotation = bluePose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0));
    return new Pose2d(mirroredX, mirroredY, mirroredRotation);
  }

  private boolean hasPitchFlipped() {
    double pitchDegrees = driveSubsystem.getGyroPitchDegrees();
    double threshold = config.downhillPitchThresholdDeg;

    if (!uphillSignCaptured) {
      if (Math.abs(pitchDegrees) >= threshold) {
        uphillSignCaptured = true;
        uphillPitchSign = Math.signum(pitchDegrees);
      }
      return false;
    }

    return pitchDegrees * uphillPitchSign <= -threshold;
  }

  public static BumpConfig createPrototypeConfig() {
  double fieldLengthMeters = Constants.Field.FIELD_LENGTH_METERS;
  double fieldWidthMeters = Constants.Field.FIELD_WIDTH_METERS;
  double allianceZoneLengthMeters = Units.inchesToMeters(158.60); // Alliance zone depth.
  double bumpStartXMeters = Units.inchesToMeters(160.0); // Bump start from alliance wall.
  double bumpDepthMeters = Units.inchesToMeters(44.4); // Bump depth across X.
  double bumpCenterOffsetMeters = Units.inchesToMeters(98.36); // Right bump center from right wall.
  // Y layout: 49.86 trench + 12 base + 73 bump + 47 hub + 73 bump + 12 + 49.86
  // Right bump center = 49.86 + 12 + 73/2 = 98.36in. Left = fieldWidth - 98.36in (mirrored).
  // Robot width=38in centered on 73in bump gives 17.5in clearance to each bump edge.
  double stagingClearanceMeters = Units.inchesToMeters(12.0); // Desired bumper clearance.
  double robotHalfLengthMeters = Units.inchesToMeters(17.0); // Half of robot length with bumpers.
  double intakeExtensionMeters = Units.inchesToMeters(12.0); // Intake extension used for clearance.

  PathConstraints constraints = new PathConstraints(
    3.75,
    3.75,
    Math.toRadians(270.0),
    Math.toRadians(360.0)); // Conservative speeds for early testing.

  PathConstraints downhillConstraints = new PathConstraints(
    3.75,
    2.5,
    Math.toRadians(270.0),
    Math.toRadians(360.0)); // Softer acceleration on descent.

  double downhillPitchThresholdDeg = 5.0;

    return new BumpConfig(
        fieldLengthMeters,
        fieldWidthMeters,
        allianceZoneLengthMeters,
        bumpStartXMeters,
        bumpDepthMeters,
        bumpCenterOffsetMeters,
        stagingClearanceMeters,
        robotHalfLengthMeters,
        intakeExtensionMeters,
        constraints,
        downhillConstraints,
        downhillPitchThresholdDeg);
  }
}