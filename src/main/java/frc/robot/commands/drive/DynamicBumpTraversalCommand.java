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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.SmartLogger;

// Prototype command for dynamic bump traversal.
// Uses PathPlanner to reach a staging pose, cross the bump, and exit downfield.
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
        PathConstraints pathConstraints) {
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
    }
  }

  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem driveSubsystem;
  private final Side side;
  private final boolean modifierRequested;
  private final BumpConfig config;

  private Command activeCommand;

  public DynamicBumpTraversalCommand(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem,
      Side side,
      boolean modifierRequested,
      BumpConfig config) {
    this.poseEstimator = poseEstimator;
    this.driveSubsystem = driveSubsystem;
    this.side = side;
    this.modifierRequested = modifierRequested;
    this.config = config;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = poseEstimator.getEstimatedPose(); // Start from current robot pose.
    TraversalPlan plan = buildTraversalPlan(currentPose, side, modifierRequested, config); // Compute staging and exit.

  Command toStaging = AutoBuilder.pathfindToPose(plan.stagingPose, config.pathConstraints); // Drive to entry pose.
  Command toMid = AutoBuilder.pathfindToPose(plan.midPose, config.pathConstraints); // Cross the bump midpoint.
  Command toExit = AutoBuilder.pathfindToPose(plan.exitPose, config.pathConstraints); // Leave the bump safely.

    activeCommand = new SequentialCommandGroup(toStaging, toMid, toExit);
    CommandScheduler.getInstance().schedule(activeCommand);

    SmartLogger.logReplay("BumpTraversal/Side", side.toString());
    SmartLogger.logReplay("BumpTraversal/StagingPose", plan.stagingPose);
    SmartLogger.logReplay("BumpTraversal/MidPose", plan.midPose);
    SmartLogger.logReplay("BumpTraversal/ExitPose", plan.exitPose);
  }

  @Override
  public void execute() {
    if (activeCommand != null && !activeCommand.isScheduled()) {
      activeCommand = null; // Mark done when the sequence finishes.
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      activeCommand.cancel(); // Stop any active path.
    }
    driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0)); // Stop the drivetrain.
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
  double allianceEndX = config.allianceZoneLengthMeters; // End of the near alliance zone.
  double opposingStartX = fieldLength - config.allianceZoneLengthMeters; // Start of the far alliance zone.

    double neutralMinX = allianceEndX;
    double neutralMaxX = opposingStartX;

  double bumpCenterY = (side == Side.LEFT)
    ? (fieldWidth - config.bumpCenterOffsetMeters)
    : config.bumpCenterOffsetMeters; // Pick left or right bump center.

  boolean inAllianceZone = currentPose.getX() < neutralMinX; // Near side of the bump.
  boolean inNeutralZone = currentPose.getX() >= neutralMinX && currentPose.getX() <= neutralMaxX; // Between zones.

  boolean goToOpposing = inNeutralZone && modifierRequested; // Modifier only works in neutral.
  boolean travelTowardNeutral = inAllianceZone; // Default when starting in alliance zone.
  boolean travelTowardOpposing = goToOpposing; // Only true with modifier.

  double bumpStartX = config.bumpStartXMeters; // Leading edge of the bump.
  double bumpEndX = bumpStartX + config.bumpDepthMeters; // Far edge of the bump.
  double bumpMidX = bumpStartX + (config.bumpDepthMeters / 2.0); // Midpoint used for rotation timing.

  double stagingOffset = config.stagingClearanceMeters
    + config.robotHalfLengthMeters
    + config.intakeExtensionMeters; // Keep the bumper and intake away from the bump.
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

  Rotation2d midHeading = stagingHeading; // Hold angle through the bump.
    Pose2d stagingPose = new Pose2d(stagingX, bumpCenterY, stagingHeading);
    Pose2d midPose = new Pose2d(bumpMidX, bumpCenterY, midHeading);
    Pose2d exitPose = new Pose2d(exitX, bumpCenterY, exitHeading);

    return applyAllianceMirroring(new TraversalPlan(stagingPose, midPose, exitPose), fieldLength); // Flip for red.
  }

  private static TraversalPlan applyAllianceMirroring(TraversalPlan plan, double fieldLengthMeters) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.get() == DriverStation.Alliance.Blue) {
      return plan; // Blue frame is the default.
    }

    Pose2d staging = mirrorPoseForRed(plan.stagingPose, fieldLengthMeters);
    Pose2d mid = mirrorPoseForRed(plan.midPose, fieldLengthMeters);
    Pose2d exit = mirrorPoseForRed(plan.exitPose, fieldLengthMeters);
    return new TraversalPlan(staging, mid, exit);
  }

  private static Pose2d mirrorPoseForRed(Pose2d bluePose, double fieldLengthMeters) {
    double mirroredX = fieldLengthMeters - bluePose.getX(); // Flip across field length.
    Rotation2d mirroredRotation = bluePose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0)); // Face downfield.
    return new Pose2d(mirroredX, bluePose.getY(), mirroredRotation);
  }

  public static BumpConfig createPrototypeConfig() {
  double fieldLengthMeters = 17.55; // Field length in meters.
  double fieldWidthMeters = Units.inchesToMeters(316.64); // Field width in meters.
  double allianceZoneLengthMeters = Units.inchesToMeters(158.60); // Alliance zone depth.
  double bumpStartXMeters = Units.inchesToMeters(160.0); // Bump start from alliance wall.
  double bumpDepthMeters = Units.inchesToMeters(44.4); // Bump depth across X.
  double bumpCenterOffsetMeters = Units.inchesToMeters(99.0); // Center from side wall.
  double stagingClearanceMeters = Units.inchesToMeters(12.0); // Desired bumper clearance.
  double robotHalfLengthMeters = Units.inchesToMeters(12.5); // Half of robot length.
  double intakeExtensionMeters = Units.inchesToMeters(12.0); // Intake extension used for clearance.

  PathConstraints constraints = new PathConstraints(
    2.0,
    2.0,
    Math.toRadians(270.0),
    Math.toRadians(360.0)); // Conservative speeds for early testing.

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
        constraints);
  }
}