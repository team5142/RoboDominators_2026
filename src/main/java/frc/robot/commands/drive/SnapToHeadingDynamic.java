package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.SmartLogger;
import java.util.function.DoubleSupplier;

// Picks a heading based on robot pose at the moment the button is pressed, then
// delegates to SnapToHeadingFixed for the rest of the hold.
// Near/far walls use the same X boundaries as NeutralZoneSweepSimplifiedCommand.
// Robots within 1.5m of the left or right wall snap to face the near/far direction;
// robots in the middle snap to face left or right (parallel to the walls).
public class SnapToHeadingDynamic extends Command {
  private static final double GOVERNED_SPEED_MPS = 3.5;
  private static final double SPEED_SCALE = GOVERNED_SPEED_MPS / Constants.Swerve.MAX_TRANSLATION_SPEED_MPS;
  private static final double WALL_Y_THRESHOLD = 0.5; // meters from left/right wall
  private static final double DOUBLE_TAP_WINDOW_SEC = 1.0; // max gap between release and re-press to override

  // Static state persists across instances so double-tap works between separate command activations.
  private static double lastWallHeading = Double.NaN; // last heading chosen when near a wall
  private static double lastEndTimestamp = Double.NaN; // Timer.getFPGATimestamp() when command last ended
  private static final double LEFT_WALL_Y  = 7.248;
  private static final double RIGHT_WALL_Y = 0.624;
  private static final double CENTER_Y = (LEFT_WALL_Y + RIGHT_WALL_Y) / 2.0;

  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private SnapToHeadingFixed delegate = null;

  public SnapToHeadingDynamic(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this.poseEstimator = poseEstimator;
    this.driveSubsystem = driveSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    double heading = computeHeading();
    SmartLogger.logConsole("->DynSnap: heading=" + String.format("%.0f", heading) + "deg", "DynSnap");
    final double lockedHeading = heading;
    delegate = new SnapToHeadingFixed(
        driveSubsystem,
        () -> xSupplier.getAsDouble() * SPEED_SCALE,
        () -> ySupplier.getAsDouble() * SPEED_SCALE,
        () -> lockedHeading);
    delegate.initialize();
  }

  @Override
  public void execute() {
    if (delegate != null) delegate.execute();
  }

  @Override
  public void end(boolean interrupted) {
    if (delegate != null) delegate.end(interrupted);
    delegate = null;
    lastEndTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double computeHeading() {
    boolean isRed = DriverStation.getAlliance()
        .map(a -> a == DriverStation.Alliance.Red).orElse(false);

    Pose2d pose = poseEstimator.getEstimatedPose();
    double rawX = pose.getX();
    double robotX = isRed ? (Constants.Field.FIELD_LENGTH_METERS - rawX) : rawX;
    double robotY = pose.getY();

    double nearX = isRed ? nearXRed() : nearXBlue();
    double farX  = isRed ? farXRed()  : farXBlue();

    boolean inAllianceZone = robotX <= ALLIANCE_ZONE_FAR_X;
    boolean inCorridor     = robotX > ALLIANCE_ZONE_FAR_X && robotX < NEUTRAL_ZONE_NEAR_X;
    boolean inNeutralZone  = robotX >= NEUTRAL_ZONE_NEAR_X && robotX <= farX;

    // Trench/bump corridor - always face into the field (0 deg Blue, 180 deg Red)
    if (inCorridor) {
      return isRed ? 180.0 : 0.0;
    }

    if (inNeutralZone || !inAllianceZone) {
      return wallHeading(robotX, robotY, nearX, farX, isRed);
    } else {
      return allianceHeading(robotX, robotY, ALLIANCE_ZONE_FAR_X, isRed);
    }
  }

  // Near the left/right wall: snap to face near or far depending on which corner is closer.
  // Double-tap within DOUBLE_TAP_WINDOW_SEC overrides to the perpendicular (left/right) heading.
  // In the middle: face left or right to travel parallel to the walls.
  private double wallHeading(double robotX, double robotY, double nearX, double farX, boolean isRed) {
    boolean nearLeftWall  = (LEFT_WALL_Y  - robotY) < WALL_Y_THRESHOLD;
    boolean nearRightWall = (robotY - RIGHT_WALL_Y) < WALL_Y_THRESHOLD;

    if (nearLeftWall || nearRightWall) {
      double distToFarCorner  = Math.abs(robotX - farX);
      double distToNearCorner = Math.abs(robotX - nearX);
      double naturalHeading = (distToFarCorner < distToNearCorner)
          ? (isRed ? 0.0 : 180.0)   // toNear
          : (isRed ? 180.0 : 0.0);  // toFar

      // Double-tap: if re-pressed within window after a wall heading, give perpendicular instead.
      double timeSinceEnd = Timer.getFPGATimestamp() - lastEndTimestamp;
      boolean isDoubleTap = !Double.isNaN(lastEndTimestamp)
          && timeSinceEnd <= DOUBLE_TAP_WINDOW_SEC
          && !Double.isNaN(lastWallHeading);
      if (isDoubleTap) {
        lastWallHeading = Double.NaN;   // consume the double-tap
        lastEndTimestamp = Double.NaN;  // prevent any further triggers until next release
        SmartLogger.logConsole("->DynSnap: double-tap override to perpendicular", "DynSnap");
        return nearRightWall ? 90.0 : 270.0;
      }

      lastWallHeading = naturalHeading;
      return naturalHeading;
    }

    lastWallHeading = Double.NaN; // not a wall heading, clear double-tap state
    return robotY < CENTER_Y ? 90.0 : 270.0;
  }

  // In alliance zone: closer to back wall = face toFar (into field); closer to nearX = face toNear (back to alliance).
  // Double-tap within DOUBLE_TAP_WINDOW_SEC when near a wall overrides to the perpendicular (left/right) heading.
  private double allianceHeading(double robotX, double robotY, double nearX, boolean isRed) {
    boolean nearLeftWall  = (LEFT_WALL_Y  - robotY) < WALL_Y_THRESHOLD;
    boolean nearRightWall = (robotY - RIGHT_WALL_Y) < WALL_Y_THRESHOLD;

    if (nearLeftWall || nearRightWall) {
      double backWallX      = isRed ? Constants.Field.FIELD_LENGTH_METERS : 0.0;
      double distToBackWall = Math.abs(robotX - backWallX);
      double distToNearX    = Math.abs(robotX - nearX);
      double naturalHeading = (distToBackWall < distToNearX)
          ? (isRed ? 180.0 : 0.0)   // toFar - face into field
          : (isRed ? 0.0 : 180.0);  // toNear - face back toward alliance zone

      double timeSinceEnd = Timer.getFPGATimestamp() - lastEndTimestamp;
      boolean isDoubleTap = !Double.isNaN(lastEndTimestamp)
          && timeSinceEnd <= DOUBLE_TAP_WINDOW_SEC
          && !Double.isNaN(lastWallHeading);
      if (isDoubleTap) {
        lastWallHeading = Double.NaN;
        lastEndTimestamp = Double.NaN;
        SmartLogger.logConsole("->DynSnap: double-tap override to perpendicular", "DynSnap");
        return nearRightWall ? 90.0 : 270.0;
      }

      lastWallHeading = naturalHeading;
      return naturalHeading;
    }

    lastWallHeading = Double.NaN;
    return robotY < CENTER_Y ? 90.0 : 270.0;
  }

  // Blue-origin X boundaries (Red mirrors automatically via robotX transform).
  // ALLIANCE_ZONE_FAR_X and NEUTRAL_ZONE_NEAR_X define the trench/bump corridor between them.
  private static final double ALLIANCE_ZONE_FAR_X = 3.33; // far edge of alliance sweep area
  private static final double NEUTRAL_ZONE_NEAR_X = 5.6;  // near edge of neutral sweep area

  private static double nearXBlue() { return NEUTRAL_ZONE_NEAR_X; }
  private static double farXBlue()  { return RobotContainer.COMPETITION_MODE ? 11.574 : 9.024; }
  private static double nearXRed()  { return RobotContainer.COMPETITION_MODE ? 10.564 : 9.024; }
  private static double farXRed()   { return 5.976; }
}

