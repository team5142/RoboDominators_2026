package frc.robot.commands.drive;

// TODO (next session with robot):
// Same issues as AllianceZoneSweepSimplifiedCommand - review all three sweep files together:
// 1. SpinToHeadingCommand is copied verbatim here - extract to a shared file.
// 2. SweepSegment.move() takes a 'start' parameter that is never used - remove it.
// 3. toLeft() and toRight() take a boolean isRed they never use - remove the parameter.
// 4. formatPose() duplicates SmartLogger.formatPose() - replace with the shared util call.

import static frc.robot.Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SEC;
import static frc.robot.Constants.Field.FIELD_LENGTH_METERS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.SmartLogger;
import java.util.ArrayList;
import java.util.List;

// Same loop shape as AllianceZoneSweepSimplifiedCommand but mirrored to the far
// (opposing alliance) side of the field. All X coordinates are reflected across
// the field center. Y and heading logic are unchanged.
public class OpposingAllianceZoneSweepSimplifiedCommand extends Command {
  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem driveSubsystem;
  private final PathConstraints pathConstraints;

  private List<SweepSegment> segments;
  private int currentSegmentIndex = 0;
  private Command activeCommand = null;
  private boolean isInEntryPhase = true;
  private static int sessionCounter = 0;
  private int sessionId = 0;

  private static final double FIELD_WIDTH  = 8.0427; // 316.64 in - 2026 AndyMark perimeter (confirmed)

  public OpposingAllianceZoneSweepSimplifiedCommand(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem) {
    this.poseEstimator = poseEstimator;
    this.driveSubsystem = driveSubsystem;
    this.pathConstraints = new PathConstraints(2.0, 2.0, Math.toRadians(360.0), Math.toRadians(540.0));
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    sessionId = ++sessionCounter;
    // isRed flips the mirror: if we are Red, the opposing zone is the Blue zone (low X).
    // The coord helpers mirror AllianceZoneSweep's X values across the field center,
    // so we pass the same isRed flag and the mirroring is applied on top.
    boolean isRed = DriverStation.getAlliance()
        .map(a -> a == DriverStation.Alliance.Red).orElse(false);
    segments = buildSegments(isRed);
    currentSegmentIndex = 0;
    isInEntryPhase = true;

    Pose2d currentPose = poseEstimator.getEstimatedPose();
    Pose2d entryPose = findNearestEntryCorner(currentPose, isRed);

    double distanceToEntry = currentPose.getTranslation().getDistance(entryPose.getTranslation());
    Rotation2d currentHeading = driveSubsystem.getGyroRotation();
    double headingErrorDeg = Math.abs(currentHeading.minus(entryPose.getRotation()).getDegrees());

    if (distanceToEntry > 0.3 || headingErrorDeg > 5.0) {
      SmartLogger.logConsole(
          "->OZSWEEP: Session " + sessionId + " | Entry to " + formatPose(entryPose)
              + " (dist=" + String.format("%.2f", distanceToEntry) + "m)",
          "OZSweep");
      activeCommand = AutoBuilder.pathfindToPose(entryPose, pathConstraints);
      CommandScheduler.getInstance().schedule(activeCommand);
    } else {
      SmartLogger.logConsole(
          "->OZSWEEP: Session " + sessionId + " | Already at entry, starting sweep",
          "OZSweep");
      isInEntryPhase = false;
      scheduleNextSegment();
    }
  }

  @Override
  public void execute() {
    if (activeCommand == null || !activeCommand.isScheduled()) {
      if (isInEntryPhase) {
        SmartLogger.logConsole("->OZSWEEP: Entry complete, starting sweep", "OZSweep");
        isInEntryPhase = false;
        scheduleNextSegment();
      } else {
        currentSegmentIndex = (currentSegmentIndex + 1) % segments.size();
        scheduleNextSegment();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      activeCommand.cancel();
    }
    driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    SmartLogger.logConsole("->OZSWEEP: Session " + sessionId + " ended", "OZSweep");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void scheduleNextSegment() {
    SweepSegment segment = segments.get(currentSegmentIndex);

    if (segment.isSpin) {
      activeCommand = new SpinToHeadingCommand(driveSubsystem, segment.targetHeading);
      SmartLogger.logConsole(
          "->OZSWEEP: Spin to " + String.format("%.0f", segment.targetHeading.getDegrees()) + "°",
          "OZSweep");
    } else {
      Rotation2d intendedHeading = segment.endPose.getRotation();
      Rotation2d currentHeading = driveSubsystem.getGyroRotation();
      double headingErrorDeg = Math.abs(currentHeading.minus(intendedHeading).getDegrees());

      Pose2d moveTarget = new Pose2d(segment.endPose.getTranslation(), intendedHeading);
      Command moveCommand = AutoBuilder.pathfindToPose(moveTarget, pathConstraints);

      if (headingErrorDeg > 5.0) {
        activeCommand = new SequentialCommandGroup(
            new SpinToHeadingCommand(driveSubsystem, intendedHeading),
            moveCommand);
        SmartLogger.logConsole(
            "->OZSWEEP: Pre-align to " + String.format("%.0f", intendedHeading.getDegrees())
                + "° then move to " + formatPose(segment.endPose),
            "OZSweep");
      } else {
        activeCommand = moveCommand;
        SmartLogger.logConsole(
            "->OZSWEEP: Move to " + formatPose(segment.endPose),
            "OZSweep");
      }
    }

    CommandScheduler.getInstance().schedule(activeCommand);
  }

  private Pose2d findNearestEntryCorner(Pose2d currentPose, boolean isRed) {
    double fX = farX(isRed);
    double nX = nearX(isRed);
    double bX = backupX(isRed);
    double lY = leftY();
    double rY = rightY();

    Pose2d[] corners = {
      new Pose2d(fX, rY, Rotation2d.fromDegrees(toLeft(isRed))),
      new Pose2d(fX, lY, Rotation2d.fromDegrees(toNear(isRed))),
      new Pose2d(bX, lY, Rotation2d.fromDegrees(toRight(isRed))),
      new Pose2d(nX, rY, Rotation2d.fromDegrees(toFar(isRed))),
    };
    int[] segIndices = { 0, 1, 4, 10 };

    int best = 0;
    double bestDist = currentPose.getTranslation().getDistance(corners[0].getTranslation());
    for (int i = 1; i < corners.length; i++) {
      double d = currentPose.getTranslation().getDistance(corners[i].getTranslation());
      if (d < bestDist) {
        bestDist = d;
        best = i;
      }
    }

    currentSegmentIndex = segIndices[best];
    return corners[best];
  }

  // ---- Coordinate helpers ----
  // All values mirror AllianceZoneSweepSimplifiedCommand using FIELD_LENGTH_METERS - x and FIELD_WIDTH - y.
  // Source safe values: nearX=0.876m, backupX=2.134m, leftY=7.175m, rightY=0.876m, towerPassY=2.134m.
  // Tower footprint on opposing side: same 44x47in square, mirrored to high-X, high-Y corner.

  // Far edge of opposing alliance zone (neutral zone boundary on the far side)
  private static double farX(boolean isRed) {
    return isRed ? 3.3 : (FIELD_LENGTH_METERS - 3.3);
  }

  // Near edge: deepest point toward the opposing driver station wall.
  // Intake faces wall — must be >= 34.5in = 0.876m from wall.
  private static double nearX(boolean isRed) {
    return isRed ? 0.876 : (FIELD_LENGTH_METERS - 0.876);
  }

  // Backup X: must clear opposing tower far face + turning radius + margin = 2.134m (84in).
  private static double backupX(boolean isRed) {
    return isRed ? 2.134 : (FIELD_LENGTH_METERS - 2.134);
  }

  // Y walls mirrored: opposing tower is on HIGH-Y side so leftY/rightY swap roles.
  // Source leftY=7.175 → mirrored rightY=FIELD_WIDTH-7.175=0.876 (34.5in from right wall) ✅
  // Source rightY=0.876 → mirrored leftY=FIELD_WIDTH-0.876=7.175 (34.5in from left wall) ✅
  private static double leftY()  { return FIELD_WIDTH - 7.175; } // 0.876m — mirrored, 34.5in from right wall
  private static double rightY() { return FIELD_WIDTH - 0.876; } // 7.175m — mirrored, 34.5in from left wall

  // Tower pass Y mirrored: source towerPassY=2.134m → FIELD_WIDTH-2.134=5.909m
  private static double towerPassY() { return FIELD_WIDTH - 2.134; }

  // Headings are swapped vs. alliance zone because the loop runs in the opposite Y direction.
  private static double toLeft(boolean isRed)  { return 270.0; } // toward mirrored "left" (low-Y)
  private static double toRight(boolean isRed) { return 90.0;  } // toward mirrored "right" (high-Y)
  private static double toFar(boolean isRed)   { return isRed ? 0.0   : 180.0; }
  private static double toNear(boolean isRed)  { return isRed ? 180.0 : 0.0;   }

  private List<SweepSegment> buildSegments(boolean isRed) {
    double fX = farX(isRed);
    double nX = nearX(isRed);
    double bX = backupX(isRed);
    double lY = leftY();
    double rY = rightY();
    double tY = towerPassY();

    double tFar   = toFar(isRed);
    double tNear  = toNear(isRed);
    double tLeft  = toLeft(isRed);
    double tRight = toRight(isRed);

    List<SweepSegment> segs = new ArrayList<>();

    // Leg 1: right wall
    segs.add(SweepSegment.move(p(fX, rY, tLeft), p(fX, lY, tLeft)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(tNear)));

    // Leg 2: left wall toward opposing driver station
    segs.add(SweepSegment.move(p(fX, lY, tNear), p(nX, lY, tNear)));

    // Leg 3: back up to avoid outpost
    segs.add(SweepSegment.move(p(nX, lY, tNear), p(bX, lY, tNear)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(tRight)));

    // Leg 4: past tower
    segs.add(SweepSegment.move(p(bX, lY, tRight), p(bX, tY, tRight)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(tNear)));

    // Leg 5: second near-wall approach
    segs.add(SweepSegment.move(p(bX, tY, tNear), p(nX, tY, tNear)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(tRight)));

    // Leg 6: right wall lower section
    segs.add(SweepSegment.move(p(nX, tY, tRight), p(nX, rY, tRight)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(tFar)));

    // Leg 7: right wall return to start
    segs.add(SweepSegment.move(p(nX, rY, tFar), p(fX, rY, tFar)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(tLeft)));

    return segs;
  }

  private static Pose2d p(double x, double y, double deg) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(deg));
  }

  private String formatPose(Pose2d pose) {
    return String.format("(%.2f, %.2f, %.0f°)",
        pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  private static class SweepSegment {
    final boolean isSpin;
    final Pose2d endPose;
    final Rotation2d targetHeading;

    private SweepSegment(boolean isSpin, Pose2d endPose, Rotation2d targetHeading) {
      this.isSpin = isSpin;
      this.endPose = endPose;
      this.targetHeading = targetHeading;
    }

    static SweepSegment move(Pose2d start, Pose2d end) {
      return new SweepSegment(false, end, null);
    }

    static SweepSegment spin(Rotation2d heading) {
      return new SweepSegment(true, null, heading);
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
          5.0, 0.0, 0.1,
          new TrapezoidProfile.Constraints(
              MAX_ANGULAR_SPEED_RAD_PER_SEC,
              MAX_ANGULAR_SPEED_RAD_PER_SEC * 2.0));
      this.headingController.setTolerance(Math.toRadians(2.0));
      addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
      double currentRad = driveSubsystem.getGyroRotation().getRadians();
      double shortestArc = MathUtil.angleModulus(targetHeading.getRadians() - currentRad);
      headingController.reset(currentRad);
      headingController.setGoal(currentRad + shortestArc);
    }

    @Override
    public void execute() {
      double currentRad = driveSubsystem.getGyroRotation().getRadians();
      double omega = headingController.calculate(currentRad);
      omega = MathUtil.clamp(omega, -MAX_ANGULAR_SPEED_RAD_PER_SEC, MAX_ANGULAR_SPEED_RAD_PER_SEC);
      driveSubsystem.drive(0.0, 0.0, omega, true);
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
}
