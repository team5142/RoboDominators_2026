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

// CCW sweep of the alliance zone edges, avoiding the tower and outpost.
// Coordinates are measured in Blue field frame; Red is handled via explicit named values.
// Loop shape (Blue): right wall → left wall → near wall → back up → past tower →
//   near wall again → right wall → back to start.
public class AllianceZoneSweepSimplifiedCommand extends Command {
  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem driveSubsystem;
  private final PathConstraints pathConstraints;

  private List<SweepSegment> segments;
  private int currentSegmentIndex = 0;
  private Command activeCommand = null;
  private boolean isInEntryPhase = true;
  private boolean justScheduled = false;
  private static int sessionCounter = 0;
  private int sessionId = 0;

  public AllianceZoneSweepSimplifiedCommand(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem) {
    this.poseEstimator = poseEstimator;
    this.driveSubsystem = driveSubsystem;
    this.pathConstraints = new PathConstraints(2.0, 2.0, Math.toRadians(360.0), Math.toRadians(540.0));
    // Claim driveSubsystem so whileTrue can interrupt and cancel activeCommand on release.
    // SpinToHeadingCommand must NOT also claim it to avoid a conflict.
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    sessionId = ++sessionCounter;
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
          "->AZSWEEP: Session " + sessionId + " | Entry to " + formatPose(entryPose)
              + " (dist=" + String.format("%.2f", distanceToEntry) + "m)",
          "AZSweep");
      activeCommand = AutoBuilder.pathfindToPose(entryPose, pathConstraints);
      activeCommand = activeCommand.asProxy();
      justScheduled = true;
      CommandScheduler.getInstance().schedule(activeCommand);
    } else {
      SmartLogger.logConsole(
          "->AZSWEEP: Session " + sessionId + " | Already at entry, starting sweep",
          "AZSweep");
      isInEntryPhase = false;
      scheduleNextSegment();
    }
  }

  @Override
  public void execute() {
    if (justScheduled) {
      justScheduled = false;
      return;
    }
    if (activeCommand == null || !activeCommand.isScheduled()) {
      if (isInEntryPhase) {
        SmartLogger.logConsole("->AZSWEEP: Entry complete, starting sweep", "AZSweep");
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
    SmartLogger.logConsole("->AZSWEEP: Session " + sessionId + " ended", "AZSweep");
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
          "->AZSWEEP: Spin to " + String.format("%.0f", segment.targetHeading.getDegrees()) + "°",
          "AZSweep");
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
            "->AZSWEEP: Pre-align to " + String.format("%.0f", intendedHeading.getDegrees())
                + "° then move to " + formatPose(segment.endPose),
            "AZSweep");
      } else {
        activeCommand = moveCommand;
        SmartLogger.logConsole(
            "->AZSWEEP: Move to " + formatPose(segment.endPose),
            "AZSweep");
      }
    }

    activeCommand = activeCommand.asProxy();
    justScheduled = true;
    CommandScheduler.getInstance().schedule(activeCommand);
  }

  // Entry corners are pose 1 (right wall) and pose 7 (right wall near driver station).
  // Both are on the right/low-Y side. We pick whichever is closer.
  private Pose2d findNearestEntryCorner(Pose2d currentPose, boolean isRed) {
    // Four corners of the loop, each mapped to the segment index that follows entry there.
    // A: far/right — start of leg 1 (index 0)
    // B: far/left  — start of leg 2 spin (index 1)
    // C: near/left — start of leg 4 spin (index 4)
    // D: near/right — start of leg 7 spin (index 10)
    double fX = farX(isRed);
    double nX = nearX(isRed);
    double bX = backupX(isRed);
    double lY = leftY();
    double rY = rightY();

    Pose2d[] corners = {
      new Pose2d(fX, rY, Rotation2d.fromDegrees(toLeft(isRed))),  // A: far/right
      new Pose2d(fX, lY, Rotation2d.fromDegrees(toNear(isRed))),  // B: far/left
      new Pose2d(bX, lY, Rotation2d.fromDegrees(toRight(isRed))), // C: near/left (backup X avoids outpost)
      new Pose2d(nX, rY, Rotation2d.fromDegrees(toFar(isRed))),   // D: near/right
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

  // ---- Coordinate helpers (all in Blue field frame, Red uses mirrored X) ----

  // X at far edge of alliance zone (neutral zone boundary = start of loop on trench side)
  private static double farX(boolean isRed) {
    return isRed ? (RobotContainer.COMPETITION_MODE ? 16.540 - 3.333 : 16.540 - 3.333) : 3.333;
  }

  // X at near edge: deepest point robot drives toward driver station wall
  private static double nearX(boolean isRed) {
    return isRed ? (16.540 - 0.540) : 0.540;
  }

  // X at backup point: robot backs away from driver station wall to avoid outpost
  private static double backupX(boolean isRed) {
    return isRed ? (16.540 - 1.530) : 1.530;
  }

  // Y of left wall (high-Y side) — same for both alliances
  private static double leftY() { return 7.285; }

  // Y of right wall (low-Y side) — same for both alliances
  private static double rightY() { return 0.620; }

  // Y of tower/outpost avoidance: robot passes through this Y corridor to avoid the tower.
  // Measured at Y=2.633 (just past the tower on the right/low-Y side).
  private static double towerPassY() { return 2.633; }

  // Headings — Y axis is alliance-independent so toLeft/toRight are always the same.
  // toFar/toNear flip with alliance.
  private static double toLeft(boolean isRed)  { return 90.0;  }  // toward high-Y wall
  private static double toRight(boolean isRed) { return 270.0; }  // toward low-Y wall
  private static double toFar(boolean isRed)   { return isRed ? 180.0 : 0.0; }   // away from driver station
  private static double toNear(boolean isRed)  { return isRed ? 0.0   : 180.0; } // toward driver station

  private List<SweepSegment> buildSegments(boolean isRed) {
    double fX   = farX(isRed);
    double nX   = nearX(isRed);
    double bX   = backupX(isRed);
    double lY   = leftY();
    double rY   = rightY();
    double tY   = towerPassY();

    double tFar   = toFar(isRed);
    double tNear  = toNear(isRed);
    double tLeft  = toLeft(isRed);
    double tRight = toRight(isRed);

    List<SweepSegment> segs = new ArrayList<>();

    // Leg 1: right wall — pose 1 to pose 2 (facing left, sweeping low-Y → high-Y)
    segs.add(SweepSegment.move(p(fX, rY, tLeft), p(fX, lY, tLeft)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(tNear)));

    // Leg 2: left wall — pose 2 to pose 3 (facing near wall, sweep to driver station)
    segs.add(SweepSegment.move(p(fX, lY, tNear), p(nX, lY, tNear)));
    // No spin — back up in place (PathPlanner pathfindToPose handles the reverse)

    // Leg 3: back up from driver station wall to avoid outpost — pose 3 to pose 4
    // Robot stays facing near (180 Blue / 0 Red), just moves away from wall
    segs.add(SweepSegment.move(p(nX, lY, tNear), p(bX, lY, tNear)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(tRight)));

    // Leg 4: past tower — pose 4 to pose 5 (facing right wall, sweep high-Y → tower pass Y)
    segs.add(SweepSegment.move(p(bX, lY, tRight), p(bX, tY, tRight)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(tNear)));

    // Leg 5: second near-wall approach — pose 5 to pose 6 (facing near wall)
    segs.add(SweepSegment.move(p(bX, tY, tNear), p(nX, tY, tNear)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(tRight)));

    // Leg 6: right wall lower section — pose 6 to pose 7 (facing right wall, high side → low side)
    segs.add(SweepSegment.move(p(nX, tY, tRight), p(nX, rY, tRight)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(tFar)));

    // Leg 7: right wall return — pose 7 back to pose 1 (facing far/neutral zone)
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

  // Identical to NeutralZoneSweepSimplifiedCommand - shortest-arc spin controller
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
      // No requirement here - the outer AllianceZoneSweepSimplifiedCommand owns driveSubsystem.
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
