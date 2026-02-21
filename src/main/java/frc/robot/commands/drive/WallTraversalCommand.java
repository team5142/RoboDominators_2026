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

// Traverses a single wall of the current zone while the driver holds a DPAD button.
// Finds the nearest end of the requested wall, pathfinds there, then drives to the far end.
// Stops (isFinished = true) when the far end is reached — does not loop.
//
// DPAD Up    → FAR wall  (neutral zone boundary)
// DPAD Down  → NEAR wall (driver station wall; includes tower dogleg in alliance zone)
// DPAD Left  → LEFT wall (high-Y side)
// DPAD Right → RIGHT wall (low-Y side)
public class WallTraversalCommand extends Command {

  public enum Wall { FAR, NEAR, LEFT, RIGHT }

  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem drive;
  private final Wall wall;
  private final PathConstraints constraints;

  private List<TraversalSegment> segments;
  private int segIndex = 0;
  private Command activeCommand = null;
  private boolean isInEntry = true;
  private boolean finished = false;

  public WallTraversalCommand(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem drive,
      Wall wall) {
    this.poseEstimator = poseEstimator;
    this.drive = drive;
    this.wall = wall;
    this.constraints = new PathConstraints(
        2.0, 2.0, Math.toRadians(360.0), Math.toRadians(540.0));
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    finished = false;
    segIndex = 0;
    isInEntry = true;

    boolean isRed = DriverStation.getAlliance()
        .map(a -> a == DriverStation.Alliance.Red).orElse(false);
    boolean inAllianceZone = isInAllianceZone(isRed);
    boolean inOpposingZone = isInOpposingZone(isRed);

    segments = buildSegments(isRed, inAllianceZone, inOpposingZone);

    Pose2d current = poseEstimator.getEstimatedPose();
    // Choose which end of the wall to enter from — whichever is closer
    Pose2d entry = chooseEntry(current, segments);

    double dist = current.getTranslation().getDistance(entry.getTranslation());
    double headingErr = Math.abs(
        current.getRotation().minus(entry.getRotation()).getDegrees());

    if (dist > 0.3 || headingErr > 5.0) {
      SmartLogger.logConsole(
          "->WALL[" + wall + "]: Entry to " + fmt(entry)
              + " (dist=" + String.format("%.2f", dist) + "m)", "WallTraversal");
      activeCommand = AutoBuilder.pathfindToPose(entry, constraints);
      CommandScheduler.getInstance().schedule(activeCommand);
    } else {
      SmartLogger.logConsole(
          "->WALL[" + wall + "]: Already at entry, starting traversal", "WallTraversal");
      isInEntry = false;
      scheduleNext();
    }
  }

  @Override
  public void execute() {
    if (activeCommand == null || !activeCommand.isScheduled()) {
      if (isInEntry) {
        isInEntry = false;
        scheduleNext();
      } else {
        segIndex++;
        if (segIndex >= segments.size()) {
          finished = true;
          SmartLogger.logConsole("->WALL[" + wall + "]: Traversal complete", "WallTraversal");
        } else {
          scheduleNext();
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) activeCommand.cancel();
    drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    SmartLogger.logConsole("->WALL[" + wall + "]: ended (interrupted=" + interrupted + ")", "WallTraversal");
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  // ---- Segment execution ----

  private void scheduleNext() {
    TraversalSegment seg = segments.get(segIndex);
    if (seg.isSpin) {
      activeCommand = new SpinToHeadingCommand(drive, seg.heading);
      SmartLogger.logConsole(
          "->WALL[" + wall + "]: Spin to "
              + String.format("%.0f", seg.heading.getDegrees()) + "°", "WallTraversal");
    } else {
      Rotation2d heading = seg.pose.getRotation();
      double headingErr = Math.abs(
          drive.getGyroRotation().minus(heading).getDegrees());
      Command move = AutoBuilder.pathfindToPose(seg.pose, constraints);
      if (headingErr > 5.0) {
        activeCommand = new SequentialCommandGroup(
            new SpinToHeadingCommand(drive, heading), move);
        SmartLogger.logConsole(
            "->WALL[" + wall + "]: Pre-align then move to " + fmt(seg.pose), "WallTraversal");
      } else {
        activeCommand = move;
        SmartLogger.logConsole(
            "->WALL[" + wall + "]: Move to " + fmt(seg.pose), "WallTraversal");
      }
    }
    CommandScheduler.getInstance().schedule(activeCommand);
  }

  // ---- Entry selection ----
  // Picks the first or last pose in the segment list (whichever end is nearer).
  // Reorders segments and flips headings so the robot always faces its direction of travel.
  private Pose2d chooseEntry(Pose2d current, List<TraversalSegment> segs) {
    Pose2d firstPose = firstMovePose(segs, 0);
    Pose2d lastPose  = lastMovePose(segs);

    double distFirst = current.getTranslation().getDistance(firstPose.getTranslation());
    double distLast  = current.getTranslation().getDistance(lastPose.getTranslation());

    if (distLast < distFirst) {
      // Robot is closer to the far end — reverse list and flip headings to match new travel direction
      java.util.Collections.reverse(segs);
      for (int i = 0; i < segs.size(); i++) {
        segs.set(i, segs.get(i).flipped());
      }
      return firstMovePose(segs, 0);
    }
    return firstPose;
  }

  private static Pose2d firstMovePose(List<TraversalSegment> segs, int from) {
    for (int i = from; i < segs.size(); i++) {
      if (!segs.get(i).isSpin) return segs.get(i).pose;
    }
    return segs.get(0).pose;
  }

  private static Pose2d lastMovePose(List<TraversalSegment> segs) {
    for (int i = segs.size() - 1; i >= 0; i--) {
      if (!segs.get(i).isSpin) return segs.get(i).pose;
    }
    return segs.get(0).pose;
  }

  // ---- Zone detection ----
  private boolean isInAllianceZone(boolean isRed) {
    double x = poseEstimator.getEstimatedPose().getX();
    double normX = isRed ? (FIELD_LEN - x) : x;
    return normX < AZ_FAR_X;
  }

  private boolean isInOpposingZone(boolean isRed) {
    double x = poseEstimator.getEstimatedPose().getX();
    double normX = isRed ? (FIELD_LEN - x) : x;
    return normX > (FIELD_LEN - AZ_FAR_X);
  }

  // ---- Coordinate constants ----
  private static final double FIELD_LEN  = 16.540;
  private static final double AZ_FAR_X  = 3.3;     // alliance zone far edge (neutral boundary)
  private static final double AZ_NEAR_X = 0.540;   // alliance zone near edge (driver station)
  private static final double AZ_BACK_X = 1.530;   // backup X to clear outpost
  private static final double NZ_NEAR_X_BLUE = 5.976;   // neutral zone near edge Blue side
  private static final double NZ_FAR_X_BLUE  = RobotContainer.COMPETITION_MODE ? 11.574 : 9.024;
  private static final double NZ_NEAR_X_RED  = RobotContainer.COMPETITION_MODE ? 10.564 : 9.024;
  private static final double NZ_FAR_X_RED   = 5.976;
  private static final double LEFT_Y    = 7.285;
  private static final double RIGHT_Y   = 0.620;
  private static final double TOWER_Y   = 2.633;   // tower pass Y in alliance zone

  // ---- Segment building ----
  private List<TraversalSegment> buildSegments(
      boolean isRed, boolean inAllianceZone, boolean inOpposingZone) {

    double fX = allianceFarX(isRed, inAllianceZone, inOpposingZone);
    double nX = allianceNearX(isRed, inAllianceZone, inOpposingZone);
    double bX = allianceBackX(isRed, inAllianceZone, inOpposingZone);
    double lY = LEFT_Y;
    double rY = RIGHT_Y;
    double tY = TOWER_Y;

    double toFar   = isRed ? 180.0 : 0.0;
    double toNear  = isRed ? 0.0   : 180.0;
    double toLeft  = 90.0;
    double toRight = 270.0;

    // Opposing zone: mirror headings
    if (inOpposingZone) {
      toFar  = isRed ? 0.0   : 180.0;
      toNear = isRed ? 180.0 : 0.0;
    }

    List<TraversalSegment> segs = new ArrayList<>();

    switch (wall) {
      case FAR -> {
        // Far wall: traverse lY → rY, facing direction of travel in both directions
        segs.add(TraversalSegment.spin(toLeft, toRight));
        segs.add(TraversalSegment.move(p(fX, lY, toLeft), toLeft, toRight));
      }
      case NEAR -> {
        if (inAllianceZone || inOpposingZone) {
          // Near wall with tower dogleg — not reversible, always starts from left side.
          // Headings match the sweep command convention (intake faces the wall being swept).
          segs.add(TraversalSegment.spin(Rotation2d.fromDegrees(toNear)));
          segs.add(TraversalSegment.move(p(nX, lY, toNear), toNear));
          segs.add(TraversalSegment.move(p(bX, lY, toNear), toNear));
          segs.add(TraversalSegment.spin(Rotation2d.fromDegrees(toRight)));
          segs.add(TraversalSegment.move(p(bX, tY, toRight), toRight));
          segs.add(TraversalSegment.spin(Rotation2d.fromDegrees(toNear)));
          segs.add(TraversalSegment.move(p(nX, tY, toNear), toNear));
          segs.add(TraversalSegment.spin(Rotation2d.fromDegrees(toRight)));
          segs.add(TraversalSegment.move(p(nX, rY, toRight), toRight));
        } else {
          // Neutral zone: straight line, facing direction of travel in both directions
          segs.add(TraversalSegment.spin(toRight, toLeft));
          segs.add(TraversalSegment.move(p(nX, rY, toRight), toRight, toLeft));
        }
      }
      case LEFT -> {
        // Left wall: nX → fX along high-Y wall, facing direction of travel in both directions
        segs.add(TraversalSegment.spin(toFar, toNear));
        segs.add(TraversalSegment.move(p(fX, lY, toFar), toFar, toNear));
      }
      case RIGHT -> {
        // Right wall: nX → fX along low-Y wall, facing direction of travel in both directions
        segs.add(TraversalSegment.spin(toFar, toNear));
        segs.add(TraversalSegment.move(p(fX, rY, toFar), toFar, toNear));
      }
    }
    return segs;
  }

  // ---- X coordinate resolution based on current zone ----
  private double allianceFarX(boolean isRed, boolean inAZ, boolean inOZ) {
    if (inAZ) return isRed ? (FIELD_LEN - AZ_FAR_X)  : AZ_FAR_X;
    if (inOZ) return isRed ? AZ_FAR_X : (FIELD_LEN - AZ_FAR_X);
    return isRed ? NZ_FAR_X_RED  : NZ_FAR_X_BLUE;
  }

  private double allianceNearX(boolean isRed, boolean inAZ, boolean inOZ) {
    if (inAZ) return isRed ? (FIELD_LEN - AZ_NEAR_X) : AZ_NEAR_X;
    if (inOZ) return isRed ? AZ_NEAR_X : (FIELD_LEN - AZ_NEAR_X);
    return isRed ? NZ_NEAR_X_RED : NZ_NEAR_X_BLUE;
  }

  private double allianceBackX(boolean isRed, boolean inAZ, boolean inOZ) {
    if (inAZ) return isRed ? (FIELD_LEN - AZ_BACK_X) : AZ_BACK_X;
    if (inOZ) return isRed ? AZ_BACK_X : (FIELD_LEN - AZ_BACK_X);
    return allianceNearX(isRed, inAZ, inOZ); // no outpost in neutral, backupX = nearX
  }

  private static Pose2d p(double x, double y, double deg) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(deg));
  }

  private String fmt(Pose2d pose) {
    return String.format("(%.2f, %.2f, %.0f°)",
        pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  // ---- Segment record ----
  private static class TraversalSegment {
    final boolean isSpin;
    final Pose2d pose;
    final Rotation2d heading;        // heading used in the forward direction
    final Rotation2d reverseHeading; // heading used when this segment is reversed

    private TraversalSegment(boolean isSpin, Pose2d pose, Rotation2d heading, Rotation2d reverseHeading) {
      this.isSpin = isSpin;
      this.pose = pose;
      this.heading = heading;
      this.reverseHeading = reverseHeading;
    }

    // Returns a copy of this segment with heading and reverseHeading swapped,
    // and the move destination replaced with a reversed-direction pose.
    TraversalSegment flipped() {
      Rotation2d rev = reverseHeading != null ? reverseHeading : heading;
      if (isSpin) {
        return new TraversalSegment(true, null, rev, heading);
      }
      // Rebuild pose with the reversed heading
      Pose2d flippedPose = new Pose2d(pose.getTranslation(), rev);
      return new TraversalSegment(false, flippedPose, rev, heading);
    }

    static TraversalSegment move(Pose2d end, double forwardDeg, double reverseDeg) {
      return new TraversalSegment(false,
          new Pose2d(end.getTranslation(), Rotation2d.fromDegrees(forwardDeg)),
          Rotation2d.fromDegrees(forwardDeg),
          Rotation2d.fromDegrees(reverseDeg));
    }

    static TraversalSegment move(Pose2d end, double headingDeg) {
      return move(end, headingDeg, headingDeg);
    }

    static TraversalSegment spin(Rotation2d heading) {
      return new TraversalSegment(true, null, heading, heading);
    }

    static TraversalSegment spin(double headingDeg, double reverseHeadingDeg) {
      return new TraversalSegment(true, null,
          Rotation2d.fromDegrees(headingDeg),
          Rotation2d.fromDegrees(reverseHeadingDeg));
    }
  }

  // ---- Spin-in-place command (same as in sweep commands) ----
  private static class SpinToHeadingCommand extends Command {
    private final DriveSubsystem drive;
    private final Rotation2d target;
    private final ProfiledPIDController pid;

    SpinToHeadingCommand(DriveSubsystem drive, Rotation2d target) {
      this.drive = drive;
      this.target = target;
      this.pid = new ProfiledPIDController(
          5.0, 0.0, 0.1,
          new TrapezoidProfile.Constraints(
              MAX_ANGULAR_SPEED_RAD_PER_SEC,
              MAX_ANGULAR_SPEED_RAD_PER_SEC * 2.0));
      this.pid.setTolerance(Math.toRadians(2.0));
      addRequirements(drive);
    }

    @Override
    public void initialize() {
      double cur = drive.getGyroRotation().getRadians();
      double arc = MathUtil.angleModulus(target.getRadians() - cur);
      pid.reset(cur);
      pid.setGoal(cur + arc);
    }

    @Override
    public void execute() {
      double cur = drive.getGyroRotation().getRadians();
      double omega = MathUtil.clamp(
          pid.calculate(cur), -MAX_ANGULAR_SPEED_RAD_PER_SEC, MAX_ANGULAR_SPEED_RAD_PER_SEC);
      drive.drive(0, 0, omega, true);
    }

    @Override
    public void end(boolean interrupted) { drive.drive(0, 0, 0, true); }

    @Override
    public boolean isFinished() { return pid.atGoal(); }
  }
}
