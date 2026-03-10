package frc.robot.commands.drive;

// TODO (robot session): SpinToHeadingCommand is copied from the sweep files - extract to a shared util class.

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
  private Command pendingMove = null; // set when a spin must complete before a move starts

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
    segIndex = 0;
    activeCommand = null;
    pendingMove = null;

    boolean isRed = DriverStation.getAlliance()
        .map(a -> a == DriverStation.Alliance.Red).orElse(false);
    boolean inAllianceZone = isInAllianceZone(isRed);
    boolean inOpposingZone = isInOpposingZone(isRed);

    segments = buildSegments(isRed, inAllianceZone, inOpposingZone);

    SmartLogger.logConsole(
        "->WALL[" + wall + "]: Starting with " + segments.size() + " segments", "WallTraversal");
    startNext();
  }

  @Override
  public void execute() {
    if (activeCommand == null) return;
    activeCommand.execute();
    if (activeCommand.isFinished()) {
      activeCommand.end(false);
      // If a move was queued behind a spin, start it now
      if (pendingMove != null) {
        activeCommand = pendingMove;
        pendingMove = null;
        activeCommand.initialize();
        return;
      }
      activeCommand = null;
      segIndex++;
      if (segIndex >= segments.size()) {
        SmartLogger.logConsole("->WALL[" + wall + "]: Traversal complete, holding", "WallTraversal");
      } else {
        startNext();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      activeCommand.end(true);
      activeCommand = null;
    }
    pendingMove = null;
    drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    SmartLogger.logConsole("->WALL[" + wall + "]: ended (interrupted=" + interrupted + ")", "WallTraversal");
  }

  @Override
  public boolean isFinished() {
    return false; // whileTrue cancels this command when the button is released
  }

  // ---- Segment execution ----

  private static final double SPIN_THRESHOLD_DEG = 15.0;

  private void startNext() {
    TraversalSegment seg = segments.get(segIndex);
    Rotation2d targetHeading = seg.pose.getRotation();
    double curDeg = MathUtil.inputModulus(drive.getGyroRotation().getDegrees(), -180, 180);
    double tgtDeg = MathUtil.inputModulus(targetHeading.getDegrees(), -180, 180);
    double headingErr = Math.abs(MathUtil.inputModulus(curDeg - tgtDeg, -180, 180));

    // Pass the target heading as the goal pose rotation so PP only translates — all
    // rotation is handled by SpinToHeadingCommand before the move starts.
    Pose2d movePose = new Pose2d(seg.pose.getTranslation(), Rotation2d.fromDegrees(tgtDeg));
    Command move = AutoBuilder.pathfindToPose(movePose, constraints);
    SmartLogger.logConsole("->WALL[" + wall + "]: Move to " + SmartLogger.formatPose(movePose), "WallTraversal");

    if (headingErr > SPIN_THRESHOLD_DEG) {
      SmartLogger.logConsole(
          "->WALL[" + wall + "]: Pre-spin " + String.format("%.0f", tgtDeg)
              + "° (err=" + String.format("%.0f", headingErr) + "°)", "WallTraversal");
      activeCommand = new SpinToHeadingCommand(drive, Rotation2d.fromDegrees(tgtDeg));
      pendingMove = move;
      activeCommand.initialize();
    } else {
      activeCommand = move;
      activeCommand.initialize();
    }
  }
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
  // All values match the safe boundaries from the three sweep commands.
  // Turning radius (28.5in) + 6in margin = 34.5in = 0.876m from every wall/obstacle.
  // Tower footprint: X 0-44in, Y 123.5-170.5in. backupX clears tower at 84in=2.134m.
  private static final double FIELD_LEN  = FIELD_LENGTH_METERS;
  private static final double AZ_FAR_X  = 3.3;     // alliance zone far edge (neutral boundary)
  private static final double AZ_NEAR_X = 0.876;   // 34.5in from driver station wall
  private static final double AZ_BACK_X = 2.134;   // 84in — clears tower far face + turning radius
  private static final double NZ_NEAR_X_BLUE = 6.086;   // clears Blue hub virtual wall
  private static final double NZ_FAR_X_BLUE  = RobotContainer.COMPETITION_MODE ? 10.427 : 8.821;
  private static final double NZ_NEAR_X_RED  = RobotContainer.COMPETITION_MODE ? 10.427 : 8.821;
  private static final double NZ_FAR_X_RED   = 6.086;   // clears Blue hub virtual wall (mirrored)
  private static final double LEFT_Y    = 7.175;   // 34.5in from left physical wall
  private static final double RIGHT_Y   = 0.876;   // 34.5in from right physical wall
  private static final double TOWER_Y   = 2.134;   // 84in — clears tower right face + turning radius

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
    Pose2d cur = poseEstimator.getEstimatedPose();

    switch (wall) {
      case FAR -> {
        // Far wall: traverse high-Y corner to low-Y corner (or reverse).
        // Determine direction based on which Y-end is closer.
        boolean startLeft = cur.getTranslation().getDistance(p(fX, lY, 0).getTranslation())
            <= cur.getTranslation().getDistance(p(fX, rY, 0).getTranslation());
        if (startLeft) {
          segs.add(TraversalSegment.move(p(fX, lY, toLeft),  toLeft));
          segs.add(TraversalSegment.move(p(fX, rY, toRight), toRight));
        } else {
          segs.add(TraversalSegment.move(p(fX, rY, toRight), toRight));
          segs.add(TraversalSegment.move(p(fX, lY, toLeft),  toLeft));
        }
      }
      case NEAR -> {
        if (inAllianceZone || inOpposingZone) {
          // Near wall with tower dogleg. Direction based on which end is closer.
          double distLeft  = cur.getTranslation().getDistance(p(nX, lY, 0).getTranslation());
          double distRight = cur.getTranslation().getDistance(p(nX, rY, 0).getTranslation());
          boolean startLeft = distLeft <= distRight;
          if (startLeft) {
            segs.add(TraversalSegment.move(p(nX, lY, toNear),  toNear));
            segs.add(TraversalSegment.move(p(bX, lY, toNear),  toNear));
            segs.add(TraversalSegment.move(p(bX, tY, toRight), toRight));
            segs.add(TraversalSegment.move(p(nX, tY, toNear),  toNear));
            segs.add(TraversalSegment.move(p(nX, rY, toRight), toRight));
          } else {
            segs.add(TraversalSegment.move(p(nX, rY, toNear),  toNear));
            segs.add(TraversalSegment.move(p(bX, rY, toNear),  toNear));
            segs.add(TraversalSegment.move(p(bX, tY, toLeft),  toLeft));
            segs.add(TraversalSegment.move(p(nX, tY, toNear),  toNear));
            segs.add(TraversalSegment.move(p(nX, lY, toLeft),  toLeft));
          }
        } else {
          // Neutral zone: straight line along the near wall.
          // Near end (nX) is closer to alliance driver station, far end (fX would alias to nX here).
          // Actually both ends of the NEAR wall are the same X — traverse by Y only.
          boolean startRight = cur.getTranslation().getDistance(p(nX, rY, 0).getTranslation())
              <= cur.getTranslation().getDistance(p(nX, lY, 0).getTranslation());
          if (startRight) {
            segs.add(TraversalSegment.move(p(nX, rY, toRight), toRight));
            segs.add(TraversalSegment.move(p(nX, lY, toLeft),  toLeft));
          } else {
            segs.add(TraversalSegment.move(p(nX, lY, toLeft),  toLeft));
            segs.add(TraversalSegment.move(p(nX, rY, toRight), toRight));
          }
        }
      }
      case LEFT -> {
        // Left wall: traverse near-X corner to far-X corner (or reverse).
        // Near-X end is the alliance-side end; far-X is the neutral-side end.
        boolean startNear = cur.getTranslation().getDistance(p(nX, lY, 0).getTranslation())
            <= cur.getTranslation().getDistance(p(fX, lY, 0).getTranslation());
        if (startNear) {
          segs.add(TraversalSegment.move(p(nX, lY, toFar),  toFar));
          segs.add(TraversalSegment.move(p(fX, lY, toFar),  toFar));
        } else {
          segs.add(TraversalSegment.move(p(fX, lY, toNear), toNear));
          segs.add(TraversalSegment.move(p(nX, lY, toNear), toNear));
        }
      }
      case RIGHT -> {
        // Right wall: traverse near-X corner to far-X corner (or reverse).
        // If closer to far end → face toNear (e.g. 180° for blue), go fX→nX.
        // If closer to near end → face toFar (e.g. 0° for blue), go nX→fX.
        boolean startNear = cur.getTranslation().getDistance(p(nX, rY, 0).getTranslation())
            <= cur.getTranslation().getDistance(p(fX, rY, 0).getTranslation());
        if (startNear) {
          segs.add(TraversalSegment.move(p(nX, rY, toFar),  toFar));
          segs.add(TraversalSegment.move(p(fX, rY, toFar),  toFar));
        } else {
          segs.add(TraversalSegment.move(p(fX, rY, toNear), toNear));
          segs.add(TraversalSegment.move(p(nX, rY, toNear), toNear));
        }
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

  // ---- Segment record ----
  private static class TraversalSegment {
    final Pose2d pose;

    private TraversalSegment(Pose2d pose) {
      this.pose = pose;
    }

    static TraversalSegment move(Pose2d end, double headingDeg) {
      return new TraversalSegment(
          new Pose2d(end.getTranslation(), Rotation2d.fromDegrees(headingDeg)));
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
      // Use raw (unwrapped) gyro as the measurement basis, but compute the shortest arc
      // by normalizing. This keeps the PID measurement continuous through the turn.
      double cur = drive.getGyroRotation().getRadians();
      double curNorm = MathUtil.angleModulus(cur);
      double tgtNorm = MathUtil.angleModulus(target.getRadians());
      double arc = MathUtil.angleModulus(tgtNorm - curNorm);
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
