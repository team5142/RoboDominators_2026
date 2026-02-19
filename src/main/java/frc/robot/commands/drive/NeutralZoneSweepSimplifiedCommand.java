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

public class NeutralZoneSweepSimplifiedCommand extends Command {
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

  public NeutralZoneSweepSimplifiedCommand(
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
        "->SWEEP: Session " + sessionId + " | Entry to " + formatPose(entryPose) 
          + " (dist=" + String.format("%.2f", distanceToEntry) + "m)", 
        "SimpleSweep");
      activeCommand = buildEntryCommand(currentPose, entryPose);
      activeCommand = activeCommand.asProxy();
      justScheduled = true;
      CommandScheduler.getInstance().schedule(activeCommand);
    } else {
      SmartLogger.logConsole(
        "->SWEEP: Session " + sessionId + " | Already at entry, starting sweep", 
        "SimpleSweep");
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
        SmartLogger.logConsole("->SWEEP: Entry complete, starting sweep", "SimpleSweep");
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
    SmartLogger.logConsole("->SWEEP: Session " + sessionId + " ended", "SimpleSweep");
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
        "->SWEEP: Spin to " + String.format("%.0f", segment.targetHeading.getDegrees()) + "°",
        "SimpleSweep");
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
          "->SWEEP: Pre-align to " + String.format("%.0f", intendedHeading.getDegrees()) 
            + "° then move to " + formatPose(segment.endPose),
          "SimpleSweep");
      } else {
        activeCommand = moveCommand;
        SmartLogger.logConsole(
          "->SWEEP: Move to " + formatPose(segment.endPose),
          "SimpleSweep");
      }
    }
    
    activeCommand = activeCommand.asProxy();
    justScheduled = true;
    CommandScheduler.getInstance().schedule(activeCommand);
  }

  private Pose2d findNearestEntryCorner(Pose2d currentPose, boolean isRed) {
    // Entry corners are the near-left and near-right corners of the loop.
    // Each alliance's near wall is their own driver-station side.
    Pose2d leftCorner  = isRed
        ? new Pose2d(nearXRed(),  7.248, Rotation2d.fromDegrees(180.0))
        : new Pose2d(nearXBlue(), 7.248, Rotation2d.fromDegrees(0.0));
    Pose2d rightCorner = isRed
        ? new Pose2d(nearXRed(),  0.624, Rotation2d.fromDegrees(180.0))
        : new Pose2d(nearXBlue(), 0.624, Rotation2d.fromDegrees(0.0));

    double distToLeft  = currentPose.getTranslation().getDistance(leftCorner.getTranslation());
    double distToRight = currentPose.getTranslation().getDistance(rightCorner.getTranslation());

    if (distToLeft <= distToRight) {
      currentSegmentIndex = 0;
      return leftCorner;
    } else {
      currentSegmentIndex = 8;
      return rightCorner;
    }
  }

  private Command buildEntryCommand(Pose2d currentPose, Pose2d entryPose) {
    // Pathfind directly to entry corner - PathPlanner handles the approach heading.
    return AutoBuilder.pathfindToPose(entryPose, pathConstraints);
  }

  // Returns the near-wall X for Blue (their driver-station side, always present).
  private static double nearXBlue() { return 5.976; }

  // Returns the far-wall X for Blue (Red's driver-station side, shortened on practice field).
  private static double farXBlue()  { return RobotContainer.COMPETITION_MODE ? 11.574 : 9.024; }

  // Returns the near-wall X for Red (their driver-station side, always present).
  private static double nearXRed()  { return RobotContainer.COMPETITION_MODE ? 10.564 : 9.024; }

  // Returns the far-wall X for Red (Blue's driver-station side, always present).
  private static double farXRed()   { return 5.976; }

  private List<SweepSegment> buildSegments(boolean isRed) {

    double nearX = isRed ? nearXRed()  : nearXBlue();
    double farX  = isRed ? farXRed()   : farXBlue();
    double leftY   = 7.248;
    double centerY = 3.936;
    double rightY  = 0.624;

    // Intake heading: always faces the far wall while collecting.
    // Blue far wall is East (0 deg), Red far wall is West (180 deg).
    // toLeft/toRight follow the Y axis which is the same for both alliances:
    //   toLeft = 90 deg (North, toward high Y), toRight = 270 deg (South, toward low Y).
    double toFar   = isRed ? 180.0 : 0.0;
    double toNear  = isRed ? 0.0   : 180.0;
    double toLeft  = 90.0;  // North - always toward the left/high-Y wall
    double toRight = 270.0; // South - always toward the right/low-Y wall

    List<SweepSegment> segs = new ArrayList<>();
    // Leg 1: near-left to far-left
    segs.add(SweepSegment.move(p(nearX, leftY,   toFar),  p(farX,  leftY,   toFar)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toRight))); // turn toward center (lower Y)
    // Leg 2: far-left to far-center
    segs.add(SweepSegment.move(p(farX,  leftY,   toRight), p(farX,  centerY, toRight)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toNear)));  // turn to face near wall
    // Leg 3: far-center to near-center
    segs.add(SweepSegment.move(p(farX,  centerY, toNear),  p(nearX, centerY, toNear)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toRight))); // continue toward right wall
    // Leg 4: near-center to near-right
    segs.add(SweepSegment.move(p(nearX, centerY, toRight), p(nearX, rightY,  toRight)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toFar)));   // turn to face far wall
    // Leg 5: near-right to far-right
    segs.add(SweepSegment.move(p(nearX, rightY,  toFar),   p(farX,  rightY,  toFar)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toLeft)));  // turn toward center (higher Y)
    // Leg 6: far-right to far-center
    segs.add(SweepSegment.move(p(farX,  rightY,  toLeft),  p(farX,  centerY, toLeft)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toNear)));  // turn to face near wall
    // Leg 7: far-center to near-center
    segs.add(SweepSegment.move(p(farX,  centerY, toNear),  p(nearX, centerY, toNear)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toLeft)));  // turn toward left wall
    // Leg 8: near-center to near-left
    segs.add(SweepSegment.move(p(nearX, centerY, toLeft),  p(nearX, leftY,   toLeft)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toFar)));   // back to start heading

    return segs;
  }

  // Convenience: build a Pose2d with a heading in degrees.
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
          5.0,
          0.0,
          0.1,
          new TrapezoidProfile.Constraints(
              MAX_ANGULAR_SPEED_RAD_PER_SEC,
              MAX_ANGULAR_SPEED_RAD_PER_SEC * 2.0));
      // No continuous input - we set the goal as an absolute offset in initialize()
      // so the controller always takes the explicit shortest arc and never wraps
      this.headingController.setTolerance(Math.toRadians(2.0));

      // No requirement here - the outer NeutralZoneSweepSimplifiedCommand owns driveSubsystem.
    }

    @Override
    public void initialize() {
      double currentRad = driveSubsystem.getGyroRotation().getRadians();
      // Shortest arc from current to target, always in (-pi, pi]
      double shortestArc = MathUtil.angleModulus(targetHeading.getRadians() - currentRad);
      headingController.reset(currentRad);
      // Goal is current position plus the explicit shortest arc - no ambiguity at 0 or 180
      headingController.setGoal(currentRad + shortestArc);
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
}
