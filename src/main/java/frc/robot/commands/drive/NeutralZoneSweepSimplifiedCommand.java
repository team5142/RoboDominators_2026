package frc.robot.commands.drive;

// TODO (next session with robot):
// Same issues as AllianceZoneSweepSimplifiedCommand - review both files together:
// 1. SpinToHeadingCommand is copied verbatim here - extract to a shared file
//    (commands/drive/SpinToHeadingCommand.java) so both sweep commands use the same one.
// 2. SweepSegment.move() takes a 'start' parameter that is never used - remove it.
// 3. Check for dead conditionals and unused parameters (same pattern as AllianceZone farX() issue).
// 4. formatPose() duplicates SmartLogger.formatPose() - replace with the shared util call.
// 5. Check field coordinates match 2026 AndyMark dimensions (650.12 x 316.64 in).

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
  private static int sessionCounter = 0;
  private int sessionId = 0;

  public NeutralZoneSweepSimplifiedCommand(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem) {
    this.poseEstimator = poseEstimator;
    this.driveSubsystem = driveSubsystem;
    this.pathConstraints = new PathConstraints(2.0, 2.0, Math.toRadians(360.0), Math.toRadians(540.0));
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
    
    CommandScheduler.getInstance().schedule(activeCommand);
  }

  private Pose2d findNearestEntryCorner(Pose2d currentPose, boolean isRed) {
    // Entry corners are the far-left and far-right corners of the loop.
    // Use the same safe boundary constants as buildSegments so they can never diverge.
    double leftY  = 7.175;
    double rightY = 0.876;
    Pose2d leftCorner  = isRed
        ? new Pose2d(farXRed(),   leftY,  Rotation2d.fromDegrees(0.0))
        : new Pose2d(farXBlue(),  leftY,  Rotation2d.fromDegrees(180.0));
    Pose2d rightCorner = isRed
        ? new Pose2d(farXRed(),   rightY, Rotation2d.fromDegrees(0.0))
        : new Pose2d(farXBlue(),  rightY, Rotation2d.fromDegrees(180.0));

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

  // Returns the near-wall X for Blue — must clear Blue hub right face (181.6+23.5=205.1in) + turning radius + margin.
  // 205.1 + 28.5 + 6 = 239.6in = 6.086m
  private static double nearXBlue() { return 6.086; }

  // Returns the far-wall X for Blue — must clear Red hub left face (468.5-23.5=445.0in) - turning radius - margin.
  // 445.0 - 28.5 - 6 = 410.5in = 10.427m (competition). Practice field is shorter so Red hub is not a factor.
  // Practice value pulled in 8in from original 9.024m for intake clearance safety.
  private static double farXBlue()  { return RobotContainer.COMPETITION_MODE ? 10.427 : 8.821; }

  // Returns the near-wall X for Red — must clear Red hub left face (468.5-23.5=445.0in) - turning radius - margin.
  // 445.0 - 28.5 - 6 = 410.5in = 10.427m (competition). Practice uses same shortened X as farXBlue.
  private static double nearXRed()  { return RobotContainer.COMPETITION_MODE ? 10.427 : 8.821; }

  // Returns the far-wall X for Red — must clear Blue hub right face (181.6+23.5=205.1in) + turning radius + margin.
  // 205.1 + 28.5 + 6 = 239.6in = 6.086m (same virtual wall as nearXBlue, mirrored).
  private static double farXRed()   { return 6.086; }

  private List<SweepSegment> buildSegments(boolean isRed) {

    double nearX = isRed ? nearXRed()  : nearXBlue();
    double farX  = isRed ? farXRed()   : farXBlue();
    // Wall clearance: robot width=38in, intake extends 12in, turning radius=28.5in from center.
    // Minimum safe center-to-wall = 28.5in + 6in margin = 34.5in = 0.876m.
    // leftY: 8.051 - 0.876 = 7.175m (34.5in from left wall)
    // rightY: 0.876m (34.5in from right wall)
    double leftY   = 7.175;
    double centerY = 3.936;
    double rightY  = 0.876;

    // Intake heading: faces toward near wall while collecting (pushing fuel back to alliance zone).
    // Blue near wall is West (180 deg), Red near wall is East (0 deg).
    // toLeft/toRight follow the Y axis which is the same for both alliances:
    //   toLeft = 90 deg (North, toward high Y), toRight = 270 deg (South, toward low Y).
    double toFar   = isRed ? 180.0 : 0.0;
    double toNear  = isRed ? 0.0   : 180.0;
    double toLeft  = 90.0;  // North - always toward the left/high-Y wall
    double toRight = 270.0; // South - always toward the right/low-Y wall

    List<SweepSegment> segs = new ArrayList<>();
    // Leg 1: far-left to near-left (robot faces toNear, pushing fuel toward alliance zone)
    segs.add(SweepSegment.move(p(farX,  leftY,   toNear),  p(nearX, leftY,   toNear)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toRight))); // -90 right turn
    // Leg 2: near-left to near-center (robot faces toRight along the near wall)
    segs.add(SweepSegment.move(p(nearX, leftY,   toRight), p(nearX, centerY, toRight)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toFar)));   // spin directly to toFar, skipping intermediate toLeft to avoid double-turn
    // Leg 3: near-center to far-center (robot faces toFar)
    segs.add(SweepSegment.move(p(nearX, centerY, toFar),   p(farX,  centerY, toFar)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toRight))); // -90 right turn
    // Leg 4: far-center to far-right (robot faces toRight along the far wall)
    segs.add(SweepSegment.move(p(farX,  centerY, toRight), p(farX,  rightY,  toRight)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toRight))); // -90 right turn
    // Leg 5: far-right to near-right (robot faces toNear, pushing fuel toward alliance zone)
    segs.add(SweepSegment.move(p(farX,  rightY,  toNear),  p(nearX, rightY,  toNear)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toLeft)));  // +90 left turn
    // Leg 6: near-right to near-center (robot faces toLeft along the near wall)
    segs.add(SweepSegment.move(p(nearX, rightY,  toLeft),  p(nearX, centerY, toLeft)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toFar)));   // spin directly to toFar, skipping intermediate toRight to avoid double-turn
    // Leg 7: near-center to far-center (robot faces toFar)
    segs.add(SweepSegment.move(p(nearX, centerY, toFar),   p(farX,  centerY, toFar)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toLeft)));  // +90 left turn
    // Leg 8: far-center to far-left (robot faces toLeft along the far wall)
    segs.add(SweepSegment.move(p(farX,  centerY, toLeft),  p(farX,  leftY,   toLeft)));
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(toLeft)));  // +90 left turn, back to start

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

      // Requires driveSubsystem to displace DriveWithJoysticks while spinning.
      addRequirements(driveSubsystem);
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
