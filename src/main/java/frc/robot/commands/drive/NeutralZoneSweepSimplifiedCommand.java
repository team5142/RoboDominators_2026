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
    
    // DO NOT add requirements - sub-commands handle their own requirements
  }

  @Override
  public void initialize() {
    sessionId = ++sessionCounter;
    segments = buildSegments();
    currentSegmentIndex = 0;
    isInEntryPhase = true;
    
    Pose2d currentPose = poseEstimator.getEstimatedPose();
    Pose2d entryPose = findNearestEntryCorner(currentPose);
    
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

  private Pose2d findNearestEntryCorner(Pose2d currentPose) {
    Pose2d leftCorner = new Pose2d(5.976, 7.248, Rotation2d.fromDegrees(0.0));
    Pose2d rightCorner = new Pose2d(5.976, 0.624, Rotation2d.fromDegrees(0.0));
    
    double distToLeft = currentPose.getTranslation().getDistance(leftCorner.getTranslation());
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
    // Pathfind directly to entry corner with intake-forward heading (0 deg).
    // PathPlanner handles the approach heading - no pre-spin needed.
    return AutoBuilder.pathfindToPose(entryPose, pathConstraints);
  }

  private List<SweepSegment> buildSegments() {
    boolean isCompetition = RobotContainer.COMPETITION_MODE;
    double farX = isCompetition ? 11.574 : 9.024;
    double nearX = 5.976;
    double leftY = 7.248;    // was 7.324, moved 3in away from left wall
    double centerY = 3.936;  // midpoint of new left/right
    double rightY = 0.624;   // was 0.776, moved 6in closer to right wall
    
    List<SweepSegment> segs = new ArrayList<>();
    
    // Segment 0: Move from near-left to far-left (facing East 0°)
    segs.add(SweepSegment.move(
        new Pose2d(nearX, leftY, Rotation2d.fromDegrees(0.0)),
        new Pose2d(farX, leftY, Rotation2d.fromDegrees(0.0))));
    
    // Spin from 0° to 90°
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(90.0)));
    
    // Segment 1: Move from far-left to far-center (facing North 90°)
    segs.add(SweepSegment.move(
        new Pose2d(farX, leftY, Rotation2d.fromDegrees(90.0)),
        new Pose2d(farX, centerY, Rotation2d.fromDegrees(90.0))));
    
    // Spin from 90° to 180°
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(180.0)));
    
    // Segment 2: Move from far-center to near-center (facing West 180°)
    segs.add(SweepSegment.move(
        new Pose2d(farX, centerY, Rotation2d.fromDegrees(180.0)),
        new Pose2d(nearX, centerY, Rotation2d.fromDegrees(180.0))));
    
    // Spin from 180° to 270°
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(270.0)));
    
    // Segment 3: Move from near-center to near-right (facing South 270°)
    segs.add(SweepSegment.move(
        new Pose2d(nearX, centerY, Rotation2d.fromDegrees(270.0)),
        new Pose2d(nearX, rightY, Rotation2d.fromDegrees(270.0))));
    
    // Spin from 270° to 0°
    // Spin from 270° to 0°
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(0.0)));
    
    // Segment 4: Move from near-right to far-right (facing East 0°)
    segs.add(SweepSegment.move(
        new Pose2d(nearX, rightY, Rotation2d.fromDegrees(0.0)),
        new Pose2d(farX, rightY, Rotation2d.fromDegrees(0.0))));
    
    // Spin from 0° to 90°
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(90.0)));
    
    // Segment 5: Move from far-right to far-center (facing North 90°)
    segs.add(SweepSegment.move(
        new Pose2d(farX, rightY, Rotation2d.fromDegrees(90.0)),
        new Pose2d(farX, centerY, Rotation2d.fromDegrees(90.0))));
    
    // Spin from 90° to 180°
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(180.0)));
    
    // Segment 6: Move from far-center to near-center (facing West 180°)
    segs.add(SweepSegment.move(
        new Pose2d(farX, centerY, Rotation2d.fromDegrees(180.0)),
        new Pose2d(nearX, centerY, Rotation2d.fromDegrees(180.0))));
    
    // Spin from 180° to 270°
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(270.0)));
    
    // Segment 7: Move from near-center to near-left (facing South 270°)
    segs.add(SweepSegment.move(
        new Pose2d(nearX, centerY, Rotation2d.fromDegrees(270.0)),
        new Pose2d(nearX, leftY, Rotation2d.fromDegrees(270.0))));
    
    // Spin from 270° to 0° (back to start)
    segs.add(SweepSegment.spin(Rotation2d.fromDegrees(0.0)));
    
    return segs;
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
}
