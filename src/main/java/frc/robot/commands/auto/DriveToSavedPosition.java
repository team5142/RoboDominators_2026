package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimatorSubsystem; // NEW: Import PoseEstimatorSubsystem
import org.littletonrobotics.junction.Logger;

/**
 * Drives to a saved field position using PathPlanner's pathfinding
 */
public class DriveToSavedPosition extends Command {
  private final Pose2d targetPose;
  private final String positionName;
  private final PoseEstimatorSubsystem poseEstimator; // NEW: Add this field
  private Command pathCommand;

  // NEW: Settle time tracking
  private final Timer settleTimer = new Timer();
  private static final double SETTLE_TIME_SECONDS = 0.2; // REDUCED from 0.3 - finish faster
  private static final double POSITION_TOLERANCE_METERS = 0.15; // INCREASED from 0.05 (15cm instead of 5cm)
  private static final double ROTATION_TOLERANCE_DEGREES = 10.0; // INCREASED from 3.0 (10° instead of 3°)
  private boolean settling = false;

  public DriveToSavedPosition(Pose2d targetPose, String positionName, PoseEstimatorSubsystem poseEstimator) {
    this.targetPose = targetPose;
    this.positionName = positionName;
    this.poseEstimator = poseEstimator; // NEW: Store reference
  }

  @Override
  public void initialize() {
    System.out.println("Driving to: " + positionName);
    System.out.println("Target: " + targetPose);
    
    // Start timing path generation
    double startTime = Timer.getFPGATimestamp();
    
    // MUCH FASTER constraints - more responsive start
    PathConstraints constraints = new PathConstraints(
        3.0, // max velocity m/s (UP from 2.5 - faster movement)
        2.5, // max acceleration m/s² (UP from 1.5 - MUCH quicker start)
        Units.degreesToRadians(240),  // max angular velocity (UP from 180 - faster rotation)
        Units.degreesToRadians(480)   // max angular acceleration (UP from 360 - quicker heading changes)
    );
    
    // Generate path (this is what takes 3 seconds)
    pathCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0); // end velocity (full stop at target)
    
    double pathGenTime = Timer.getFPGATimestamp() - startTime;
    System.out.println("Path generation took: " + String.format("%.3f", pathGenTime) + "s");
    Logger.recordOutput("DriveToSavedPosition/PathGenTime", pathGenTime);
    
    if (pathCommand != null) {
      pathCommand.initialize();
    } else {
      System.err.println("WARNING: PathPlanner returned null command!");
    }

    settling = false;
    settleTimer.reset();
    settleTimer.stop();
  }

  @Override
  public void execute() {
    if (pathCommand != null) {
      pathCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (pathCommand != null) {
      pathCommand.end(interrupted);
    }
    
    if (interrupted) {
      System.out.println("Drive to " + positionName + " interrupted");
    } else {
      System.out.println("Arrived at " + positionName);
    }
  }

  @Override
  public boolean isFinished() {
    if (pathCommand != null && !pathCommand.isFinished()) {
      return false; // PathPlanner still running
    }
    
    // PathPlanner finished - check if we're truly settled
    Pose2d currentPose = poseEstimator.getEstimatedPose(); // FIXED: Get current pose
    
    double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double rotationError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());
    
    boolean withinTolerance = distanceError < POSITION_TOLERANCE_METERS && 
                              rotationError < ROTATION_TOLERANCE_DEGREES;
    
    if (withinTolerance && !settling) {
      // Start settle timer
      settling = true;
      settleTimer.reset();
      settleTimer.start();
      Logger.recordOutput("DriveToSavedPosition/Settling", true);
    }
    
    if (!withinTolerance) {
      // Drifted out of tolerance - reset timer
      settling = false;
      settleTimer.stop();
    }
    
    // Only finish if we've been settled for SETTLE_TIME_SECONDS
    boolean fullySettled = settling && settleTimer.hasElapsed(SETTLE_TIME_SECONDS);
    
    Logger.recordOutput("DriveToSavedPosition/DistanceError", distanceError);
    Logger.recordOutput("DriveToSavedPosition/RotationError", rotationError);
    Logger.recordOutput("DriveToSavedPosition/Settling", settling);
    Logger.recordOutput("DriveToSavedPosition/FullySettled", fullySettled);
    
    return fullySettled;
  }
}