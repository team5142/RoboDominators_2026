package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import org.littletonrobotics.junction.Logger;

// PathPlanner-based navigation to saved field positions - uses on-the-fly pathfinding
// Generates path around obstacles to target pose when command starts (takes ~3 seconds)
// Includes settle time to prevent finishing while robot is still oscillating
public class DriveToSavedPosition extends Command {
  private final Pose2d targetPose; // Where to drive (field-relative position)
  private final String positionName; // Human-readable name for logging
  private final PoseEstimatorSubsystem poseEstimator; // Get current position
  private Command pathCommand; // PathPlanner's generated follow command

  private final Timer settleTimer = new Timer(); // Tracks how long we've been at target
  private static final double SETTLE_TIME_SECONDS = 0.2; // Wait 200ms to confirm we're settled
  private static final double POSITION_TOLERANCE_METERS = 0.15; // Accept if within 15cm
  private static final double ROTATION_TOLERANCE_DEGREES = 10.0; // Accept if within 10 degrees
  private boolean settling = false; // Tracks if we're in settle phase

  public DriveToSavedPosition(Pose2d targetPose, String positionName, PoseEstimatorSubsystem poseEstimator) {
    this.targetPose = targetPose;
    this.positionName = positionName;
    this.poseEstimator = poseEstimator;
  }

  @Override
  public void initialize() {
    System.out.println("Driving to: " + positionName);
    System.out.println("Target: " + targetPose);
    
    double startTime = Timer.getFPGATimestamp();
    
    // Define motion constraints - tuned for responsive but safe movement
    PathConstraints constraints = new PathConstraints(
        3.0, // max velocity m/s (fast movement)
        2.5, // max acceleration m/s² (quick start/stop)
        Units.degreesToRadians(240),  // max angular velocity (4 rev/s)
        Units.degreesToRadians(480)); // max angular acceleration (8 rev/s²)
    
    // Generate path from current position to target - this is computationally expensive
    // PathPlanner calculates collision-free path around field obstacles
    pathCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0); // end velocity (stop at target, not drive-through)
    
    double pathGenTime = Timer.getFPGATimestamp() - startTime;
    System.out.println("Path generation took: " + String.format("%.3f", pathGenTime) + "s");
    Logger.recordOutput("DriveToSavedPosition/PathGenTime", pathGenTime);
    
    if (pathCommand != null) {
      pathCommand.initialize(); // Start following the generated path
    } else {
      System.err.println("WARNING: PathPlanner returned null command!");
    }

    settling = false; // Not settled yet
    settleTimer.reset();
    settleTimer.stop();
  }

  @Override
  public void execute() {
    if (pathCommand != null) {
      pathCommand.execute(); // Let PathPlanner control the drivetrain
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (pathCommand != null) {
      pathCommand.end(interrupted); // Stop path following
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
      return false; // PathPlanner still executing path
    }
    
    // PathPlanner finished - now check if we're actually settled at target
    // This prevents finishing while robot is still bouncing/oscillating
    Pose2d currentPose = poseEstimator.getEstimatedPose();
    
    double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double rotationError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());
    
    boolean withinTolerance = distanceError < POSITION_TOLERANCE_METERS && 
                              rotationError < ROTATION_TOLERANCE_DEGREES;
    
    if (withinTolerance && !settling) {
      // Just entered tolerance zone - start settle timer
      settling = true;
      settleTimer.reset();
      settleTimer.start();
      Logger.recordOutput("DriveToSavedPosition/Settling", true);
    }
    
    if (!withinTolerance) {
      // Drifted out of tolerance (wind, defense, etc) - reset timer
      settling = false;
      settleTimer.stop();
    }
    
    // Only finish if we've stayed within tolerance for full settle time
    boolean fullySettled = settling && settleTimer.hasElapsed(SETTLE_TIME_SECONDS);
    
    // Debug logging
    Logger.recordOutput("DriveToSavedPosition/DistanceError", distanceError);
    Logger.recordOutput("DriveToSavedPosition/RotationError", rotationError);
    Logger.recordOutput("DriveToSavedPosition/Settling", settling);
    Logger.recordOutput("DriveToSavedPosition/FullySettled", fullySettled);
    
    return fullySettled;
  }
}