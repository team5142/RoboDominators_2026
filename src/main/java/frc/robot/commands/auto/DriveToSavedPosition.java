package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drives to a saved field position using PathPlanner's pathfinding
 */
public class DriveToSavedPosition extends Command {
  private final Pose2d targetPose;
  private final String positionName;
  private Command pathCommand;

  public DriveToSavedPosition(Pose2d targetPose, String positionName) {
    this.targetPose = targetPose;
    this.positionName = positionName;
  }

  @Override
  public void initialize() {
    System.out.println("Driving to: " + positionName);
    System.out.println("Target: " + targetPose);
    
    // Create path constraints (40% of max speed)
    PathConstraints constraints = new PathConstraints(
        2.1, // max velocity m/s (~40% of 5.21 m/s)
        2.0, // max acceleration m/s²
        Units.degreesToRadians(360), // max angular velocity
        Units.degreesToRadians(540)  // max angular acceleration
    );
    
    // Build path on-the-fly from current position to target
    pathCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0 // end velocity (stop at target)
    );
    
    if (pathCommand != null) {
      pathCommand.initialize();
    }
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
    return pathCommand != null && pathCommand.isFinished();
  }
}
