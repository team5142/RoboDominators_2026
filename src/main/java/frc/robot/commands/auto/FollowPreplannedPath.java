package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveSubsystem;

// Skeleton for following pre-planned PathPlanner paths (.path files)
// This is for autonomous routines where paths are designed ahead of time in PathPlanner GUI
// Different from DriveToSavedPosition which generates paths on-the-fly
// TODO: Implement when creating autonomous routines
public class FollowPreplannedPath extends Command {
  private final DriveSubsystem drive;
  private final RobotState robotState;
  private final String pathName; // Name of .path file (e.g. "BlueLeftSide1Piece")

  public FollowPreplannedPath(DriveSubsystem drive, RobotState robotState, String pathName) {
    this.drive = drive;
    this.robotState = robotState;
    this.pathName = pathName;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    robotState.setAssistMode(RobotState.AssistMode.FOLLOW_PREPLANNED_PATH);
    // TODO: Load path from PathPlanner and start following
    // Example: AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName))
  }

  @Override
  public void execute() {
    // TODO: PathPlanner's FollowPath command will handle execution
    // This method may not be needed once integrated
  }

  @Override
  public void end(boolean interrupted) {
    robotState.setAssistMode(RobotState.AssistMode.NONE);
    // TODO: Stop path following if interrupted
  }
}
