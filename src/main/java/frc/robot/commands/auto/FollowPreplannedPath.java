package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Skeleton for following a named PathPlanner path.
 * This will be filled in once PathPlanner is integrated.
 */
public class FollowPreplannedPath extends Command {
  private final DriveSubsystem drive;
  private final RobotState robotState;
  private final String pathName;

  public FollowPreplannedPath(DriveSubsystem drive, RobotState robotState, String pathName) {
    this.drive = drive;
    this.robotState = robotState;
    this.pathName = pathName;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    robotState.setAssistMode(RobotState.AssistMode.FOLLOW_PREPLANNED_PATH);
  }

  @Override
  public void execute() {
    // TODO: Integrate PathPlanner follow command here.
  }

  @Override
  public void end(boolean interrupted) {
    robotState.setAssistMode(RobotState.AssistMode.NONE);
  }
}
