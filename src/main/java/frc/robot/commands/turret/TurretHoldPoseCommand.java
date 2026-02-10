package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.turret.TurretAimGoal;
import frc.robot.subsystems.turret.TurretShotProfile;
import frc.robot.subsystems.turret.TurretSubsystem;

// Holds a fixed turret pose while scheduled
public class TurretHoldPoseCommand extends RunCommand {
  private final TurretSubsystem turretSubsystem;
  private final TurretAimGoal goal;

  public TurretHoldPoseCommand(TurretSubsystem turretSubsystem, TurretAimGoal goal) {
    super(() -> {}, turretSubsystem);
    this.turretSubsystem = turretSubsystem;
    this.goal = goal;
    this.goal.enable = true;
  }

  public TurretHoldPoseCommand(TurretSubsystem turretSubsystem, TurretShotProfile profile) {
    this(turretSubsystem, profile.toAimGoal());
  }

  @Override
  public void execute() {
    turretSubsystem.setAimGoal(goal);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopAll();
  }
}
