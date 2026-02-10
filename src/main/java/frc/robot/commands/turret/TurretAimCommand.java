package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.turret.TurretAimGoal;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.function.Supplier;

// Holds a supplied aim goal while scheduled
public class TurretAimCommand extends RunCommand {
  public TurretAimCommand(TurretSubsystem turretSubsystem, Supplier<TurretAimGoal> goalSupplier) {
    super(() -> {
      TurretAimGoal goal = goalSupplier.get();
      if (goal != null) {
        turretSubsystem.setAimGoal(goal);
      }
    }, turretSubsystem);
  }
}
