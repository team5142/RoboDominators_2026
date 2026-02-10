package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretAimGoal;
import frc.robot.subsystems.turret.TurretShotProfile;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.function.Supplier;

// Creates turret commands with shared defaults
public class TurretCommandFactory {
  private final TurretSubsystem turretSubsystem;

  public TurretCommandFactory(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
  }

  public Command openLoop(double flywheelPercent, double hoodPercent, double turretPercent, double singulatorPercent) {
    return new TurretOpenLoopCommand(
        turretSubsystem,
        flywheelPercent,
        hoodPercent,
        turretPercent,
        singulatorPercent);
  }

  public Command aimWithSupplier(Supplier<TurretAimGoal> goalSupplier) {
    return new TurretAimCommand(turretSubsystem, goalSupplier);
  }

  public Command holdAimGoal(TurretAimGoal goal) {
    return new TurretHoldPoseCommand(turretSubsystem, goal);
  }

  public Command holdProfile(TurretShotProfile profile) {
    return new TurretHoldPoseCommand(turretSubsystem, profile);
  }
}
