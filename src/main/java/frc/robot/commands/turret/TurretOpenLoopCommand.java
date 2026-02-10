package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

// Runs turret outputs in open loop while scheduled
public class TurretOpenLoopCommand extends StartEndCommand {
  public TurretOpenLoopCommand(
      TurretSubsystem turretSubsystem,
      double flywheelPercent,
      double hoodPercent,
      double turretPercent,
      double singulatorPercent) {
    super(
        () -> turretSubsystem.setOpenLoopPercents(
            flywheelPercent,
            hoodPercent,
            turretPercent,
            singulatorPercent),
        turretSubsystem::stopAll,
        turretSubsystem);
  }
}
