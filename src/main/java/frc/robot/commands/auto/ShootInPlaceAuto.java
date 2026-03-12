package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

// Stationary shoot auto — no driving.
// Enables tracking, spins up flywheels, deploys intake, then feeds all loaded balls into the hub.
// Tune SPINUP_SECONDS and SHOOT_SECONDS in Constants.Auto.
public class ShootInPlaceAuto extends SequentialCommandGroup {

  public ShootInPlaceAuto(
      TurretSubsystem turret,
      SpindexerSubsystem spindexer,
      SingulatorSubsystem singulator,
      IntakeSubsystem intake,
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem drive) {

    addCommands(
        // Seed pose so aim solver knows where we are on the field
        Commands.runOnce(() -> poseEstimator.manualCompSeed(
            Constants.StartingPositions.SHOOT_IN_PLACE_START,
            drive.getGyroRotation())),

        // Enable tracking, spin up flywheels, deploy intake — all at once
        Commands.runOnce(() -> {
          turret.enableTracking();
          turret.setFlywheelFrontRps(Constants.TurretTargets.HUBCLOSE_FRONT_RPS);
          turret.setFlywheelBackRps(Constants.TurretTargets.HUBCLOSE_BACK_RPS);
          if (intake != null) {
            intake.extend();
            intake.spinIn();
          }
        }, turret),

        // Wait for flywheels to reach speed
        Commands.waitSeconds(Constants.Auto.SHOOT_IN_PLACE_SPINUP_SECONDS),

        // Start the feed chain
        Commands.runOnce(() -> {
          turret.enableFire();
          spindexer.spinForward();
          singulator.primeAndFeed();
        }, turret, spindexer, singulator),

        // Shoot window
        Commands.waitSeconds(Constants.Auto.SHOOT_IN_PLACE_SHOOT_SECONDS),

        // Stop everything
        Commands.runOnce(() -> {
          turret.disableFire();
          turret.setFlywheelPercent(0.0);
          spindexer.stop();
          singulator.pause();
        }, turret, spindexer, singulator)
    );

    setName("ShootInPlace");
  }
}

