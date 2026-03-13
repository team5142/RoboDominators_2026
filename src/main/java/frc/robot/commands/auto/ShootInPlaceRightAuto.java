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

// Stationary shoot auto — no driving. Right side of field (works for both alliances).
// Pose is seeded from PoseInitializer which flips for Red automatically, same as PP autos.
// Tune SPINUP_SECONDS and SHOOT_SECONDS in Constants.Auto.
public class ShootInPlaceRightAuto extends SequentialCommandGroup {

  public ShootInPlaceRightAuto(
      TurretSubsystem turret,
      SpindexerSubsystem spindexer,
      SingulatorSubsystem singulator,
      IntakeSubsystem intake,
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem drive) {

    setName("ShootInPlaceRight");

    addCommands(
        // Seed pose — PoseInitializer applies the Red flip automatically if needed
        Commands.runOnce(() -> {
          edu.wpi.first.math.geometry.Pose2d startPose = poseEstimator.getStartPoseForAutoName("ShootInPlaceRight");
          if (startPose != null) poseEstimator.manualCompSeed(startPose, drive.getGyroRotation());
        }),

        // Enable tracking, spin up flywheels, deploy intake.
        // Holds the turret requirement so the default command can't zero the flywheels.
        // Phase 1: run for the minimum spinup time regardless of aim state.
        Commands.run(() -> {
          turret.enableTracking();
          turret.setFlywheelFrontRps(Constants.TurretTargets.HUBCLOSE_FRONT_RPS);
          turret.setFlywheelBackRps(Constants.TurretTargets.HUBCLOSE_BACK_RPS);
          if (intake != null) {
            intake.extend();
            intake.spinIn();
          }
        }, turret)
            .withTimeout(Constants.Auto.SHOOT_IN_PLACE_SPINUP_SECONDS),

        // Phase 2: keep holding flywheels at speed while waiting for turret to reach target.
        // Still holds the turret requirement so the default command stays interrupted.
        Commands.run(() -> {
          turret.setFlywheelFrontRps(Constants.TurretTargets.HUBCLOSE_FRONT_RPS);
          turret.setFlywheelBackRps(Constants.TurretTargets.HUBCLOSE_BACK_RPS);
        }, turret)
            .until(turret::isAimed),

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

    setName("ShootInPlaceRight");
  }
}

