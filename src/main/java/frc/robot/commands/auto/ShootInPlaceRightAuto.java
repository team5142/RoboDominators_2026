package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotState;
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
      DriveSubsystem drive,
      RobotState robotState) {

    setName("ShootInPlaceRight");

    addCommands(
        // Seed pose — PoseInitializer applies the Red flip automatically if needed
        Commands.runOnce(() -> {
          edu.wpi.first.math.geometry.Pose2d startPose = poseEstimator.getStartPoseForAutoName("ShootInPlaceRight");
          if (startPose != null) poseEstimator.manualCompSeed(startPose, drive.getGyroRotation());
        }),

        // Enable tracking and spin up flywheels. No turret requirement here — default aim
        // pipeline keeps running so the turret actually rotates to the computed bearing.
        Commands.runOnce(() -> {
          //turret.enableTracking();
          robotState.setFlywheelOn(true);
          if (intake != null) {
            intake.extend();
            intake.spinIn();
          }
        }),

        // Wait for spinup + settle — default pipeline aims turret during this time.
        Commands.waitSeconds(Constants.Auto.SHOOT_IN_PLACE_SPINUP_SECONDS + 1.0),

        // Start the feed chain
        Commands.runOnce(() -> {
          turret.enableFire();
          spindexer.spinForward();
          singulator.primeAndFeed();
        }, spindexer, singulator),

        // Shoot window
        Commands.waitSeconds(Constants.Auto.SHOOT_IN_PLACE_SHOOT_SECONDS),

        // Stop everything
        Commands.runOnce(() -> {
          turret.disableFire();
          turret.setFlywheelPercent(0.0);
          spindexer.stop();
          singulator.pause();
        }, spindexer, singulator)
    );

    setName("ShootInPlaceRight");
  }
}

