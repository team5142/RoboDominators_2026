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

// Stationary shoot auto — no driving. Robot starts up against the hub facing left (90 deg).
// Uses BLUE_REBUILT_HUB_RIGHT_ACCURATE pose for more precise QuestNav seeding than ShootInPlaceRight.
public class ShootInPlaceRightAccurateAuto extends SequentialCommandGroup {

  public ShootInPlaceRightAccurateAuto(
      TurretSubsystem turret,
      SpindexerSubsystem spindexer,
      SingulatorSubsystem singulator,
      IntakeSubsystem intake,
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem drive,
      RobotState robotState) {

    setName("ShootInPlaceRightAccurate");

    addCommands(
        // Seed pose — PoseInitializer applies the Red flip automatically if needed
        Commands.runOnce(() -> {
          edu.wpi.first.math.geometry.Pose2d startPose = poseEstimator.getStartPoseForAutoName("ShootInPlaceRightAccurate");
          if (startPose != null) poseEstimator.manualCompSeed(startPose, drive.getGyroRotation());
        }),

        Commands.runOnce(() -> {
          robotState.setFlywheelOn(true);
          if (intake != null) {
            intake.extendOnly();
          }
        }),

        // Wait for flywheels to reach 95% of target speed before feeding.
        // 4s timeout backstop so auto never hangs if flywheel fails to spin up.
        Commands.waitUntil(() -> turret.isFlywheelSpinningFast())
            .withTimeout(4.0),

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
          robotState.setFlywheelOn(false);
          spindexer.stop();
          singulator.pause();
        }, spindexer, singulator)
    );
  }
}
