package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

// Center start auto — robot begins touching the hub so intake must NOT deploy until after backup.
// Turret is locked forward under PID (Phase1Fallback). Bot backs up 1 foot robot-relative,
// then intake deploys, flywheels spin up, and the robot shoots. No rotation needed — hub is
// directly ahead (Blue) or directly behind (Red, facing 180 deg).
public class CenterMoveToShootAuto extends SequentialCommandGroup {

  private static final double BACKUP_SPEED_MPS  = 0.5;
  private static final double BACKUP_DISTANCE_M = 0.457; // 1.5 feet
  private static final double BACKUP_TIME_SECS  = BACKUP_DISTANCE_M / BACKUP_SPEED_MPS;

  public CenterMoveToShootAuto(
      TurretSubsystem turret,
      SpindexerSubsystem spindexer,
      SingulatorSubsystem singulator,
      IntakeSubsystem intake,
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem drive,
      RobotState robotState) {

    setName("CenterMoveToShoot");

    addCommands(
        // Seed pose to center start position — flipped for Red automatically
        Commands.runOnce(() -> {
          edu.wpi.first.math.geometry.Pose2d startPose = poseEstimator.getStartPoseForAutoName("CenterMoveToShoot");
          if (startPose != null) poseEstimator.manualCompSeed(startPose, drive.getGyroRotation());
        }),

        // Lock turret forward under PID and spin up flywheels — intake stays retracted
        // until after backup so it doesn't hit the hub
        Commands.runOnce(() -> {
          robotState.setTurretPhase1Fallback(true);
          robotState.setFlywheelOn(true);
        }),

        // Back up 1 foot robot-relative to clear the hub
        Commands.run(() -> drive.driveRobotRelative(new ChassisSpeeds(-BACKUP_SPEED_MPS, 0.0, 0.0)), drive)
            .withTimeout(BACKUP_TIME_SECS)
            .finallyDo(() -> drive.drive(0.0, 0.0, 0.0, true)),

        // Now safe to deploy intake
        Commands.runOnce(() -> {
          if (intake != null) { intake.extend(); intake.spinIn(); }
        }),

        // Settle + finish flywheel spinup
        Commands.waitSeconds(Constants.Auto.SHOOT_IN_PLACE_SPINUP_SECONDS),

        // Fire
        Commands.runOnce(() -> {
          turret.enableFire();
          spindexer.spinForward();
          singulator.primeAndFeed();
        }, spindexer, singulator),

        Commands.waitSeconds(Constants.Auto.SHOOT_IN_PLACE_SHOOT_SECONDS),

        // Stop everything and clear the fallback flag for teleop
        Commands.runOnce(() -> {
          turret.disableFire();
          turret.setFlywheelPercent(0.0);
          spindexer.stop();
          singulator.pause();
          robotState.setTurretPhase1Fallback(false);
        }, spindexer, singulator)
    );
  }
}
