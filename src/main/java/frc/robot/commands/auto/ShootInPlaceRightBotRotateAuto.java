package frc.robot.commands.auto;

import static frc.robot.Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SEC;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

// Emergency fallback auto when turret rotation is broken (Phase 4 failed).
// Turret is locked forward under PID. Bot backs up 1 foot to clear the hub,
// then rotates 49 deg CCW to face the hub, then shoots.
// Only run this on Red right side. On Blue use ShootInPlaceRight instead.
public class ShootInPlaceRightBotRotateAuto extends SequentialCommandGroup {

  private static final double ROTATE_LEFT_DEG       = 49.0; // recalculated after 1 foot backup
  private static final double HEADING_TOLERANCE_DEG = 3.0;
  private static final double ROTATE_TIMEOUT_SECS   = 3.0;
  private static final double BACKUP_SPEED_MPS      = 0.5;  // robot-relative, -X = backward
  private static final double BACKUP_DISTANCE_M     = 0.305; // 1 foot
  private static final double BACKUP_TIME_SECS      = BACKUP_DISTANCE_M / BACKUP_SPEED_MPS;

  public ShootInPlaceRightBotRotateAuto(
      TurretSubsystem turret,
      SpindexerSubsystem spindexer,
      SingulatorSubsystem singulator,
      IntakeSubsystem intake,
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem drive,
      RobotState robotState) {

    setName("ShootInPlaceRightBotRotate");

    ProfiledPIDController headingController = new ProfiledPIDController(
        5.0, 0.0, 0.1,
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RAD_PER_SEC,
            MAX_ANGULAR_SPEED_RAD_PER_SEC * 2.0));
    headingController.setTolerance(Math.toRadians(HEADING_TOLERANCE_DEG));

    addCommands(
        // Seed pose — same start position as ShootInPlaceRight, flipped for Red
        Commands.runOnce(() -> {
          Pose2d startPose = poseEstimator.getStartPoseForAutoName("ShootInPlaceRightBotRotate");
          if (startPose != null) poseEstimator.manualCompSeed(startPose, drive.getGyroRotation());
        }),

        // Lock turret forward under PID, spin up flywheels, extend intake
        Commands.runOnce(() -> {
          robotState.setTurretPhase1Fallback(true);
          robotState.setFlywheelOn(true);
          if (intake != null) { intake.extend(); intake.spinIn(); }
        }),

        // Back up 1 foot robot-relative to clear the hub before rotating
        Commands.run(() -> drive.driveRobotRelative(new ChassisSpeeds(-BACKUP_SPEED_MPS, 0.0, 0.0)), drive)
            .withTimeout(BACKUP_TIME_SECS)
            .finallyDo(() -> drive.drive(0.0, 0.0, 0.0, true)),

        // Rotate bot 49 deg CCW (left) from current gyro heading to face hub
        Commands.runOnce(() -> {
          double currentRad = drive.getGyroRotation().getRadians();
          double goalRad    = currentRad + Math.toRadians(ROTATE_LEFT_DEG);
          headingController.reset(currentRad);
          headingController.setGoal(goalRad);
        }),
        Commands.run(() -> {
          double omegaRadPerSec = headingController.calculate(drive.getGyroRotation().getRadians());
          omegaRadPerSec = MathUtil.clamp(omegaRadPerSec,
              -MAX_ANGULAR_SPEED_RAD_PER_SEC, MAX_ANGULAR_SPEED_RAD_PER_SEC);
          drive.drive(0.0, 0.0, omegaRadPerSec, true);
        }, drive)
            .until(headingController::atGoal)
            .withTimeout(ROTATE_TIMEOUT_SECS)
            .finallyDo(() -> drive.drive(0.0, 0.0, 0.0, true)),

        // Settle after rotate + finish flywheel spinup
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
