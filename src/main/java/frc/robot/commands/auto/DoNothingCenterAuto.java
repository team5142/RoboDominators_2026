package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

// Safety auto — no movement, no shooting, no intake deployment.
// Seeds pose to center field at the same X as ShootInPlace autos so Quest has a known position.
// Enables turret tracking so teleop starts with full controls immediately (no LB+RB confirm needed).
public class DoNothingCenterAuto extends SequentialCommandGroup {

  public DoNothingCenterAuto(
      TurretSubsystem turret,
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem drive) {

    setName("DoNothingCenter");

    addCommands(
        // Seed pose to center field — flipped for Red automatically
        Commands.runOnce(() -> {
          edu.wpi.first.math.geometry.Pose2d startPose = poseEstimator.getStartPoseForAutoName("DoNothingCenter");
          if (startPose != null) poseEstimator.manualCompSeed(startPose, drive.getGyroRotation());
        }),

        // Enable tracking so onTeleopInit() sees it and skips the LB+RB confirm
        Commands.runOnce(() -> {
          if (turret != null) turret.homeForward();
          if (turret != null) turret.enableTracking();
        }, turret)
    );
  }
}
