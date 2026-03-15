package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

// Safety auto — no movement, no shooting. Seeds pose to the right ShootInPlace position.
// Enables turret tracking so teleop starts with full controls (no LB+RB confirm needed).
public class DoNothingRightAuto extends SequentialCommandGroup {

  public DoNothingRightAuto(
      TurretSubsystem turret,
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem drive) {

    setName("DoNothingRight");

    addCommands(
        // Seed pose to the right ShootInPlace position — flipped for Red automatically
        Commands.runOnce(() -> {
          edu.wpi.first.math.geometry.Pose2d startPose = poseEstimator.getStartPoseForAutoName("DoNothingRight");
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
