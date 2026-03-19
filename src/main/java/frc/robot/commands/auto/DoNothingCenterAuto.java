package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

// Safety auto — no movement, no shooting. Seeds pose and lets autonomousInit handle turret homing.
public class DoNothingCenterAuto extends SequentialCommandGroup {

  public DoNothingCenterAuto(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem drive) {

    setName("DoNothingCenter");

    addCommands(
        Commands.runOnce(() -> {
          edu.wpi.first.math.geometry.Pose2d startPose = poseEstimator.getStartPoseForAutoName("DoNothingCenter");
          if (startPose != null) poseEstimator.manualCompSeed(startPose, drive.getGyroRotation());
        })
    );
  }
}
