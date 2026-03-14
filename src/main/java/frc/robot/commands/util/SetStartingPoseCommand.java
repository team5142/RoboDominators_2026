package frc.robot.commands.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.SmartLogger;

// Practice/shop calibration tool - manually seeds robot pose to a known field position.
// If FMS is attached, only works while DISABLED. Without FMS, works anytime (teleop calibration).
// Verifies via pose estimator after seeding; retries up to MAX_SEED_ATTEMPTS if not confirmed.
public class SetStartingPoseCommand extends Command {
  private static final double CONFIRM_TOLERANCE_METERS = 0.15;
  private static final double CONFIRM_TOLERANCE_DEG = 5.0;
  private static final double CONFIRM_TIMEOUT_SEC = 0.6;
  private static final int MAX_SEED_ATTEMPTS = 3;

  private final Pose2d targetPose;
  private final String positionName;
  private final GyroSubsystem gyro;
  private final QuestNavSubsystem questNav;
  private final DriveSubsystem drive;
  private final PoseEstimatorSubsystem poseEstimator;

  private boolean executionBlocked = false;
  private boolean confirmed = false;
  private int seedAttempts = 0;
  private final Timer confirmTimer = new Timer();

  public SetStartingPoseCommand(
      Pose2d targetPose,
      String positionName,
      GyroSubsystem gyro,
      QuestNavSubsystem questNav,
      DriveSubsystem drive,
      PoseEstimatorSubsystem poseEstimator) {
    this.targetPose = targetPose;
    this.positionName = positionName;
    this.gyro = gyro;
    this.questNav = questNav;
    this.drive = drive;
    this.poseEstimator = poseEstimator;
    addRequirements(poseEstimator);
  }

  @Override
  public void initialize() {
    executionBlocked = false;
    confirmed = false;
    seedAttempts = 0;

    boolean isFMSAttached = DriverStation.isFMSAttached();
    boolean isDisabled = DriverStation.isDisabled();

    if (isFMSAttached && !isDisabled) {
      SmartLogger.logConsoleError("=== MANUAL SEED BLOCKED: must be DISABLED during match ===");
      Logger.recordOutput("ManualReset/BlockedFMSEnabled", true);
      executionBlocked = true;
      return;
    }

    SmartLogger.logConsole("=== MANUAL POSE RESET: " + positionName + " ===");
    SmartLogger.logConsole("Target: " + SmartLogger.formatPose(targetPose));
    SmartDashboard.putString("Seed/Status", "SEEDING...");
    SmartLogger.logReplay("ManualReset/SeedRequested", true);

    // Reset gyro to 0 so CTRE field-centric offset stays consistent.
    // The pose estimator stores the heading internally via WPILib's resetPosition offset.
    // Do NOT set gyro to targetPose.getRotation() - that would double the field-centric offset
    // because CTRE uses (pigeon - operatorPerspective) as the effective field angle.
    // setOperatorPerspectiveForward is called at the end of execute() with the alliance downfield direction.
    gyro.setHeading(0.0);
    Rotation2d confirmedGyroAngle = drive.getGyroRotation();
    poseEstimator.manualCompSeed(targetPose, confirmedGyroAngle);
    // Seed QuestNav directly so it stops fighting the new pose estimator position.
    // Without this, incoming QuestNav measurements pull the fused estimate back to the old pose.
    if (questNav.isTracking()) {
      questNav.seedToPose(targetPose);
    }
    seedAttempts = 1;

    confirmTimer.reset();
    confirmTimer.start();

    Logger.recordOutput("ManualReset/Name", positionName);
    Logger.recordOutput("ManualReset/TargetPose", targetPose);
  }

  @Override
  public void execute() {
    if (executionBlocked || confirmed) return;

    // Confirm via pose estimator (which was just seeded) rather than raw QuestNav pose.
    // QuestNav reports in its own tracking space and doesn't know about the field seed.
    Pose2d confirmedPose = poseEstimator.getEstimatedPose();
    if (confirmedPose != null) {
      double posErr = confirmedPose.getTranslation().getDistance(targetPose.getTranslation());
      double rotErr = Math.abs(confirmedPose.getRotation().minus(targetPose.getRotation()).getDegrees());

      SmartDashboard.putNumber("Seed/PosErrorMeters", posErr);
      SmartDashboard.putNumber("Seed/RotErrorDeg", rotErr);

      if (posErr < CONFIRM_TOLERANCE_METERS && rotErr < CONFIRM_TOLERANCE_DEG) {
        confirmed = true;
        SmartDashboard.putString("Seed/Status", "CONFIRMED");
        SmartLogger.logConsole("Seed confirmed after " + seedAttempts + " attempt(s). PosErr=" +
            String.format("%.3fm", posErr) + " RotErr=" + String.format("%.1fdeg", rotErr));
        Logger.recordOutput("ManualReset/SeedConfirmed", true);
        Logger.recordOutput("ManualReset/SeedAttempts", seedAttempts);
        return;
      }
    }

    // Retry if timed out and attempts remain
    if (confirmTimer.hasElapsed(CONFIRM_TIMEOUT_SEC) && seedAttempts < MAX_SEED_ATTEMPTS) {
      seedAttempts++;
      SmartLogger.logConsole("Seed not confirmed - retry " + seedAttempts + "/" + MAX_SEED_ATTEMPTS);
      SmartDashboard.putString("Seed/Status", "RETRY " + seedAttempts);
      poseEstimator.manualCompSeed(targetPose, drive.getGyroRotation());
      if (questNav.isTracking()) {
        questNav.seedToPose(targetPose);
      }
      confirmTimer.reset();
    }
  }

  @Override
  public boolean isFinished() {
    if (executionBlocked) return true;
    if (confirmed) return true;
    // Give up after all retries are exhausted and final timeout elapses
    if (seedAttempts >= MAX_SEED_ATTEMPTS && confirmTimer.hasElapsed(CONFIRM_TIMEOUT_SEC)) {
      SmartDashboard.putString("Seed/Status", "FAILED - CHECK QUEST");
      SmartLogger.logConsoleError("Seed FAILED after " + MAX_SEED_ATTEMPTS + " attempts - Quest may not be tracking");
      Logger.recordOutput("ManualReset/SeedConfirmed", false);
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    confirmTimer.stop();
    if (!executionBlocked && !confirmed) {
      SmartDashboard.putString("Seed/Status", "INTERRUPTED");
    }
    // Set perspective to alliance downfield (0 Blue, 180 Red) - matches what SHOP_RESUME sets.
    // Use alliance direction, not targetPose.getRotation(), since Pigeon was reset to 0.
    if (!executionBlocked) {
      boolean isRed = DriverStation.getAlliance()
          .map(a -> a == DriverStation.Alliance.Red).orElse(false);
      drive.setOperatorPerspectiveForward(Rotation2d.fromDegrees(isRed ? 180.0 : 0.0));
    }
    // Final verification log
    Pose2d actualPose = poseEstimator.getEstimatedPose();
    SmartDashboard.putString("Actual Pose", SmartLogger.formatPose(actualPose));
    Logger.recordOutput("ManualReset/ActualPose", actualPose);
    Logger.recordOutput("ManualReset/Success", confirmed);
  }
}