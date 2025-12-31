package frc.robot.commands.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.SmartLogger;

/**
 * SHOP/PRACTICE calibration tool - Sets robot's starting pose to known field position
 * 
 * Safety features:
 * - If FMS attached: Requires DISABLED (prevent mid-match cheating)
 * - If no FMS (practice): Allows seeding even while enabled (teleop calibration)
 * 
 * Use case: Manually place robot at known position (e.g., AprilTag), press START button
 * 
 * Match behavior: PoseInitializer handles automatic COMP_SEED - this command is rarely used
 */
public class SetStartingPoseCommand extends Command {
  private final Pose2d targetPose;
  private final String positionName;
  private final GyroSubsystem gyro;
  private final QuestNavSubsystem questNav;
  private final DriveSubsystem drive;
  private final PoseEstimatorSubsystem poseEstimator;
  
  private boolean executionBlocked = false;

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
    
    // Prevent conflicts with autonomous/teleop control
    addRequirements(poseEstimator);
  }

  @Override
  public void initialize() {
    // === SAFETY CHECKS (Match vs Practice mode) ===
    
    boolean isFMSAttached = DriverStation.isFMSAttached();
    boolean isDisabled = DriverStation.isDisabled();
    
    // MATCH MODE: FMS attached - must be disabled to prevent mid-match cheating
    if (isFMSAttached && !isDisabled) {
      SmartLogger.logConsoleError("=== MANUAL SEED BLOCKED ===");
      SmartLogger.logConsoleError("FMS attached - must be DISABLED to set pose!");
      SmartLogger.logConsoleError("(Prevents mid-match position cheating)");
      SmartLogger.logConsoleError("==========================");
      
      Logger.recordOutput("ManualReset/BlockedFMSEnabled", true);
      executionBlocked = true;
      return;
    }
    
    // PRACTICE MODE: No FMS - allow seeding anytime (even teleop)
    // This lets you calibrate/test while driving around
    
    // === MANUAL COMP_SEED (Shop/Practice) ===
    
        // === MANUAL COMP_SEED (Shop/Practice) ===
    
        double targetAngleDegrees = targetPose.getRotation().getDegrees();
    
        SmartLogger.logConsole("=== MANUAL POSE RESET ===");
        SmartLogger.logConsole("Position: " + positionName);
        SmartLogger.logConsole("Target: " + formatPose(targetPose));
        SmartLogger.logConsole("Mode: " + (isFMSAttached ? "MATCH (disabled)" : "PRACTICE"));
        
        // Step 1: Reset gyro to target heading (MUST happen first!)
        gyro.setHeading(targetAngleDegrees);
        SmartLogger.logConsole("✓ Pigeon2 reset to " + targetAngleDegrees + "°");
        
        // Step 2: Get confirmed gyro angle (after reset)
        Rotation2d confirmedGyroAngle = drive.getGyroRotation();
        SmartLogger.logConsole("✓ Confirmed gyro: " + String.format("%.1f°", confirmedGyroAngle.getDegrees()));
        
        // Step 3: Centralized COMP_SEED reset (pass confirmed gyro angle)
        poseEstimator.manualCompSeed(targetPose, confirmedGyroAngle);
        
        // SmartDashboard feedback
        SmartDashboard.putString("Starting Position", positionName);
        SmartDashboard.putString("Starting Pose", formatPoseForDisplay(targetPose));
        
        // Verify
        Pose2d actualPose = poseEstimator.getEstimatedPose();
        double actualAngle = drive.getGyroRotation().getDegrees();
        
        SmartDashboard.putString("Actual Pose", formatPoseForDisplay(actualPose));
        
        SmartLogger.logConsole("Verification:");
        SmartLogger.logConsole("  Actual pose: " + formatPose(actualPose));
        SmartLogger.logConsole("  Actual angle: " + String.format("%.1f°", actualAngle));
        SmartLogger.logConsole("========================");
        
        // AdvantageKit logs
        Logger.recordOutput("ManualReset/Name", positionName);
        Logger.recordOutput("ManualReset/TargetPose", targetPose);
        Logger.recordOutput("ManualReset/TargetAngle", targetAngleDegrees);
        Logger.recordOutput("ManualReset/ConfirmedGyroAngle", confirmedGyroAngle.getDegrees());
        Logger.recordOutput("ManualReset/ActualPose", actualPose);
        Logger.recordOutput("ManualReset/ActualAngle", actualAngle);
        Logger.recordOutput("ManualReset/Mode", isFMSAttached ? "MATCH" : "PRACTICE");
        Logger.recordOutput("ManualReset/Success", true);
      }
    
      @Override
      public boolean isFinished() {
        return true; // Instant command
      }
      
      private String formatPose(Pose2d pose) {
        return String.format("(%.2fm, %.2fm, %.1f°)", 
            pose.getX(), 
            pose.getY(), 
            pose.getRotation().getDegrees());
      }
      
      private String formatPoseForDisplay(Pose2d pose) {
        return String.format("X: %.2fm, Y: %.2fm, Θ: %.1f°",
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees());
      }
    }