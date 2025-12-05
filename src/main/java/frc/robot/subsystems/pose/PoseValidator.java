package frc.robot.subsystems.pose;

import static frc.robot.Constants.Auto.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Validates robot pose against expected auto starting position
 * 
 * Responsibilities:
 * - Periodically check if robot is aligned for auto
 * - Provide driver feedback (SmartDashboard + console)
 * - Log validation results for debugging
 */
public class PoseValidator {
  
  private double lastValidationTime = 0.0;
  private static final double VALIDATION_INTERVAL_SECONDS = 10.0;
  
  private SendableChooser<Command> autoChooser;
  
  public void setAutoChooser(SendableChooser<Command> autoChooser) {
    this.autoChooser = autoChooser;
  }
  
  /**
   * Periodically validate pose against expected auto start
   * Called every cycle from PoseEstimatorSubsystem.periodic()
   */
  public void periodicValidation(Pose2d currentPose) {
    double currentTime = Timer.getFPGATimestamp();
    
    if (currentTime - lastValidationTime < VALIDATION_INTERVAL_SECONDS) {
      return;
    }
    lastValidationTime = currentTime;
    
    Pose2d expectedPose = getExpectedAutoStartPose();
    
    if (expectedPose == null) {
      /* System.out.println("========== POSE VALIDATION (No Auto) ==========");
      System.out.println("Current pose: " + formatPose(currentPose));
      System.out.println("Auto: NONE SELECTED");
      System.out.println("Ready for manual placement");
      System.out.println("=============================================="); */
      
      SmartDashboard.putString("Pose/Validation", "No auto selected");
      SmartDashboard.putBoolean("Pose/AutoAligned", false);
      return;
    }
    
    double posError = currentPose.getTranslation().getDistance(expectedPose.getTranslation());
    double rotError = Math.abs(currentPose.getRotation().minus(expectedPose.getRotation()).getDegrees());
    
    boolean withinTolerance = 
        posError < STARTING_POSE_TOLERANCE_METERS && 
        rotError < STARTING_POSE_TOLERANCE_DEGREES;
    
    System.out.println("========== POSE VALIDATION ==========");
    System.out.println("Current:  " + formatPose(currentPose));
    System.out.println("Expected: " + formatPose(expectedPose));
    System.out.println("Position error: " + String.format("%.2fm", posError) + 
                       " (tolerance: " + STARTING_POSE_TOLERANCE_METERS + "m)");
    System.out.println("Rotation error: " + String.format("%.1f°", rotError) + 
                       " (tolerance: " + STARTING_POSE_TOLERANCE_DEGREES + "°)");
    
    if (withinTolerance) {
      System.out.println("Status: ALIGNED - Ready for auto");
    } else {
      System.err.println("Status: NOT ALIGNED - Adjust robot position!");
      if (posError >= STARTING_POSE_TOLERANCE_METERS) {
        System.err.println("  → Move robot " + String.format("%.2fm", posError) + " closer");
      }
      if (rotError >= STARTING_POSE_TOLERANCE_DEGREES) {
        System.err.println("  → Rotate robot " + String.format("%.1f°", rotError) + " more");
      }
    }
    System.out.println("=====================================");
    
    SmartDashboard.putString("Pose/Validation", withinTolerance ? "ALIGNED" : "NOT ALIGNED");
    SmartDashboard.putBoolean("Pose/AutoAligned", withinTolerance);
    SmartDashboard.putNumber("Pose/PosError", posError);
    SmartDashboard.putNumber("Pose/RotError", rotError);
    SmartDashboard.putString("Pose/Current", formatPose(currentPose));
    SmartDashboard.putString("Pose/Expected", formatPose(expectedPose));
    
    Logger.recordOutput("PoseValidation/WithinTolerance", withinTolerance);
    Logger.recordOutput("PoseValidation/PosError", posError);
    Logger.recordOutput("PoseValidation/RotError", rotError);
    Logger.recordOutput("PoseValidation/CurrentPose", currentPose);
    Logger.recordOutput("PoseValidation/ExpectedPose", expectedPose);
  }
  
  private Pose2d getExpectedAutoStartPose() {
    if (autoChooser == null) return null;
    
    try {
      Command selectedAuto = autoChooser.getSelected();
      if (selectedAuto == null) return null;
      
      String autoName = selectedAuto.getName();
      
      switch (autoName.toLowerCase()) {
        case "leftside1piece":
        case "leftside3piece":
          return new Pose2d(7.20, 0.45, Rotation2d.fromDegrees(180.0));
        case "rightside1piece":
          return new Pose2d(7.20, 5.50, Rotation2d.fromDegrees(180.0));
        default:
          Logger.recordOutput("PoseValidation/UnknownAuto", autoName);
          return null;
      }
    } catch (Exception e) {
      Logger.recordOutput("PoseValidation/GetPoseError", e.getMessage());
      return null;
    }
  }
  
  private String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", 
        pose.getX(), 
        pose.getY(), 
        pose.getRotation().getDegrees());
  }
}
