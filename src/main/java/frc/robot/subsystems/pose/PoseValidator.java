package frc.robot.subsystems.pose;

import static frc.robot.Constants.Auto.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.SmartLogger;

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
  private PoseInitializer poseInitializer;
  
  public void setAutoChooser(SendableChooser<Command> autoChooser) {
    this.autoChooser = autoChooser;
  }

  public void setPoseInitializer(PoseInitializer poseInitializer) {
    this.poseInitializer = poseInitializer;
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
      SmartDashboard.putString("Pose/Validation", "No auto selected");
      SmartDashboard.putBoolean("Pose/AutoAligned", false);
      return;
    }
    
    double posError = currentPose.getTranslation().getDistance(expectedPose.getTranslation());
    double rotError = Math.abs(currentPose.getRotation().minus(expectedPose.getRotation()).getDegrees());
    
    boolean withinTolerance = 
        posError < STARTING_POSE_TOLERANCE_METERS && 
        rotError < STARTING_POSE_TOLERANCE_DEGREES;
    
    SmartDashboard.putString("Pose/Validation", withinTolerance ? "ALIGNED" : "NOT ALIGNED");
    SmartDashboard.putBoolean("Pose/AutoAligned", withinTolerance);
    SmartDashboard.putNumber("Pose/PosError", posError);
    SmartDashboard.putNumber("Pose/RotError", rotError);
    SmartDashboard.putString("Pose/Current", SmartLogger.formatPose(currentPose));
    SmartDashboard.putString("Pose/Expected", SmartLogger.formatPose(expectedPose));
    
    Logger.recordOutput("PoseValidation/WithinTolerance", withinTolerance);
    Logger.recordOutput("PoseValidation/PosError", posError);
    Logger.recordOutput("PoseValidation/RotError", rotError);
    Logger.recordOutput("PoseValidation/CurrentPose", currentPose);
    Logger.recordOutput("PoseValidation/ExpectedPose", expectedPose);
  }
  
  private Pose2d getExpectedAutoStartPose() {
    if (autoChooser == null || poseInitializer == null) return null;

    try {
      Command selectedAuto = autoChooser.getSelected();
      if (selectedAuto == null) return null;

      String autoName = selectedAuto.getName();
      Pose2d pose = poseInitializer.getStartPoseForAutoName(autoName);

      if (pose == null) {
        Logger.recordOutput("PoseValidation/UnknownAuto", autoName);
      }

      return pose;
    } catch (Exception e) {
      Logger.recordOutput("PoseValidation/GetPoseError", e.getMessage());
      return null;
    }
  }
}
