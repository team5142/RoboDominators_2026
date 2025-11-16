package frc.robot.subsystems;

import static frc.robot.Constants.QuestNav.*;
import static frc.robot.Constants.Swerve.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

/**
 * Manages robot heading using QuestNav IMU with Pigeon2 fallback.
 * Automatically fails over to Pigeon if QuestNav disconnects or provides bad data.
 */
public class GyroSubsystem extends SubsystemBase {
  public enum GyroSource {
    QUESTNAV,
    PIGEON
  }

  private final QuestNav questNav;
  private final Pigeon2 pigeon;

  private GyroSource activeSource = GyroSource.QUESTNAV;
  private GyroSource lastActiveSource = GyroSource.PIGEON;  // Track transitions
  private double lastQuestNavUpdateTime = 0.0;
  private long lastQuestNavFrameCount = -1; 
  private Rotation2d currentRotation = new Rotation2d();
  private Rotation2d lastValidQuestNavRotation = new Rotation2d();

  public GyroSubsystem() {
    questNav = new QuestNav();
    pigeon = new Pigeon2(PIGEON_CAN_ID, CAN_BUS_NAME);
  
    // Configure Pigeon to match QuestNav reference frame if needed
    pigeon.reset();
    
    // Initialize timestamp to prevent immediate timeout on boot
    lastQuestNavUpdateTime = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    updateRotation();
    
    // Detect QuestNav recovery/loss
    if (activeSource != lastActiveSource) {
      onSourceChanged(lastActiveSource, activeSource);
      lastActiveSource = activeSource;
    }
    
    // Logging
    Logger.recordOutput("Gyro/ActiveSource", activeSource.toString());
    Logger.recordOutput("Gyro/Rotation", currentRotation);
    Logger.recordOutput("Gyro/RotationDegrees", currentRotation.getDegrees());
    Logger.recordOutput("Gyro/QuestNavTracking", isQuestNavTracking());
    Logger.recordOutput("Gyro/TimeSinceQuestNavUpdate", 
        Timer.getFPGATimestamp() - lastQuestNavUpdateTime);
  }

  /**
   * Called when gyro source changes (QuestNav recovery/loss)
   */
  private void onSourceChanged(GyroSource from, GyroSource to) {
    String transition = from + " → " + to;
    
    if (to == GyroSource.QUESTNAV) {
      System.out.println("=== QUESTNAV RECOVERED ===");
      System.out.println("Transition: " + transition);
      Logger.recordOutput("Gyro/QuestNavRecovered", true);
    } else if (to == GyroSource.PIGEON) {
      System.err.println("=== QUESTNAV LOST - FALLING BACK TO PIGEON ===");
      System.err.println("Transition: " + transition);
      Logger.recordOutput("Gyro/QuestNavLost", true);
    }
  }

  /**
   * Update rotation from best available source
   */
  private void updateRotation() {
    // Try QuestNav first
    if (tryUpdateFromQuestNav()) {
      activeSource = GyroSource.QUESTNAV;
      return;
    }

    // Fallback to Pigeon
    activeSource = GyroSource.PIGEON;
    currentRotation = getPigeonRotation();
  }

  /**
   * Attempt to get rotation from QuestNav
   * @return true if successful, false if should fallback to Pigeon
   */
  private boolean tryUpdateFromQuestNav() {
    // Check if QuestNav is tracking
    if (!isQuestNavTracking()) {
      return false;
    }

    // Try to get new rotation data (this updates lastQuestNavUpdateTime internally if new data)
    Rotation2d questNavRotation = getQuestNavRotation();
    
    // Check if timeout exceeded (no new data in a while)
    double currentTime = Timer.getFPGATimestamp();
    double timeSinceUpdate = currentTime - lastQuestNavUpdateTime;
    
    if (timeSinceUpdate > MAX_QUESTNAV_DISCONNECT_TIME_SECONDS) {
      Logger.recordOutput("Gyro/QuestNavTimeout", true);
      return false;
    }

    // Sanity check: reject unrealistic angular rates (only if we have previous valid data)
    if (!lastValidQuestNavRotation.equals(new Rotation2d()) && timeSinceUpdate > 0) {
      double angleDelta = Math.abs(
          questNavRotation.minus(lastValidQuestNavRotation).getDegrees());
      double angularRate = angleDelta / timeSinceUpdate;
      
      if (angularRate > MAX_ANGULAR_RATE_DEG_PER_SEC) {
        Logger.recordOutput("Gyro/QuestNavUnrealisticRate", angularRate);
        return false;
      }
    }

    // Valid data - use it
    currentRotation = questNavRotation;
    lastValidQuestNavRotation = questNavRotation;
    return true;
  }

  /**
   * Get rotation from QuestNav (with robot-to-QuestNav transform applied)
   * Updates lastQuestNavUpdateTime when NEW frames are received
   */
  private Rotation2d getQuestNavRotation() {
    try {
      var poseFrames = questNav.getAllUnreadPoseFrames();
      if (poseFrames != null && poseFrames.length > 0) {
        // Get latest frame
        var latestFrame = poseFrames[poseFrames.length - 1];
        
        // Check if this is actually NEW data
        if (latestFrame.frameCount() != lastQuestNavFrameCount) {
          // New data received - update timestamp
          lastQuestNavFrameCount = latestFrame.frameCount();
          lastQuestNavUpdateTime = Timer.getFPGATimestamp();
          
          Logger.recordOutput("Gyro/QuestNavFrameCount", lastQuestNavFrameCount);
        }
        
        // Apply yaw offset from mounting position
        Rotation2d questRotation = latestFrame.questPose().getRotation();
        return questRotation.minus(Rotation2d.fromDegrees(QUEST_YAW_DEG));
      }
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNavError", e.getMessage());
    }
    
    return currentRotation; // Return last known good value if no new frames
  }

  /**
   * Get rotation from Pigeon2
   */
  private Rotation2d getPigeonRotation() {
    return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
  }

  /**
   * Check if QuestNav is actively tracking
   */
  private boolean isQuestNavTracking() {
    try {
      return questNav.isTracking();
    } catch (Exception e) {
      return false;
    }
  }

  /**
   * Get the current best rotation estimate
   */
  public Rotation2d getRotation() {
    return currentRotation;
  }

  /**
   * Get which gyro source is currently active
   */
  public GyroSource getActiveSource() {
    return activeSource;
  }

  /**
   * Reset gyro heading to zero (facing away from drivers)
   * Behavior differs between auto and teleop
   */
  public void resetHeading() {
    // Reset Pigeon immediately
    pigeon.reset();
    
    // Reset QuestNav yaw to 0° while preserving X,Y position
    // QuestNav doesn't have a resetYaw() method, so we use setPose() with new rotation
    try {
      var poseFrames = questNav.getAllUnreadPoseFrames();
      if (poseFrames != null && poseFrames.length > 0) {
        // Get current pose
        Pose2d currentQuestPose = poseFrames[poseFrames.length - 1].questPose();
        
        // Create new pose with same X,Y but 0° rotation
        Pose2d newPose = new Pose2d(
            currentQuestPose.getTranslation(),
            Rotation2d.fromDegrees(0.0));
        
        questNav.setPose(newPose);  // Use setPose, not resetYaw
        Logger.recordOutput("Gyro/QuestNavYawReset", true);
      }
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNavYawResetFailed", e.getMessage());
    }
    
    // Log the reset
    if (DriverStation.isAutonomous()) {
      Logger.recordOutput("Gyro/HeadingReset/Auto", true);
      System.out.println("Gyro reset in AUTONOMOUS mode");
    } else {
      Logger.recordOutput("Gyro/HeadingReset/Teleop", true);
      System.out.println("Gyro reset in TELEOP mode");
    }
    
    Logger.recordOutput("Gyro/HeadingReset", true);
  }

  /**
   * Reset gyro to a specific angle
   * @param angleDegrees Desired heading in degrees
   */
  public void setHeading(double angleDegrees) {
    // Reset Pigeon to specific angle
    pigeon.setYaw(angleDegrees);
    
    // Reset QuestNav to same angle while preserving X,Y
    try {
      var poseFrames = questNav.getAllUnreadPoseFrames();
      if (poseFrames != null && poseFrames.length > 0) {
        // Get current pose
        Pose2d currentQuestPose = poseFrames[poseFrames.length - 1].questPose();
        
        // Create new pose with same X,Y but new rotation
        Pose2d newPose = new Pose2d(
            currentQuestPose.getTranslation(),
            Rotation2d.fromDegrees(angleDegrees));
        
        questNav.setPose(newPose);  // Use setPose, not resetYaw
        Logger.recordOutput("Gyro/QuestNavYawSet", angleDegrees);
      }
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNavYawSetFailed", e.getMessage());
    }
    
    Logger.recordOutput("Gyro/HeadingSet", angleDegrees);
  }

  /**
   * Check if QuestNav is currently being used
   */
  public boolean isUsingQuestNav() {
    return activeSource == GyroSource.QUESTNAV;
  }

  /**
   * Check if Pigeon is currently being used
   */
  public boolean isUsingPigeon() {
    return activeSource == GyroSource.PIGEON;
  }
}