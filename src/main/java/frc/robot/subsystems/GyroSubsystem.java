package frc.robot.subsystems;

import static frc.robot.Constants.QuestNav.*;
import static frc.robot.Constants.Swerve.*;

import java.util.OptionalInt;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.Logger;

public class GyroSubsystem extends SubsystemBase {
  public enum GyroSource {
    QUESTNAV,
    PIGEON
  }

  private final QuestNav questNav;
  private final Pigeon2 pigeon;

  private GyroSource activeSource = GyroSource.QUESTNAV;
  private GyroSource lastActiveSource = GyroSource.PIGEON;
  private double lastQuestNavUpdateTime = 0.0;
  private int lastQuestNavFrameCount = -1; 
  private Rotation2d currentRotation = new Rotation2d();
  private Rotation2d lastValidQuestNavRotation = new Rotation2d();
  
  // Debug/logging counters
  private int consecutiveQuestNavFailures = 0;
  private int totalFramesProcessed = 0;

  public GyroSubsystem() {
    System.out.println("=== GyroSubsystem Initialization ===");
    
    try {
      questNav = new QuestNav();
      System.out.println("[OK] QuestNav object created successfully");
    } catch (Exception e) {
      System.err.println("[ERROR] Failed to create QuestNav object!");
      e.printStackTrace();
      throw e;
    }
    
    pigeon = new Pigeon2(PIGEON_CAN_ID, CAN_BUS_NAME);
    pigeon.reset();
    lastQuestNavUpdateTime = Timer.getFPGATimestamp();
    
    System.out.println("Pigeon2 initialized on CAN ID: " + PIGEON_CAN_ID);
    System.out.println("Disconnect timeout: " + MAX_QUESTNAV_DISCONNECT_TIME_SECONDS + "s");
    System.out.println("Max angular rate: " + MAX_ANGULAR_RATE_DEG_PER_SEC + " deg/s");
    
    // Test QuestNav connection immediately
    System.out.println("\n--- QuestNav Connection Test ---");
    testQuestNavConnection();
    System.out.println("=====================================\n");
  }
  
  /**
   * Test QuestNav connection at startup to help diagnose issues
   */
  private void testQuestNavConnection() {
    try {
      System.out.println("QuestNav Connection Details:");
      System.out.println("  - Connection: USB > Ethernet Switch > RoboRIO");
      System.out.println("  - Expected Quest IP: 10.51.42.X (team-based subnet)");
      System.out.println("  - NetworkTables server: RoboRIO (10.51.42.2)");
      System.out.println("  - Protocol: NetworkTables 4 (port 1735)");
      System.out.println("");
      
      // Test 1: Can we call isTracking()?
      boolean tracking = questNav.isTracking();
      System.out.println("isTracking() returned: " + tracking);
      
      // Test 1.5: Battery status
      try {
        OptionalInt batteryLevel = questNav.getBatteryPercent();
        if (batteryLevel.isPresent()) {
          int level = batteryLevel.getAsInt();
          System.out.println("Battery Level: " + level + "%");
          
          if (level < 20) {
            System.err.println("[WARNING] Quest battery low (" + level + "%)!");
          } else if (level < 50) {
            System.err.println("[INFO] Quest battery at " + level + "% - consider charging");
          }
        } else {
          System.err.println("[INFO] Battery level not available");
        }
      } catch (Exception e) {
        System.err.println("[WARNING] Could not read battery info: " + e.getMessage());
      }
      
      if (!tracking) {
        System.err.println("[WARNING] QuestNav is NOT tracking!");
        System.err.println("   Possible causes:");
        System.err.println("   1. Quest 3 app not running");
        System.err.println("   2. USB-C to Ethernet adapter not working");
        System.err.println("   3. Ethernet switch not powered or connected");
        System.err.println("   4. Quest not on 10.51.42.X subnet");
        System.err.println("   5. Quest just booted (wait 10-15 seconds for tracking)");
        System.err.println("");
        System.err.println("   Debugging steps:");
        System.err.println("   - Check Quest Settings > Network > USB Ethernet");
        System.err.println("   - Verify Quest IP is 10.51.42.X");
        System.err.println("   - In QuestNav app, check NT server: 10.51.42.2");
        System.err.println("   - Wait 10-15 seconds and check AdvantageScope");
      } else {
        System.out.println("[OK] QuestNav is tracking!");
      }
      
      // Test 2: Can we get pose frames?
      PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
      
      if (frames == null) {
        System.err.println("[ERROR] getAllUnreadPoseFrames() returned null");
        System.err.println("   Quest app running but NOT publishing to NetworkTables");
        System.err.println("   Check NT server setting in QuestNav app");
      } else if (frames.length == 0) {
        System.err.println("[INFO] getAllUnreadPoseFrames() returned empty array");
        System.err.println("   Quest connected to NT but no pose data yet");
        System.err.println("   This is NORMAL if just started - wait 10-15 seconds");
        System.err.println("   Check AdvantageScope in 10 seconds for frame updates");
      } else {
        System.out.println("[OK] Received " + frames.length + " pose frames!");
        System.out.println("   Latest frame count: " + frames[frames.length - 1].frameCount());
        System.out.println("   Quest is actively sending data over Ethernet!");
      }
      
      // NEW: Print ALL public methods in QuestNav API
      System.out.println("\n=== QuestNav Complete API (All Public Methods) ===");
      for (var method : questNav.getClass().getMethods()) {
        // Only show QuestNav methods, not inherited Object methods
        if (method.getDeclaringClass() == questNav.getClass()) {
          String returnType = method.getReturnType().getSimpleName();
          String methodName = method.getName();
          String params = java.util.Arrays.stream(method.getParameterTypes())
              .map(Class::getSimpleName)
              .collect(java.util.stream.Collectors.joining(", "));
          System.out.println("  " + returnType + " " + methodName + "(" + params + ")");
        }
      }
      System.out.println("=================================================\n");
      
    } catch (Exception e) {
      System.err.println("[ERROR] Exception testing QuestNav connection:");
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // CRITICAL: Call QuestNav's periodic update method
    // This processes new frames and updates internal state
    try {
      questNav.commandPeriodic();
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNav/PeriodicError", e.getMessage());
    }
    
    updateRotation();
    
    if (activeSource != lastActiveSource) {
      onSourceChanged(lastActiveSource, activeSource);
      lastActiveSource = activeSource;
    }
    
    // Core gyro outputs
    Logger.recordOutput("Gyro/ActiveSource", activeSource.toString());
    Logger.recordOutput("Gyro/Rotation", currentRotation);
    Logger.recordOutput("Gyro/RotationDegrees", currentRotation.getDegrees());
    Logger.recordOutput("Gyro/RotationRadians", currentRotation.getRadians());
    
    // QuestNav status and health - USE NEW API METHODS
    Logger.recordOutput("Gyro/QuestNavTracking", isQuestNavTracking());
    Logger.recordOutput("Gyro/QuestNavConnected", isQuestNavConnected()); // NEW
    Logger.recordOutput("Gyro/TimeSinceQuestNavUpdate", Timer.getFPGATimestamp() - lastQuestNavUpdateTime);
    
    // Use API methods instead of manual tracking
    try {
      Logger.recordOutput("Gyro/QuestNavFrameCount", questNav.getFrameCount().orElse(-1)); // NEW
      Logger.recordOutput("Gyro/QuestNavTrackingLostCount", questNav.getTrackingLostCounter().orElse(0)); // NEW
      Logger.recordOutput("Gyro/QuestNavLatency", questNav.getLatency()); // NEW
      Logger.recordOutput("Gyro/QuestNavAppTimestamp", questNav.getAppTimestamp().orElse(-1.0)); // NEW
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNav/MetricsError", e.getMessage());
    }
    
    Logger.recordOutput("Gyro/TotalFramesProcessed", totalFramesProcessed);
    Logger.recordOutput("Gyro/ConsecutiveFailures", consecutiveQuestNavFailures);
    Logger.recordOutput("Gyro/IsUsingQuestNav", activeSource == GyroSource.QUESTNAV);
    Logger.recordOutput("Gyro/IsUsingPigeon", activeSource == GyroSource.PIGEON);
    
    // QuestNav battery status
    try {
      OptionalInt batteryLevel = questNav.getBatteryPercent();
      Logger.recordOutput("Gyro/QuestNav/BatteryLevel", batteryLevel.orElse(-1));
      Logger.recordOutput("Gyro/QuestNav/BatteryLow", batteryLevel.orElse(100) < 20);
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNav/BatteryError", e.getMessage());
    }
    
    // Pigeon status (for comparison/backup)
    Logger.recordOutput("Gyro/PigeonRotation", getPigeonRotation());
    Logger.recordOutput("Gyro/PigeonYawDegrees", pigeon.getYaw().getValueAsDouble());
    Logger.recordOutput("Gyro/PigeonYawRadians", Math.toRadians(pigeon.getYaw().getValueAsDouble()));
    
    // Debug outputs (always present in AdvantageKit for troubleshooting)
    Logger.recordOutput("Gyro/Debug/QuestNavIsTracking", isQuestNavTracking());
    Logger.recordOutput("Gyro/Debug/TimeSinceUpdate", Timer.getFPGATimestamp() - lastQuestNavUpdateTime);
    
    // Try to get current QuestNav pose frame for additional logging
    try {
      PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
      if (frames != null && frames.length > 0) {
        PoseFrame latest = frames[frames.length - 1];
        
        // Log the full 3D pose from QuestNav
        Pose3d questPose3d = latest.questPose3d();
        Logger.recordOutput("Gyro/QuestNav/Pose3d", questPose3d);
        Logger.recordOutput("Gyro/QuestNav/Position/X", questPose3d.getX());
        Logger.recordOutput("Gyro/QuestNav/Position/Y", questPose3d.getY());
        Logger.recordOutput("Gyro/QuestNav/Position/Z", questPose3d.getZ());
        Logger.recordOutput("Gyro/QuestNav/Rotation/Roll", questPose3d.getRotation().getX());
        Logger.recordOutput("Gyro/QuestNav/Rotation/Pitch", questPose3d.getRotation().getY());
        Logger.recordOutput("Gyro/QuestNav/Rotation/Yaw", questPose3d.getRotation().getZ());
        
        // Log frame metadata
        Logger.recordOutput("Gyro/QuestNav/FrameTimestamp", latest.dataTimestamp());
        Logger.recordOutput("Gyro/QuestNav/AppTimestamp", latest.appTimestamp());
        Logger.recordOutput("Gyro/QuestNav/FrameCount", latest.frameCount());
        
        // Log how many frames we got this cycle
        Logger.recordOutput("Gyro/QuestNav/UnreadFrameCount", frames.length);
      } else {
        // No frames available
        Logger.recordOutput("Gyro/QuestNav/UnreadFrameCount", 0);
      }
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNav/Error", e.getMessage());
    }
  }

  private void onSourceChanged(GyroSource from, GyroSource to) {
    String transition = from + " -> " + to;
    
    if (to == GyroSource.QUESTNAV) {
      System.out.println("=== QUESTNAV RECOVERED ===");
      System.out.println("Transition: " + transition);
      System.out.println("Frames processed: " + totalFramesProcessed);
      Logger.recordOutput("Gyro/QuestNavRecovered", true);
      consecutiveQuestNavFailures = 0;
    } else if (to == GyroSource.PIGEON) {
      System.err.println("=== QUESTNAV LOST - FALLING BACK TO PIGEON ===");
      System.err.println("Transition: " + transition);
      System.err.println("Consecutive failures: " + consecutiveQuestNavFailures);
      Logger.recordOutput("Gyro/QuestNavLost", true);
    }
  }

  private void updateRotation() {
    // Try to get latest QuestNav rotation
    Rotation2d questNavRotation = getQuestNavRotation();
    
    // Use API method to check connection instead of manual timeout
    boolean questNavConnected = isQuestNavConnected();
    boolean questNavIsUpdating = (Timer.getFPGATimestamp() - lastQuestNavUpdateTime) < 0.5;
    
    if (questNavConnected && questNavIsUpdating) {
      // QuestNav is working - use it
      activeSource = GyroSource.QUESTNAV;
      currentRotation = questNavRotation;
      consecutiveQuestNavFailures = 0;
    } else {
      // QuestNav is frozen/broken - fall back to Pigeon
      activeSource = GyroSource.PIGEON;
      currentRotation = getPigeonRotation();
      consecutiveQuestNavFailures++;
      
      // Log why we failed back
      if (!questNavConnected) {
        Logger.recordOutput("Gyro/Debug/FailoverReason", "Not connected");
      } else if (!questNavIsUpdating) {
        Logger.recordOutput("Gyro/Debug/FailoverReason", "No new frames");
      }
    }
    
    // Log which source we're using
    Logger.recordOutput("Gyro/Debug/QuestNavConnected", questNavConnected);
    Logger.recordOutput("Gyro/Debug/QuestNavIsUpdating", questNavIsUpdating);
    Logger.recordOutput("Gyro/Debug/TimeSinceLastUpdate", Timer.getFPGATimestamp() - lastQuestNavUpdateTime);
  }

  private Rotation2d getQuestNavRotation() {
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      
      // Log frame array status
      Logger.recordOutput("Gyro/Debug/FrameArrayNull", poseFrames == null);
      
      if (poseFrames == null) {
        Logger.recordOutput("Gyro/Debug/FrameArrayLength", 0);
        return currentRotation;
      }
      
      Logger.recordOutput("Gyro/Debug/FrameArrayLength", poseFrames.length);
      
      if (poseFrames.length == 0) {
        return currentRotation;
      }
      
      // Get latest frame
      PoseFrame latestFrame = poseFrames[poseFrames.length - 1];
      
      // Check if it's a new frame
      int newFrameCount = latestFrame.frameCount();
      boolean isNewFrame = (newFrameCount != lastQuestNavFrameCount);
      
      Logger.recordOutput("Gyro/Debug/LatestFrameCount", newFrameCount);
      Logger.recordOutput("Gyro/Debug/IsNewFrame", isNewFrame);
      
      if (isNewFrame) {
        lastQuestNavFrameCount = newFrameCount;
        lastQuestNavUpdateTime = Timer.getFPGATimestamp();
        totalFramesProcessed++;
        
        Logger.recordOutput("Gyro/QuestNavFrameCount", lastQuestNavFrameCount);
        Logger.recordOutput("Gyro/Debug/FrameTimestamp", latestFrame.dataTimestamp());
      }
      
      // Get pose and rotation
      Pose3d questPose = latestFrame.questPose3d();
      Logger.recordOutput("Gyro/Debug/QuestPose3d", questPose);
      
      Rotation2d questRotation = questPose.getRotation().toRotation2d();
      Logger.recordOutput("Gyro/Debug/QuestRotationRaw", questRotation.getDegrees());
      
      Rotation2d correctedRotation = questRotation.minus(Rotation2d.fromDegrees(QUEST_YAW_DEG));
      Logger.recordOutput("Gyro/Debug/QuestRotationCorrected", correctedRotation.getDegrees());
      
      return correctedRotation;
      
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNavError", e.getMessage());
      Logger.recordOutput("Gyro/Debug/ExceptionClass", e.getClass().getSimpleName());
      System.err.println("QuestNav exception: " + e.getMessage());
      e.printStackTrace();
    }
    
    return currentRotation;
  }

  private Rotation2d getPigeonRotation() {
    return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
  }

  private boolean isQuestNavTracking() {
    try {
      boolean tracking = questNav.isTracking();
      Logger.recordOutput("Gyro/Debug/QuestNavTrackingMethod", tracking);
      return tracking;
    } catch (Exception e) {
      Logger.recordOutput("Gyro/Debug/TrackingException", e.getMessage());
      System.err.println("QuestNav isTracking() exception: " + e.getMessage());
      return false;
    }
  }

  /**
   * Check if QuestNav is connected to NetworkTables
   * Uses new API method instead of checking for null frames
   */
  private boolean isQuestNavConnected() {
    try {
      return questNav.isConnected();
    } catch (Exception e) {
      Logger.recordOutput("Gyro/Debug/ConnectionCheckException", e.getMessage());
      return false;
    }
  }

  public Rotation2d getRotation() {
    return currentRotation;
  }

  public GyroSource getActiveSource() {
    return activeSource;
  }

  public void resetHeading() {
    pigeon.reset();
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      if (poseFrames != null && poseFrames.length > 0) {
        Pose3d currentQuestPose = poseFrames[poseFrames.length - 1].questPose3d();
        Pose3d newPose = new Pose3d(
            currentQuestPose.getTranslation(),
            new Rotation3d(0.0, 0.0, 0.0));
        questNav.setPose(newPose);
        Logger.recordOutput("Gyro/QuestNavYawReset", true);
        System.out.println("QuestNav pose reset to: " + newPose);
      }
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNavYawResetFailed", e.getMessage());
      System.err.println("QuestNav reset failed: " + e.getMessage());
    }
    
    if (DriverStation.isAutonomous()) {
      Logger.recordOutput("Gyro/HeadingReset/Auto", true);
      System.out.println("Gyro reset in AUTONOMOUS mode");
    } else {
      Logger.recordOutput("Gyro/HeadingReset/Teleop", true);
      System.out.println("Gyro reset in TELEOP mode");
    }
    
    Logger.recordOutput("Gyro/HeadingReset", true);
  }

  public void setHeading(double angleDegrees) {
    pigeon.setYaw(angleDegrees);
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      if (poseFrames != null && poseFrames.length > 0) {
        Pose3d currentQuestPose = poseFrames[poseFrames.length - 1].questPose3d();
        Pose3d newPose = new Pose3d(
            currentQuestPose.getTranslation(), // Keep X, Y, Z position
            new Rotation3d(0.0, 0.0, Math.toRadians(angleDegrees))); // Update only yaw
        questNav.setPose(newPose);
        Logger.recordOutput("Gyro/QuestNavYawSet", angleDegrees);
        System.out.println("QuestNav heading set to: " + angleDegrees + " deg");
      }
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNavYawSetFailed", e.getMessage());
      System.err.println("QuestNav setHeading failed: " + e.getMessage());
    }
    
    Logger.recordOutput("Gyro/HeadingSet", angleDegrees);
  }
  
  /**
   * Set QuestNav to a specific 3D pose (full position + rotation reset)
   * Use this when resetting robot pose to a known starting position
   */
  public void setQuestNavPose(Pose3d newPose) {
    try {
      questNav.setPose(newPose);
      Logger.recordOutput("Gyro/QuestNavPoseSet", newPose);
      // REMOVED: Console output - only log to AdvantageScope
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNavPoseSetFailed", e.getMessage());
      System.err.println("QuestNav setPose failed: " + e.getMessage());
      e.printStackTrace();
    }
  }

  public boolean isUsingQuestNav() {
    return activeSource == GyroSource.QUESTNAV;
  }

  public boolean isUsingPigeon() {
    return activeSource == GyroSource.PIGEON;
  }

  /**
   * Get QuestNav pose as Pose2d (for pose estimator integration)
   * Returns null if no QuestNav data available
   */
  public Pose2d getQuestNavPose2d() {
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      
      if (poseFrames == null || poseFrames.length == 0) {
        return null; // No QuestNav data available
      }
      
      PoseFrame latestFrame = poseFrames[poseFrames.length - 1];
      Pose3d questPose3d = latestFrame.questPose3d();
      
      // Convert Pose3d to Pose2d (drop Z coordinate)
      Pose2d questPose2d = new Pose2d(
        questPose3d.getX(),
        questPose3d.getY(),
        questPose3d.getRotation().toRotation2d()
      );
      
      return questPose2d;
      
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNavPose2dError", e.getMessage());
      return null;
    }
  }
  
  /**
   * Check if QuestNav has new pose data available
   * Used by PoseEstimatorSubsystem to avoid duplicate vision measurements
   */
  public boolean hasNewQuestNavPose() {
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      return (poseFrames != null && poseFrames.length > 0);
    } catch (Exception e) {
      return false;
    }
  }
}