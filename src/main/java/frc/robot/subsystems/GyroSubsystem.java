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

// Hybrid gyro system - manages Meta Quest 3 VR headset (QuestNav) with Pigeon2 gyroscope failover
// QuestNav provides SLAM-based 6DOF tracking (very accurate) but can lose tracking if view obstructed
// Pigeon2 is traditional IMU gyro - always works but drifts over time (no absolute position)
// System automatically switches to Pigeon if QuestNav disconnects or stops updating
public class GyroSubsystem extends SubsystemBase {
  // Gyro source tracking - which sensor are we currently using
  public enum GyroSource {
    QUESTNAV, // Quest 3 VR headset (preferred - SLAM tracking)
    PIGEON    // Pigeon2 IMU (backup - traditional gyro)
  }

  private final QuestNav questNav; // Meta Quest 3 VR headset object
  private final Pigeon2 pigeon; // CTRE Pigeon2 IMU on CANivore

  private GyroSource activeSource = GyroSource.QUESTNAV; // Start with QuestNav (switch to Pigeon if it fails)
  private GyroSource lastActiveSource = GyroSource.PIGEON; // Track transitions for logging
  private double lastQuestNavUpdateTime = 0.0; // Timestamp of last QuestNav frame (detect timeouts)
  private int lastQuestNavFrameCount = -1; // Track frame count to detect new data
  private Rotation2d currentRotation = new Rotation2d(); // Current heading estimate
  private Rotation2d lastValidQuestNavRotation = new Rotation2d(); // Fallback if QuestNav glitches
  
  private int consecutiveQuestNavFailures = 0; // Count failures for diagnostics
  private int totalFramesProcessed = 0; // Total QuestNav updates received

  public GyroSubsystem() {
    System.out.println("=== GyroSubsystem Initialization ===");
    
    // Initialize QuestNav (Meta Quest 3 VR headset via USB-C to Ethernet adapter)
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
    
    testQuestNavConnection(); // Run diagnostics at startup
    System.out.println("=====================================\n");
  }
  
  // Test QuestNav connection at startup - helps diagnose NetworkTables issues
  private void testQuestNavConnection() {
    try {
      System.out.println("\n--- QuestNav Connection Test ---");
      System.out.println("QuestNav Connection Details:");
      System.out.println("  - Connection: USB > Ethernet Switch > RoboRIO");
      System.out.println("  - Expected Quest IP: 10.51.42.X (team-based subnet)");
      System.out.println("  - NetworkTables server: RoboRIO (10.51.42.2)");
      System.out.println("  - Protocol: NetworkTables 4 (port 1735)");
      System.out.println("");
      
      // Test tracking status
      boolean tracking = questNav.isTracking();
      System.out.println("isTracking() returned: " + tracking);
      
      // Test battery status
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
      } else {
        System.out.println("[OK] QuestNav is tracking!");
      }
      
      // Test pose frame availability
      PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
      
      if (frames == null) {
        System.err.println("[ERROR] getAllUnreadPoseFrames() returned null");
        System.err.println("   Quest app running but NOT publishing to NetworkTables");
      } else if (frames.length == 0) {
        System.err.println("[INFO] getAllUnreadPoseFrames() returned empty array");
        System.err.println("   Quest connected to NT but no pose data yet");
        System.err.println("   This is NORMAL if just started - wait 10-15 seconds");
      } else {
        System.out.println("[OK] Received " + frames.length + " pose frames!");
        System.out.println("   Latest frame count: " + frames[frames.length - 1].frameCount());
        System.out.println("   Quest is actively sending data over Ethernet!");
      }
      
      // Print complete QuestNav API for debugging
      System.out.println("\n=== QuestNav Complete API (All Public Methods) ===");
      for (var method : questNav.getClass().getMethods()) {
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
    // CRITICAL: Call QuestNav's update method - processes new frames from Quest
    try {
      questNav.commandPeriodic();
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNav/PeriodicError", e.getMessage());
    }
    
    updateRotation(); // Decide which sensor to use and get rotation
    
    // Log source transitions (QuestNav recovered, or QuestNav lost)
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

  // Log source transitions for diagnostics
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

  // Decide which sensor to use and get current rotation
  private void updateRotation() {
    Rotation2d questNavRotation = getQuestNavRotation();
    
    // Check QuestNav health using API method
    boolean questNavConnected = isQuestNavConnected();
    boolean questNavIsUpdating = (Timer.getFPGATimestamp() - lastQuestNavUpdateTime) < 0.5;
    
    if (questNavConnected && questNavIsUpdating) {
      activeSource = GyroSource.QUESTNAV; // QuestNav is working - use it
      currentRotation = questNavRotation;
      consecutiveQuestNavFailures = 0;
    } else {
      activeSource = GyroSource.PIGEON; // QuestNav failed - fall back to Pigeon
      currentRotation = getPigeonRotation();
      consecutiveQuestNavFailures++;
      
      // Log why we failed over
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

  // Get latest QuestNav rotation - returns last known rotation if no new data
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
        return currentRotation; // No new frames - use last known rotation
      }
      
      PoseFrame latestFrame = poseFrames[poseFrames.length - 1];
      
      // Check if frame is new (compare frame count to last processed)
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
      
      Pose3d questPose = latestFrame.questPose3d();
      Logger.recordOutput("Gyro/Debug/QuestPose3d", questPose);
      
      Rotation2d questRotation = questPose.getRotation().toRotation2d();
      Logger.recordOutput("Gyro/Debug/QuestRotationRaw", questRotation.getDegrees());
      
      // Apply yaw offset from mounting angle (Quest might not be facing exactly forward)
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

  // Check if QuestNav is connected to NetworkTables (uses battery as proof of connection)
  private boolean isQuestNavConnected() {
    try {
      // PRIMARY CHECK: Battery data proves NetworkTables is working
      OptionalInt battery = questNav.getBatteryPercent();
      if (battery.isPresent()) {
        // Battery data available = Quest is on NetworkTables
        return true;
      }
      
      // SECONDARY CHECK: Frame count proves Quest is sending data
      OptionalInt frameCount = questNav.getFrameCount();
      if (frameCount.isPresent() && frameCount.getAsInt() >= 0) {
        return true;
      }
      
      // FALLBACK: Check built-in method (stricter - requires tracking)
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

  // Reset heading to zero (called by "orient to field" button)
  public void resetHeading() {
    pigeon.reset();
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      if (poseFrames != null && poseFrames.length > 0) {
        Pose3d currentQuestPose = poseFrames[poseFrames.length - 1].questPose3d();
        Pose3d newPose = new Pose3d(
            currentQuestPose.getTranslation(), // Keep XYZ position
            new Rotation3d(0.0, 0.0, 0.0)); // Reset yaw to zero
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

  // Set heading to a specific angle
  public void setHeading(double angleDegrees) {
    pigeon.setYaw(angleDegrees);
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      if (poseFrames != null && poseFrames.length > 0) {
        Pose3d currentQuestPose = poseFrames[poseFrames.length - 1].questPose3d();
        Pose3d newPose = new Pose3d(
            currentQuestPose.getTranslation(), // Keep XYZ position
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
  
  // Set QuestNav to a specific 3D pose (used by PoseEstimatorSubsystem when resetting robot position)
  public void setQuestNavPose(Pose3d newPose) {
    try {
      questNav.setPose(newPose);
      Logger.recordOutput("Gyro/QuestNavPoseSet", newPose);
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

  // Get QuestNav pose as Pose2d (for pose estimator integration) - returns null if no data
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
  
  // Check if QuestNav has new pose data (used by PoseEstimatorSubsystem to avoid duplicate measurements)
  public boolean hasNewQuestNavPose() {
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      return (poseFrames != null && poseFrames.length > 0);
    } catch (Exception e) {
      return false;
    }
  }

  // NEW: Get ALL unread QuestNav frames for multi-frame processing
  public PoseFrame[] getAllQuestNavFrames() {
    try {
      return questNav.getAllUnreadPoseFrames();
    } catch (Exception e) {
      Logger.recordOutput("Gyro/QuestNav/GetFramesError", e.getMessage());
      return null;
    }
  }
}