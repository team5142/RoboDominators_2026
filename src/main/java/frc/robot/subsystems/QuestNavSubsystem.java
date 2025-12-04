package frc.robot.subsystems;

import static frc.robot.Constants.QuestNav.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.Logger;

import java.util.OptionalInt;

/**
 * QuestNav SLAM Subsystem - Meta Quest 3 VR headset for visual-inertial odometry
 * 
 * Using QuestNav 2025-2.1.0-beta API (manual transform application)
 */
public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav;
  private final Transform3d robotToQuest; // Robot center to Quest camera transform
  
  private boolean isCalibrated = false;
  private int totalFramesProcessed = 0;
  
  public QuestNavSubsystem() {
    System.out.println("=== QuestNavSubsystem Initialization ===");
    
    try {
      questNav = new QuestNav();
      System.out.println("[OK] QuestNav object created successfully");
    } catch (Exception e) {
      System.err.println("[ERROR] Failed to create QuestNav object!");
      e.printStackTrace();
      throw e;
    }
    
    // Store transform for manual application (older API)
    robotToQuest = new Transform3d(
        new Translation3d(QUEST_X_METERS, QUEST_Y_METERS, QUEST_Z_METERS),
        new Rotation3d(0, 0, Math.toRadians(QUEST_YAW_DEG))
    );
    
    System.out.println("Robot-to-Quest transform: " + robotToQuest);
    System.out.println("Using manual transform application (2025-2.1.0-beta API)");
    
    testConnection();
    System.out.println("========================================\n");
  }
  
  @Override
  public void periodic() {
    try {
      questNav.commandPeriodic();
    } catch (Exception e) {
      Logger.recordOutput("QuestNav/PeriodicError", e.getMessage());
    }
    
    boolean connected = isConnected();
    boolean tracking = isTracking();
    
    Logger.recordOutput("QuestNav/Connected", connected);
    Logger.recordOutput("QuestNav/Tracking", tracking);
    Logger.recordOutput("QuestNav/Calibrated", isCalibrated);
    
    try {
      Logger.recordOutput("QuestNav/FrameCount", questNav.getFrameCount().orElse(-1));
      Logger.recordOutput("QuestNav/TrackingLostCount", questNav.getTrackingLostCounter().orElse(0));
      Logger.recordOutput("QuestNav/Latency", questNav.getLatency());
      Logger.recordOutput("QuestNav/AppTimestamp", questNav.getAppTimestamp().orElse(-1.0));
    } catch (Exception e) {
      Logger.recordOutput("QuestNav/MetricsError", e.getMessage());
    }
    
    try {
      OptionalInt battery = questNav.getBatteryPercent();
      Logger.recordOutput("QuestNav/BatteryLevel", battery.orElse(-1));
      Logger.recordOutput("QuestNav/BatteryLow", battery.orElse(100) < 20);
    } catch (Exception e) {
      Logger.recordOutput("QuestNav/BatteryError", e.getMessage());
    }
    
    getRobotPose().ifPresent(pose -> {
      Logger.recordOutput("QuestNav/RobotPose", pose);
      Logger.recordOutput("QuestNav/X", pose.getX());
      Logger.recordOutput("QuestNav/Y", pose.getY());
      Logger.recordOutput("QuestNav/Rotation", pose.getRotation().getDegrees());
    });
    
    getRobotPose3d().ifPresent(pose3d -> {
      Rotation3d rotation = pose3d.getRotation();
      double rollDeg = Math.toDegrees(rotation.getX());
      double pitchDeg = Math.toDegrees(rotation.getY());
      double heightMeters = pose3d.getZ();
      
      Logger.recordOutput("QuestNav/Pose3d", pose3d);
      Logger.recordOutput("QuestNav/Roll", rollDeg);
      Logger.recordOutput("QuestNav/Pitch", pitchDeg);
      Logger.recordOutput("QuestNav/Height", heightMeters);
      
      double maxTilt = Math.max(Math.abs(rollDeg), Math.abs(pitchDeg));
      Logger.recordOutput("QuestNav/MaxTilt", maxTilt);
      
      if (maxTilt > 15.0) {
        Logger.recordOutput("QuestNav/TiltWarning", true);
        DriverStation.reportWarning("Robot tilted " + maxTilt + "°", false);
      }
      
      if (heightMeters > 0.2) {
        Logger.recordOutput("QuestNav/OnPlatform", true);
      }
    });
    
    Logger.recordOutput("QuestNav/TotalFramesProcessed", totalFramesProcessed);
  }
  
  /**
   * Initialize QuestNav to a specific robot pose (manual transform)
   */
  public void initialize(Pose2d initialPose) {
    try {
      Pose3d initialPose3d = new Pose3d(
          initialPose.getX(),
          initialPose.getY(),
          0.0,
          new Rotation3d(0, 0, initialPose.getRotation().getRadians())
      );
      
      // Transform robot pose to camera pose
      Pose3d cameraPose = initialPose3d.transformBy(robotToQuest);
      
      // Set camera pose (older API)
      questNav.setPose(cameraPose);
      
      isCalibrated = true;
      Logger.recordOutput("QuestNav/Initialized", initialPose);
      System.out.println("QuestNav initialized to: " + initialPose);
    } catch (Exception e) {
      Logger.recordOutput("QuestNav/InitError", e.getMessage());
      System.err.println("QuestNav initialization failed: " + e.getMessage());
    }
  }
  
  /**
   * Calibrate from vision (manual transform)
   */
  public void calibrateFromVision(Pose2d visionPose, double confidence) {
    if (confidence < 0.8) {
      Logger.recordOutput("QuestNav/CalibrationRejected", "Low confidence: " + confidence);
      return;
    }
    
    try {
      Pose3d visionPose3d = new Pose3d(
          visionPose.getX(),
          visionPose.getY(),
          0.0,
          new Rotation3d(0, 0, visionPose.getRotation().getRadians())
      );
      
      Pose3d cameraPose = visionPose3d.transformBy(robotToQuest);
      questNav.setPose(cameraPose);
      
      Logger.recordOutput("QuestNav/CalibratedFromVision", visionPose);
      System.out.println("QuestNav calibrated from vision: " + visionPose);
    } catch (Exception e) {
      Logger.recordOutput("QuestNav/CalibrationError", e.getMessage());
    }
  }
  
  /**
   * Get robot pose (manual inverse transform)
   */
  public java.util.Optional<Pose2d> getRobotPose() {
    if (!isCalibrated) {
      return java.util.Optional.empty();
    }
    
    try {
      PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
      if (frames == null || frames.length == 0) {
        return java.util.Optional.empty();
      }
      
      // Get camera pose
      Pose3d cameraPose3d = frames[frames.length - 1].questPose3d();
      if (cameraPose3d == null) {
        return java.util.Optional.empty();
      }
      
      // Transform camera pose to robot pose
      Pose3d robotPose3d = cameraPose3d.transformBy(robotToQuest.inverse());
      
      Pose2d robotPose2d = new Pose2d(
          robotPose3d.getX(),
          robotPose3d.getY(),
          robotPose3d.getRotation().toRotation2d()
      );
      
      return java.util.Optional.of(robotPose2d);
    } catch (Exception e) {
      Logger.recordOutput("QuestNav/GetPoseError", e.getMessage());
      return java.util.Optional.empty();
    }
  }
  
  /**
   * Get robot pose 3D (manual inverse transform)
   */
  public java.util.Optional<Pose3d> getRobotPose3d() {
    if (!isCalibrated) {
      return java.util.Optional.empty();
    }
    
    try {
      PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
      if (frames == null || frames.length == 0) {
        return java.util.Optional.empty();
      }
      
      Pose3d cameraPose3d = frames[frames.length - 1].questPose3d();
      if (cameraPose3d == null) {
        return java.util.Optional.empty();
      }
      
      Pose3d robotPose3d = cameraPose3d.transformBy(robotToQuest.inverse());
      
      return java.util.Optional.of(robotPose3d);
    } catch (Exception e) {
      Logger.recordOutput("QuestNav/GetPose3dError", e.getMessage());
      return java.util.Optional.empty();
    }
  }
  
  public Rotation2d getRotation() {
    return getRobotPose()
        .map(Pose2d::getRotation)
        .orElse(new Rotation2d());
  }
  
  public PoseFrame[] getAllUnreadFrames() {
    try {
      PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
      
      if (frames != null && frames.length > 0) {
        totalFramesProcessed += frames.length;
      }
      
      return frames;
    } catch (Exception e) {
      Logger.recordOutput("QuestNav/GetFramesError", e.getMessage());
      return null;
    }
  }
  
  public boolean hasNewData() {
    try {
      PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
      return frames != null && frames.length > 0;
    } catch (Exception e) {
      return false;
    }
  }
  
  public boolean isConnected() {
    try {
      OptionalInt battery = questNav.getBatteryPercent();
      if (battery.isPresent()) {
        return true;
      }
      
      java.util.OptionalInt frameCount = questNav.getFrameCount();
      if (frameCount.isPresent() && frameCount.getAsInt() >= 0) {
        return true;
      }
      
      return questNav.isConnected();
    } catch (Exception e) {
      return false;
    }
  }
  
  public boolean isTracking() {
    try {
      return questNav.isTracking();
    } catch (Exception e) {
      return false;
    }
  }
  
  public boolean isCalibrated() {
    return isCalibrated;
  }
  
  public int getBatteryPercent() {
    try {
      return questNav.getBatteryPercent().orElse(-1);
    } catch (Exception e) {
      return -1;
    }
  }
  
  public double getLatency() {
    try {
      return questNav.getLatency();
    } catch (Exception e) {
      return -1.0;
    }
  }
  
  public void resetHeading() {
    getRobotPose3d().ifPresent(currentRobotPose -> {
      Pose3d newRobotPose = new Pose3d(
          currentRobotPose.getTranslation(),
          new Rotation3d(0, 0, 0)
      );
      
      Pose3d newCameraPose = newRobotPose.transformBy(robotToQuest);
      
      try {
        questNav.setPose(newCameraPose);
        Logger.recordOutput("QuestNav/HeadingReset", true);
        System.out.println("QuestNav heading reset to 0°");
      } catch (Exception e) {
        Logger.recordOutput("QuestNav/HeadingResetError", e.getMessage());
      }
    });
  }
  
  public void setHeading(double angleDegrees) {
    getRobotPose3d().ifPresent(currentRobotPose -> {
      Pose3d newRobotPose = new Pose3d(
          currentRobotPose.getTranslation(),
          new Rotation3d(0, 0, Math.toRadians(angleDegrees))
      );
      
      Pose3d newCameraPose = newRobotPose.transformBy(robotToQuest);
      
      try {
        questNav.setPose(newCameraPose);
        Logger.recordOutput("QuestNav/HeadingSet", angleDegrees);
        System.out.println("QuestNav heading set to: " + angleDegrees + "°");
      } catch (Exception e) {
        Logger.recordOutput("QuestNav/HeadingSetError", e.getMessage());
      }
    });
  }
  
  private void testConnection() {
    System.out.println("\n--- QuestNav Connection Test ---");
    System.out.println("Connection: USB > Ethernet Switch > RoboRIO");
    System.out.println("Expected Quest IP: 10.51.42.X");
    System.out.println("NetworkTables: RoboRIO (10.51.42.2:1735)");
    System.out.println("");
    
    boolean tracking = isTracking();
    System.out.println("Tracking: " + tracking);
    
    int battery = getBatteryPercent();
    if (battery >= 0) {
      System.out.println("Battery: " + battery + "%");
      if (battery < 20) {
        System.err.println("[WARNING] Quest battery low!");
      }
    } else {
      System.err.println("[INFO] Battery level not available");
    }
    
    if (!tracking) {
      System.err.println("[WARNING] QuestNav NOT tracking!");
      System.err.println("   Possible causes:");
      System.err.println("   1. Quest 3 app not running");
      System.err.println("   2. USB-C to Ethernet adapter issue");
      System.err.println("   3. Ethernet switch not powered");
      System.err.println("   4. Quest not on 10.51.42.X subnet");
      System.err.println("   5. Wait 10-15s for SLAM initialization");
    } else {
      System.out.println("[OK] QuestNav is tracking!");
    }
    
    PoseFrame[] frames = getAllUnreadFrames();
    if (frames == null) {
      System.err.println("[ERROR] getAllUnreadPoseFrames() returned null");
    } else if (frames.length == 0) {
      System.err.println("[INFO] No pose data yet (normal at startup)");
    } else {
      System.out.println("[OK] Received " + frames.length + " pose frames!");
    }
  }
}
