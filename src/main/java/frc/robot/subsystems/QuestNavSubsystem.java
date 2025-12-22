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
import frc.robot.util.SmartLogger;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.Logger;

import java.util.OptionalInt;

// QuestNav SLAM - Meta Quest 3 VR headset for visual-inertial odometry
// Provides pose estimation when vision tags not visible
public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav;
  private final Transform3d robotToQuest;
  
  private boolean isCalibrated = false;
  private int totalFramesProcessed = 0;
  
  public QuestNavSubsystem() {
    SmartLogger.logConsole("QuestNavSubsystem initializing...");
    
    try {
      questNav = new QuestNav();
      SmartLogger.logConsole("QuestNav object created successfully");
    } catch (Exception e) {
      SmartLogger.logConsoleError("Failed to create QuestNav object!");
      e.printStackTrace();
      throw e;
    }
    
    // Store transform for manual application (older API)
    robotToQuest = new Transform3d(
        new Translation3d(QUEST_X_METERS, QUEST_Y_METERS, QUEST_Z_METERS),
        new Rotation3d(0, 0, Math.toRadians(QUEST_YAW_DEG))
    );
    
    SmartLogger.logConsole("Robot-to-Quest transform: " + robotToQuest);
    SmartLogger.logConsole("Using manual transform application (2025-2.1.0-beta API)");
    
    testConnection();
    SmartLogger.logConsole("========================================\n");
  }
  
  @Override
  public void periodic() {
    try {
      questNav.commandPeriodic();
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/PeriodicError", e.getMessage());
    }
    
    boolean connected = isConnected();
    boolean tracking = isTracking();
    
    SmartLogger.logReplay("QuestNav/Connected", connected);
    SmartLogger.logReplay("QuestNav/Tracking", tracking);
    SmartLogger.logReplay("QuestNav/Calibrated", isCalibrated);
    
    try {
      SmartLogger.logReplay("QuestNav/FrameCount", (double) questNav.getFrameCount().orElse(-1));
      SmartLogger.logReplay("QuestNav/TrackingLostCount", (double) questNav.getTrackingLostCounter().orElse(0));
      SmartLogger.logReplay("QuestNav/Latency", questNav.getLatency());
      SmartLogger.logReplay("QuestNav/AppTimestamp", questNav.getAppTimestamp().orElse(-1.0));
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/MetricsError", e.getMessage());
    }
    
    try {
      OptionalInt battery = questNav.getBatteryPercent();
      SmartLogger.logReplay("QuestNav/BatteryLevel", (double) battery.orElse(-1));
      SmartLogger.logReplay("QuestNav/BatteryLow", battery.orElse(100) < 20);
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/BatteryError", e.getMessage());
    }
    
    getRobotPose().ifPresent(pose -> {
      SmartLogger.logReplay("QuestNav/RobotPose", pose);
      SmartLogger.logReplay("QuestNav/X", pose.getX());
      SmartLogger.logReplay("QuestNav/Y", pose.getY());
      SmartLogger.logReplay("QuestNav/Rotation", pose.getRotation().getDegrees());
    });
    
    getRobotPose3d().ifPresent(pose3d -> {
      Rotation3d rotation = pose3d.getRotation();
      double rollDeg = Math.toDegrees(rotation.getX());
      double pitchDeg = Math.toDegrees(rotation.getY());
      double heightMeters = pose3d.getZ();
      
      SmartLogger.logReplay("QuestNav/Pose3d", pose3d);
      SmartLogger.logReplay("QuestNav/Roll", rollDeg);
      SmartLogger.logReplay("QuestNav/Pitch", pitchDeg);
      SmartLogger.logReplay("QuestNav/Height", heightMeters);
      
      double maxTilt = Math.max(Math.abs(rollDeg), Math.abs(pitchDeg));
      SmartLogger.logReplay("QuestNav/MaxTilt", maxTilt);
      
      if (maxTilt > 15.0) {
        SmartLogger.logReplay("QuestNav/TiltWarning", true);
        DriverStation.reportWarning("Robot tilted " + maxTilt + "°", false);
      }
      
      if (heightMeters > 0.2) {
        SmartLogger.logReplay("QuestNav/OnPlatform", true);
      }
    });
    
    SmartLogger.logReplay("QuestNav/TotalFramesProcessed", (double) totalFramesProcessed);
  }
  
  /**
   * Initialize QuestNav to a specific robot pose (optional override).
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
      SmartLogger.logReplay("QuestNav/Initialized", initialPose);
      SmartLogger.logConsole("QuestNav initialized to: " + initialPose);
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/InitError", e.getMessage());
      SmartLogger.logConsoleError("QuestNav initialization failed: " + e.getMessage());
    }
  }
  
  /**
   * Calibrate from vision (optional override).
   */
  public void calibrateFromVision(Pose2d visionPose, double confidence) {
    if (confidence < 0.8) {
      SmartLogger.logReplay("QuestNav/CalibrationRejected", "Low confidence: " + confidence);
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
      
      SmartLogger.logReplay("QuestNav/CalibratedFromVision", visionPose);
      SmartLogger.logConsole("QuestNav calibrated from vision: " + visionPose);
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/CalibrationError", e.getMessage());
    }
  }
  
  /**
   * Get robot pose (manual inverse transform)
   */
  public java.util.Optional<Pose2d> getRobotPose() {
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
      SmartLogger.logReplay("QuestNav/GetPoseError", e.getMessage());
      return java.util.Optional.empty();
    }
  }
  
  /**
   * Get robot pose 3D (manual inverse transform)
   */
  public java.util.Optional<Pose3d> getRobotPose3d() {
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
      SmartLogger.logReplay("QuestNav/GetPose3dError", e.getMessage());
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
      
      // CHANGED: Only count frames received, not processed
      // (PoseEstimator will only use the latest one)
      if (frames != null && frames.length > 0) {
        totalFramesProcessed++; // Only increment by 1 since we only use latest
        
        // Log if we're discarding frames
        if (frames.length > 1) {
          SmartLogger.logReplay("QuestNav/FramesDiscarded", (double) (frames.length - 1));
        }
      }
      
      return frames;
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/GetFramesError", e.getMessage());
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
        SmartLogger.logReplay("QuestNav/HeadingReset", true);
        SmartLogger.logConsole("QuestNav heading reset to 0°");
      } catch (Exception e) {
        SmartLogger.logReplay("QuestNav/HeadingResetError", e.getMessage());
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
        SmartLogger.logReplay("QuestNav/HeadingSet", (double) angleDegrees);
        SmartLogger.logConsole("QuestNav heading set to: " + angleDegrees + "°");
      } catch (Exception e) {
        SmartLogger.logReplay("QuestNav/HeadingSetError", e.getMessage());
      }
    });
  }
  
  private void testConnection() {
    SmartLogger.logConsole("Testing QuestNav connection (USB > Ethernet > RoboRIO)", "QuestNav Test");
    
    boolean tracking = isTracking();
    SmartLogger.logConsole("Tracking: " + tracking);
    
    int battery = getBatteryPercent();
    if (battery >= 0) {
      SmartLogger.logConsole("Battery: " + battery + "%");
      if (battery < 20) {
        SmartLogger.logConsoleError("Quest battery low!");
      }
    }
    
    if (!tracking) {
      SmartLogger.logConsoleError("QuestNav NOT tracking - check USB/Ethernet connection");
    } else {
      SmartLogger.logConsole("QuestNav is tracking!");
    }
    
    PoseFrame[] frames = getAllUnreadFrames();
    if (frames == null) {
      SmartLogger.logConsoleError("getAllUnreadPoseFrames() returned null");
    } else if (frames.length == 0) {
      SmartLogger.logConsole("No pose data yet (normal at startup)");
    } else {
      SmartLogger.logConsole("Received " + frames.length + " pose frames!");
    }
  }
}
