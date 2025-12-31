package frc.robot.subsystems;

import static frc.robot.Constants.QuestNav.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SmartLogger;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;

import java.util.OptionalInt;

// Meta Quest 3 SLAM - Two modes: COMP_SEED (reset Quest to known pose) vs SHOP_RESUME (use existing tracking)
public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav;
  private final Transform3d robotToQuest;
  
  // Cached measurement (single consumer via QuestNavFusion)
  private Pose2d cachedRobotPose = null;
  private Pose3d cachedRobotPose3d = null;
  private double cachedReceiveTimestampFPGA = -1.0;
  private long cachedFrameSequence = -1; // Single source of truth for sequence numbering
  
  // Consumption tracking (SINGLE CONSUMER: QuestNavFusion only)
  private long lastConsumedSequence = -1;
  
  // Mode tracking
  private boolean isSeeded = false;
  
  // Frame statistics
  private int totalFramesProcessed = 0; // All frames from getAllUnreadPoseFrames() (including rejected)
  private int acceptedFrameCount = 0;   // Successfully cached frames
  private int rejectedFrameCount = 0;   // Frames rejected (null pose, etc)
  private String lastRejectionReason = ""; // Throttle rejection logging
  
  public static class QuestMeasurement {
    public final Pose2d pose;
    public final double receiveTimestampFPGA; // When RoboRIO received the frame
    public final long sequence;
    
    public QuestMeasurement(Pose2d pose, double receiveTimestampFPGA, long sequence) {
      this.pose = pose;
      this.receiveTimestampFPGA = receiveTimestampFPGA;
      this.sequence = sequence;
    }
  }
  
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
    
    robotToQuest = new Transform3d(
        new Translation3d(QUEST_X_METERS, QUEST_Y_METERS, QUEST_Z_METERS),
        new Rotation3d(0, 0, Math.toRadians(QUEST_YAW_DEG))
    );
    
    SmartLogger.logConsole("Robot-to-Quest transform: " + robotToQuest);
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
    
    drainQuestNavFrames();
    
    boolean connected = isConnected();
    boolean tracking = isTracking();
    
    SmartLogger.logReplay("QuestNav/Connected", connected);
    SmartLogger.logReplay("QuestNav/Tracking", tracking);
    SmartLogger.logReplay("QuestNav/Seeded", isSeeded);
    
    try {
      SmartLogger.logReplay("QuestNav/FrameCount", (double) questNav.getFrameCount().orElse(-1));
      SmartLogger.logReplay("QuestNav/TrackingLostCount", (double) questNav.getTrackingLostCounter().orElse(0));
      SmartLogger.logReplay("QuestNav/Latency", questNav.getLatency());
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
    
    SmartLogger.logReplay("QuestNav/CachedPoseAvailable", cachedRobotPose != null);
    SmartLogger.logReplay("QuestNav/CurrentSequence", (double) cachedFrameSequence);
    SmartLogger.logReplay("QuestNav/LastConsumedSequence", (double) lastConsumedSequence);
    SmartLogger.logReplay("QuestNav/HasUnconsumedMeasurement", hasUnconsumedMeasurement());
    
    if (cachedRobotPose != null) {
      SmartLogger.logReplay("QuestNav/RobotPose", cachedRobotPose);
      SmartLogger.logReplay("QuestNav/ReceiveTimestampFPGA", cachedReceiveTimestampFPGA);
      SmartLogger.logReplay("QuestNav/MeasurementAge", getMeasurementAge());
    }
    
    SmartLogger.logReplay("QuestNav/TotalFramesProcessed", (double) totalFramesProcessed);
    SmartLogger.logReplay("QuestNav/AcceptedFrameCount", (double) acceptedFrameCount);
    SmartLogger.logReplay("QuestNav/RejectedFrameCount", (double) rejectedFrameCount);
    SmartLogger.logReplay("QuestNav/LastRejectionReason", lastRejectionReason);
  }
  
  // Drain all unread frames, cache latest with FPGA receive timestamp
  private void drainQuestNavFrames() {
    try {
      PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
      if (frames == null || frames.length == 0) return;
      
      // Count ALL processed frames (including those we'll discard)
      totalFramesProcessed += frames.length;
      
      double fpgaReceiveTime = Timer.getFPGATimestamp();
      PoseFrame latestFrame = frames[frames.length - 1];
      
      if (frames.length > 1) {
        SmartLogger.logReplay("QuestNav/FramesDiscarded", (double) (frames.length - 1));
      }
      
      Pose3d cameraPose3d = latestFrame.questPose3d();
      if (cameraPose3d == null) {
        rejectedFrameCount++;
        String reason = "Null camera pose";
        logRejectionIfChanged(reason);
        return;
      }
      
      // Transform to robot pose
      Pose3d robotPose3d = cameraPose3d.transformBy(robotToQuest.inverse());
      Pose2d robotPose2d = new Pose2d(
          robotPose3d.getX(),
          robotPose3d.getY(),
          robotPose3d.getRotation().toRotation2d());
      
      // Increment sequence (now single source of truth)
      cachedFrameSequence++;
      
      cachedRobotPose = robotPose2d;
      cachedRobotPose3d = robotPose3d;
      cachedReceiveTimestampFPGA = fpgaReceiveTime;
      
      acceptedFrameCount++;
      
      SmartLogger.logReplay("QuestNav/FrameAccepted", true);
      
    } catch (Exception e) {
      rejectedFrameCount++;
      String reason = "Exception: " + e.getMessage();
      logRejectionIfChanged(reason);
      SmartLogger.logReplay("QuestNav/GetFramesError", e.getMessage());
    }
  }
  
  // Throttle rejection logging (only log when reason changes)
  private void logRejectionIfChanged(String reason) {
    if (!reason.equals(lastRejectionReason)) {
      SmartLogger.logReplay("QuestNav/RejectionReason", reason);
      lastRejectionReason = reason;
    }
  }
  
  // Peek: Get measurement without consuming
  public java.util.Optional<QuestMeasurement> peekLatestMeasurement() {
    if (cachedRobotPose == null || cachedFrameSequence <= lastConsumedSequence) {
      return java.util.Optional.empty();
    }
    
    QuestMeasurement measurement = new QuestMeasurement(
        cachedRobotPose,
        cachedReceiveTimestampFPGA,
        cachedFrameSequence
    );
    
    return java.util.Optional.of(measurement);
  }
  
  /**
   * SINGLE CONSUMER CONTRACT: Only QuestNavFusion should call this!
   * 
   * Acknowledges that a measurement has been successfully fused into the pose estimator.
   * This prevents the same frame from being consumed multiple times.
   * 
   * @param sequence The sequence number to acknowledge (from QuestMeasurement)
   * @return true if acknowledged, false if already consumed or invalid
   * 
   * @throws IllegalStateException If called from outside QuestNavFusion
   * (in practice we log instead of throwing to avoid crashes)
   */
  public boolean acknowledgeMeasurement(long sequence) {
    if (sequence != cachedFrameSequence || sequence <= lastConsumedSequence) {
      SmartLogger.logReplay("QuestNav/AcknowledgeFailed", 
          "Attempted to ack seq=" + sequence + " but current=" + cachedFrameSequence + 
          " lastConsumed=" + lastConsumedSequence);
      return false;
    }
    
    lastConsumedSequence = sequence;
    SmartLogger.logReplay("QuestNav/MeasurementAcknowledged", (double) sequence);
    return true;
  }
  
  @Deprecated
  public java.util.Optional<QuestMeasurement> consumeLatestMeasurement() {
    SmartLogger.logConsoleError("consumeLatestMeasurement() is deprecated! Use QuestNavFusion.forceAccept() instead");
    return java.util.Optional.empty();
  }
  
  public boolean hasUnconsumedMeasurement() {
    return cachedRobotPose != null && cachedFrameSequence > lastConsumedSequence;
  }
  
  public java.util.Optional<Pose2d> getRobotPose() {
    return java.util.Optional.ofNullable(cachedRobotPose);
  }
  
  public java.util.Optional<Pose3d> getRobotPose3d() {
    return java.util.Optional.ofNullable(cachedRobotPose3d);
  }
  
  public double getMeasurementAge() {
    if (cachedReceiveTimestampFPGA < 0) return Double.POSITIVE_INFINITY;
    return Timer.getFPGATimestamp() - cachedReceiveTimestampFPGA;
  }
  
  public Rotation2d getRotation() {
    return getRobotPose()
        .map(Pose2d::getRotation)
        .orElse(new Rotation2d());
  }
  
  public boolean isConnected() {
    try {
      OptionalInt battery = questNav.getBatteryPercent();
      if (battery.isPresent()) return true;
      
      java.util.OptionalInt frameCount = questNav.getFrameCount();
      if (frameCount.isPresent() && frameCount.getAsInt() >= 0) return true;
      
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
  
  public boolean isSeeded() {
    return isSeeded;
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
  
  // COMP MODE: Seed Quest to known field pose (resets tracking)
  public void seedToPose(Pose2d fieldPose) {
    try {
      Pose3d fieldPose3d = new Pose3d(
          fieldPose.getX(),
          fieldPose.getY(),
          0.0,
          new Rotation3d(0, 0, fieldPose.getRotation().getRadians())
      );
      
      Pose3d cameraPose = fieldPose3d.transformBy(robotToQuest);
      questNav.setPose(cameraPose);
      
      isSeeded = true;
      
      // Invalidate cache (Quest frame has changed)
      cachedRobotPose = null;
      cachedRobotPose3d = null;
      cachedReceiveTimestampFPGA = -1.0;
      cachedFrameSequence = -1;
      lastConsumedSequence = -1;
      
      SmartLogger.logReplay("QuestNav/SeedToPose", fieldPose);
      SmartLogger.logConsole("QuestNav seeded to: " + formatPose(fieldPose));
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/SeedError", e.getMessage());
      SmartLogger.logConsoleError("QuestNav seed failed: " + e.getMessage());
    }
  }
  
  // SHOP MODE: Keep Quest's existing tracking (no setPose call)
  public void enterResumeMode() {
    isSeeded = false;
    SmartLogger.logConsole("QuestNav in RESUME mode - using existing tracking");
    SmartLogger.logReplay("QuestNav/ResumeMode", true);
  }
  
  private void testConnection() {
    SmartLogger.logConsole("Testing QuestNav connection (USB > Ethernet > RoboRIO)");
    
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
    
    SmartLogger.logConsole("Pose data will be available after first periodic() cycle");
  }
  
  private String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", 
        pose.getX(), 
        pose.getY(), 
        pose.getRotation().getDegrees());
  }
}
