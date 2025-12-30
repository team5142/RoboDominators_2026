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

/**
 * QuestNav SLAM - Meta Quest 3 VR headset for visual-inertial odometry
 * 
 * FRAME CONSUMPTION MODEL:
 * - drainQuestNavFrames() runs in periodic() (SINGLE CONSUMER)
 * - Latest frame is cached with monotonic sequence number
 * - consumeLatestMeasurement() returns cached frame ONCE per sequence
 * - Prevents re-injection of same measurement into pose estimator
 * 
 * TIMESTAMP SEMANTICS (RECEIVE-ALIGNED):
 * - QuestNav does NOT provide reliable per-frame capture timestamps
 * - measurementTimeFPGA = FPGA receive time (NOT capture time)
 * - This is conservative: treats latency as zero rather than guessing
 * - SwerveDrivePoseEstimator applies measurement at receive time
 * - Age calculations reflect "time since receipt," not "time since capture"
 */
public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav;
  private final Transform3d robotToQuest;
  
  // CACHED VALUES (single consumer pattern)
  private Pose2d cachedRobotPose = null;
  private Pose3d cachedRobotPose3d = null;
  private double cachedQuestTimestamp = -1.0; // Raw Quest timestamp (for debugging only)
  private double cachedFPGAReceiveTime = -1.0; // When we got the frame (TRUE measurement time)
  private long cachedFrameSequence = -1; // Monotonic sequence to prevent re-consumption
  
  // Consumption tracking
  private long lastConsumedSequence = -1; // Last sequence number consumed by fusion
  private long currentSequence = 0; // Monotonically increasing frame counter
  
  // Timestamp alignment (Quest → FPGA) - NOT USED for measurement timestamps
  // Kept for debugging/monitoring only
  private double timestampOffset = 0.0; // (FPGA time - Quest time) offset
  private boolean timestampAligned = false;
  private static final double TIMESTAMP_FILTER_BETA = 0.05; // Low-pass filter
  private static final double MAX_MEASUREMENT_AGE_SECONDS = 2.0; // Reject stale data
  private static final double MIN_MEASUREMENT_AGE_SECONDS = -0.05; // Allow slight clock drift
  
  private boolean isCalibrated = false;
  private int totalFramesProcessed = 0;
  private int rejectedFrameCount = 0;
  
  /**
   * Represents a consumable QuestNav measurement with receive-aligned timestamp
   */
  public static class QuestMeasurement {
    public final Pose2d pose;
    public final double timestampFPGA; // FPGA receive time (NOT capture time)
    public final long sequence; // Frame sequence number
    
    public QuestMeasurement(Pose2d pose, double timestampFPGA, long sequence) {
      this.pose = pose;
      this.timestampFPGA = timestampFPGA;
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
    
    // Store transform for manual application (older API)
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
    // Service QuestNav communications every loop
    try {
      questNav.commandPeriodic();
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/PeriodicError", e.getMessage());
    }
    
    // ONLY PLACE that drains frames from QuestNav
    drainQuestNavFrames();
    
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
    
    SmartLogger.logReplay("QuestNav/CachedPoseAvailable", cachedRobotPose != null);
    SmartLogger.logReplay("QuestNav/TimestampAligned", timestampAligned);
    SmartLogger.logReplay("QuestNav/TimestampOffset", timestampOffset);
    SmartLogger.logReplay("QuestNav/CurrentSequence", (double) currentSequence);
    SmartLogger.logReplay("QuestNav/LastConsumedSequence", (double) lastConsumedSequence);
    SmartLogger.logReplay("QuestNav/UnconsumedMeasurementAvailable", hasUnconsumedMeasurement());
    
    if (cachedRobotPose != null) {
      SmartLogger.logReplay("QuestNav/RobotPose", cachedRobotPose);
      SmartLogger.logReplay("QuestNav/MeasurementTimeFPGA", cachedFPGAReceiveTime);
      SmartLogger.logReplay("QuestNav/MeasurementAge", getMeasurementAge());
    }
    
    SmartLogger.logReplay("QuestNav/TotalFramesProcessed", (double) totalFramesProcessed);
    SmartLogger.logReplay("QuestNav/RejectedFrameCount", (double) rejectedFrameCount);
  }
  
  /**
   * SINGLE CONSUMER: Drain all unread frames and cache the latest
   * Called ONLY from periodic()
   * 
   * TIMESTAMP SEMANTICS:
   * - Cached timestamp is FPGA receive time (NOT capture time)
   * - This is conservative: we don't guess at latency
   * - Fusion uses receive time as measurement time
   */
  private void drainQuestNavFrames() {
    try {
      PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
      if (frames == null || frames.length == 0) {
        return; // No new data
      }
      
      // Get FPGA time when we received these frames (THIS IS THE MEASUREMENT TIMESTAMP)
      double fpgaReceiveTime = Timer.getFPGATimestamp();
      
      // Process ONLY the latest frame (discard older stale frames)
      PoseFrame latestFrame = frames[frames.length - 1];
      
      if (frames.length > 1) {
        SmartLogger.logReplay("QuestNav/FramesDiscarded", (double) (frames.length - 1));
      }
      
      // Get camera pose from Quest
      Pose3d cameraPose3d = latestFrame.questPose3d();
      if (cameraPose3d == null) {
        rejectedFrameCount++;
        SmartLogger.logReplay("QuestNav/RejectionReason", "Null camera pose");
        return;
      }
      
      // Get raw Quest timestamp (for debugging/monitoring only)
      double questRawTimestamp = latestFrame.dataTimestamp();
      
      // SANITY CHECK: Quest timestamp must be positive
      if (questRawTimestamp <= 0) {
        rejectedFrameCount++;
        SmartLogger.logReplay("QuestNav/RejectionReason", 
            String.format("Invalid Quest timestamp: %.3f", questRawTimestamp));
        return;
      }
      
      // Update timestamp offset for monitoring (NOT used for measurement timestamps)
      double newOffset = fpgaReceiveTime - questRawTimestamp;
      if (!timestampAligned) {
        timestampOffset = newOffset;
        timestampAligned = true;
      } else {
        timestampOffset = (1.0 - TIMESTAMP_FILTER_BETA) * timestampOffset + TIMESTAMP_FILTER_BETA * newOffset;
      }
      
      // SANITY CHECK: Reject extremely stale frames (> 2s old based on offset)
      double estimatedAge = fpgaReceiveTime - (questRawTimestamp + timestampOffset);
      if (estimatedAge < MIN_MEASUREMENT_AGE_SECONDS || estimatedAge > MAX_MEASUREMENT_AGE_SECONDS) {
        rejectedFrameCount++;
        SmartLogger.logReplay("QuestNav/RejectionReason", 
            String.format("Invalid estimated age: %.3fs (min=%.2f, max=%.2f)", 
                estimatedAge, MIN_MEASUREMENT_AGE_SECONDS, MAX_MEASUREMENT_AGE_SECONDS));
        return;
      }
      
      // Transform to robot pose
      Pose3d robotPose3d = cameraPose3d.transformBy(robotToQuest.inverse());
      Pose2d robotPose2d = new Pose2d(
          robotPose3d.getX(),
          robotPose3d.getY(),
          robotPose3d.getRotation().toRotation2d());
      
      // Increment sequence number (monotonic counter)
      currentSequence++;
      
      // Cache all values (only after passing sanity checks)
      cachedRobotPose = robotPose2d;
      cachedRobotPose3d = robotPose3d;
      cachedQuestTimestamp = questRawTimestamp;
      cachedFPGAReceiveTime = fpgaReceiveTime; // THIS IS THE MEASUREMENT TIMESTAMP
      cachedFrameSequence = currentSequence;
      
      totalFramesProcessed++;
      
      SmartLogger.logReplay("QuestNav/FrameAccepted", true);
      SmartLogger.logReplay("QuestNav/EstimatedAgeOnReceipt", estimatedAge);
      
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/GetFramesError", e.getMessage());
    }
  }
  
  /**
   * PEEK (NON-CONSUMING): Get latest measurement WITHOUT consuming it
   * Allows fusion to inspect measurement before deciding to accept/reject
   * 
   * @return QuestMeasurement if available, empty otherwise
   * GUARANTEE: Does NOT mark frame as consumed
   */
  public java.util.Optional<QuestMeasurement> peekLatestMeasurement() {
    // Check if there's an unconsumed measurement
    if (cachedRobotPose == null || cachedFrameSequence <= lastConsumedSequence) {
      return java.util.Optional.empty();
    }
    
    // Return measurement WITHOUT marking as consumed
    QuestMeasurement measurement = new QuestMeasurement(
        cachedRobotPose,
        cachedFPGAReceiveTime,
        cachedFrameSequence
    );
    
    return java.util.Optional.of(measurement);
  }
  
  /**
   * ACKNOWLEDGE: Mark measurement as consumed after successful fusion
   * Only call this after measurement has been accepted and added to pose estimator
   * 
   * @param sequence Sequence number to acknowledge
   * @return true if acknowledged, false if already consumed or invalid
   */
  public boolean acknowledgeMeasurement(long sequence) {
    // Verify sequence is the current unconsumed frame
    if (sequence != cachedFrameSequence || sequence <= lastConsumedSequence) {
      SmartLogger.logReplay("QuestNav/AcknowledgeFailed", 
          "Attempted to ack seq=" + sequence + " but current=" + cachedFrameSequence + 
          " lastConsumed=" + lastConsumedSequence);
      return false;
    }
    
    // Mark as consumed
    lastConsumedSequence = sequence;
    
    SmartLogger.logReplay("QuestNav/MeasurementAcknowledged", (double) sequence);
    
    return true;
  }
  
  /**
   * DEPRECATED: Old consume-once method - use peek + acknowledge instead
   */
  @Deprecated
  public java.util.Optional<QuestMeasurement> consumeLatestMeasurement() {
    var peeked = peekLatestMeasurement();
    if (peeked.isPresent()) {
      acknowledgeMeasurement(peeked.get().sequence);
    }
    return peeked;
  }
  
  /**
   * Check if there's an unconsumed measurement available
   * 
   * @return true if new frame available since last consumption
   */
  public boolean hasUnconsumedMeasurement() {
    return cachedRobotPose != null && cachedFrameSequence > lastConsumedSequence;
  }
  
  /**
   * Get cached robot pose (READ ONLY - for display/debugging)
   * WARNING: Does NOT consume measurement. Use consumeLatestMeasurement() for fusion.
   */
  public java.util.Optional<Pose2d> getRobotPose() {
    return java.util.Optional.ofNullable(cachedRobotPose);
  }
  
  /**
   * Get cached robot pose 3D (READ ONLY - for display/debugging)
   * WARNING: Does NOT consume measurement.
   */
  public java.util.Optional<Pose3d> getRobotPose3d() {
    return java.util.Optional.ofNullable(cachedRobotPose3d);
  }
  
  /**
   * Get cached measurement timestamp in FPGA time
   * 
   * TIMESTAMP SEMANTICS: This is FPGA RECEIVE time, NOT capture time
   * 
   * WARNING: Does NOT consume measurement. Use consumeLatestMeasurement() for fusion.
   */
  public java.util.Optional<Double> getMeasurementTimeFPGA() {
    return cachedFPGAReceiveTime >= 0 ? java.util.Optional.of(cachedFPGAReceiveTime) : java.util.Optional.empty();
  }
  
  /**
   * Get age since RECEIPT (not capture)
   * Measures how stale the cached measurement is
   */
  public double getMeasurementAge() {
    if (cachedFPGAReceiveTime < 0) return Double.POSITIVE_INFINITY;
    return Timer.getFPGATimestamp() - cachedFPGAReceiveTime; // Age since receipt
  }
  
  public Rotation2d getRotation() {
    return getRobotPose()
        .map(Pose2d::getRotation)
        .orElse(new Rotation2d());
  }
  
  /**
   * DEPRECATED: Use hasUnconsumedMeasurement() instead
   * This method does NOT check consumption state!
   * 
   * Kept only for backwards compatibility - will be removed in future.
   */
  @Deprecated
  public boolean hasNewData() {
    // Simply delegate to correct method
    return hasUnconsumedMeasurement();
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
  
  /**
   * Initialize QuestNav to a specific robot pose.
   * Used when manually resetting robot pose (e.g. auto start, manual alignment).
   * 
   * @param initialPose Robot pose to initialize to (field-space)
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
      SmartLogger.logConsole("QuestNav initialized to: " + formatPose(initialPose));
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/InitError", e.getMessage());
      SmartLogger.logConsoleError("QuestNav initialization failed: " + e.getMessage());
    }
  }
  
  /**
   * Calibrate from vision (optional override for future use).
   * 
   * @param visionPose Robot pose from vision (field-space)
   * @param confidence Confidence level (0-1)
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
      SmartLogger.logConsole("QuestNav calibrated from vision: " + formatPose(visionPose));
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/CalibrationError", e.getMessage());
    }
  }
  
  /**
   * SINGLE CONSUMER: Removed frame draining from testConnection()
   * Now only checks status without consuming frames
   */
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
    
    // Frames will be drained in periodic() via drainQuestNavFrames()
    SmartLogger.logConsole("Pose data will be available after first periodic() cycle");
  }
  
  private String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", 
        pose.getX(), 
        pose.getY(), 
        pose.getRotation().getDegrees());
  }
}
