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
import frc.robot.subsystems.pose.QuestNavWakeService;
import frc.robot.util.SmartLogger;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;

import java.util.Optional;
import java.util.OptionalInt;

public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav;
  private final QuestNavWakeService wakeService;
  private final Transform3d robotToQuest;
  private final Transform3d robotToQuestInverse; // cached to avoid recomputing every frame
  
  private Pose2d cachedRobotPose = null;
  private Pose3d cachedRobotPose3d = null;
  private double cachedMeasurementTimestamp = -1.0;
  private long cachedFrameCount = -1;
  
  private long lastConsumedFrameCount = -1;
  
  private boolean isSeeded = false;
  
  private String lastRejectionReason = "";
  private long lastAcceptedFrameCount = -1;

  private boolean fusionPaused = false;

  private int logCounter = 0;
  private static final int LOG_SKIP_CYCLES = 9;

  private static final Rotation2d ZERO_ROTATION = new Rotation2d();

  // Cached every loop - avoids repeated NT reads in the same cycle
  private boolean cachedConnected = false;
  private boolean cachedTracking = false;
  
  public static class QuestMeasurement {
    public final Pose2d pose;
    public final double measurementTimestamp;
    public final long frameCount;
    
    public QuestMeasurement(Pose2d pose, double measurementTimestamp, long frameCount) {
      this.pose = pose;
      this.measurementTimestamp = measurementTimestamp;
      this.frameCount = frameCount;
    }
  }
  
  public QuestNavSubsystem() {
    SmartLogger.logConsole("QuestNavSubsystem initializing...", "QuestNav");
    
    // Start our network wake and tracking recovery service
    wakeService = new QuestNavWakeService(QUEST_IP_ADDRESS, QUEST_PORT);
    wakeService.start();
    
    try {
      questNav = new QuestNav();
      SmartLogger.logConsole("QuestNav object created successfully", "QuestNav");
    } catch (Exception e) {
      SmartLogger.logConsoleError("Failed to create QuestNav object: " + e.getMessage());
      throw e;
    }
    
    robotToQuest = new Transform3d(
        new Translation3d(QUEST_X_METERS, QUEST_Y_METERS, QUEST_Z_METERS),
        new Rotation3d(0, 0, Math.toRadians(QUEST_YAW_DEG))
    );
    robotToQuestInverse = robotToQuest.inverse();
    
    SmartLogger.logConsole("Robot-to-Quest transform: " + robotToQuest, "QuestNav");
    
    testConnectionWithTimeout(2000);
    
    SmartLogger.logConsole("========================================\n", "QuestNav");
  }
  
  private void testConnectionWithTimeout(long timeoutMs) {
    Thread testThread = new Thread(() -> {
      try {
        testConnection();
      } catch (Exception e) {
        SmartLogger.logConsoleError("Connection test failed: " + e.getMessage());
      }
    });
    
    testThread.setDaemon(true);
    testThread.setName("QuestNav-ConnectionTest");
    testThread.start();
    
    try {
      testThread.join(timeoutMs);
      
      if (testThread.isAlive()) {
        SmartLogger.logConsoleError("========================================");
        SmartLogger.logConsoleError("QuestNav connection test TIMEOUT (2s)!");
        SmartLogger.logConsoleError("Quest may be disconnected or hung");
        SmartLogger.logConsoleError("Robot will boot without Quest");
        SmartLogger.logConsoleError("========================================");
        testThread.interrupt();
      }
    } catch (InterruptedException e) {
      SmartLogger.logConsoleError("Connection test interrupted");
      Thread.currentThread().interrupt();
    }
  }
  
  // commandPeriodic() can block 10-60ms when Quest is busy. Run it at 10Hz max.
  private int commandPeriodicCounter = 0;
  private static final int COMMAND_PERIODIC_SKIP = 4; // every 5th cycle = 10Hz

  // TODO (robot session): verify these before fixing:
  // #1 - fusionPaused does not stop drainQuestNavFrames() - frames still cache while paused.
  //      Fix: add early return in drainQuestNavFrames() when fusionPaused=true.
  //      Risk: PoseEstimatorSubsystem may already gate on isFusionPaused() before using the result.
  // #2 - cachedMeasurementTimestamp uses FPGA "now", not questNativeTimestamp (actual capture time).
  //      Fix: use questNativeTimestamp when > 0 as the measurement timestamp fed to pose estimator.
  //      Risk: changes fusion math - verify Quest clock vs RoboRIO clock sync is reliable first.
  // #3 - checkConnectedRaw() reads getBatteryPercent() via NT every cycle to probe connection.
  //      Fix: use questNav.isConnected() as the primary check; reserve battery for the log-rate call.
  //      Risk: low - but confirm questNav.isConnected() is a cheap NT read before swapping.
  // #5 - enterResumeMode() sets isSeeded=false but leaves stale cachedRobotPose intact.
  //      Fix: clear cache fields same as seedToPose() does, or document that callers expect continuity.
  //      Risk: any caller reading getRobotPose() right after resume gets the old (possibly stale) pose.

  @Override
  public void periodic() {
    if (commandPeriodicCounter++ % (COMMAND_PERIODIC_SKIP + 1) == 0) {
      try {
        questNav.commandPeriodic();
      } catch (Exception e) {
        SmartLogger.logReplay("QuestNav/PeriodicError", e.getMessage());
      }
    }

    // Refresh connection state once per loop - all callers use these cached values
    cachedConnected = checkConnectedRaw();
    cachedTracking = checkTrackingRaw();

    drainQuestNavFrames();
    
    logCounter++;
    boolean doLog = (logCounter % (LOG_SKIP_CYCLES + 1) == 0);

    if (doLog) {
      SmartLogger.logReplay("QuestNav/Connected", cachedConnected);
      SmartLogger.logReplay("QuestNav/Tracking", cachedTracking);
      SmartLogger.logReplay("QuestNav/Seeded", isSeeded);
      SmartLogger.logReplay("QuestNav/Battery", (double) getBatteryPercent());
    }

    if (cachedRobotPose != null && doLog) {
      SmartLogger.logReplay("QuestNav/RobotPose", cachedRobotPose);
    }
  }
  
  public void pauseFusion() {
    fusionPaused = true;
  }
  
  public void resumeFusion() {
    fusionPaused = false;
    SmartLogger.logReplay("QuestNav/FusionPaused", false);
    SmartLogger.logConsole("Fusion resumed", "QuestNav");
  }
  
  public boolean isFusionPaused() {
    return fusionPaused;
  }
  
  private void drainQuestNavFrames() {
    try {
      PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
      if (frames == null || frames.length == 0) return;
      
      PoseFrame latestFrame = frames[frames.length - 1];
      
      if (frames.length > 1) {
        SmartLogger.logReplay("QuestNav/FramesDiscarded", (double) (frames.length - 1));
      }
      
      double measurementTimestamp = Timer.getFPGATimestamp();
      long frameCount = latestFrame.frameCount();
      
      double questNativeTimestamp = latestFrame.dataTimestamp();
      if (questNativeTimestamp > 0) {
        SmartLogger.logReplay("QuestNav/QuestNativeTimestamp", questNativeTimestamp);
        SmartLogger.logReplay("QuestNav/ClockSkew", measurementTimestamp - questNativeTimestamp);
      }
      
      Pose3d cameraPose3d = latestFrame.questPose3d();
      if (cameraPose3d == null) {
        String reason = "Null camera pose";
        logRejectionIfChanged(reason);
        return;
      }
      
      Pose3d robotPose3d = cameraPose3d.transformBy(robotToQuestInverse);
      Pose2d robotPose2d = new Pose2d(
          robotPose3d.getX(),
          robotPose3d.getY(),
          robotPose3d.getRotation().toRotation2d());
      
      long frameGap = (lastAcceptedFrameCount > 0) ? (frameCount - lastAcceptedFrameCount - 1) : 0;
      if (frameGap > 0) {
        SmartLogger.logReplay("QuestNav/FrameGap", (double) frameGap);
      }
      
      cachedRobotPose = robotPose2d;
      cachedRobotPose3d = robotPose3d;
      cachedMeasurementTimestamp = measurementTimestamp;
      cachedFrameCount = frameCount;
      
      lastAcceptedFrameCount = frameCount;
      
      SmartLogger.logReplay("QuestNav/FrameAccepted", true);
      SmartLogger.logReplay("QuestNav/AcceptedFrameCount_Native", (double) frameCount);
      
    } catch (Exception e) {
      String reason = "Exception: " + e.getMessage();
      logRejectionIfChanged(reason);
      SmartLogger.logReplay("QuestNav/GetFramesError", e.getMessage());
    }
  }
  
  private void logRejectionIfChanged(String reason) {
    if (!reason.equals(lastRejectionReason)) {
      SmartLogger.logReplay("QuestNav/RejectionReason", reason);
      lastRejectionReason = reason;
    }
  }
  
  public Optional<QuestMeasurement> peekLatestMeasurement() {
    if (cachedRobotPose == null || cachedFrameCount <= lastConsumedFrameCount) {
      return Optional.empty();
    }
    
    QuestMeasurement measurement = new QuestMeasurement(
        cachedRobotPose,
        cachedMeasurementTimestamp,
        cachedFrameCount
    );
    
    return Optional.of(measurement);
  }
  
  public boolean acknowledgeMeasurement(long frameCount) {
    // Already acknowledged - silent no-op, not an error
    if (frameCount == lastConsumedFrameCount) return true;

    if (frameCount != cachedFrameCount || frameCount <= lastConsumedFrameCount) {
      SmartLogger.logReplay("QuestNav/AcknowledgeFailed",
          "Attempted to ack frame=" + frameCount + " but current=" + cachedFrameCount +
          " lastConsumed=" + lastConsumedFrameCount);
      return false;
    }
    
    lastConsumedFrameCount = frameCount;
    SmartLogger.logReplay("QuestNav/MeasurementAcknowledged", (double) frameCount);
    return true;
  }
  
  @Deprecated
  public Optional<QuestMeasurement> consumeLatestMeasurement() {
    SmartLogger.logConsoleError("consumeLatestMeasurement() is deprecated! Use QuestNavFusion.forceAccept() instead");
    return Optional.empty();
  }
  
  public boolean hasUnconsumedMeasurement() {
    return cachedRobotPose != null && cachedFrameCount > lastConsumedFrameCount;
  }
  
  public Optional<Pose2d> getRobotPose() {
    return Optional.ofNullable(cachedRobotPose);
  }
  
  public Optional<Pose3d> getRobotPose3d() {
    return Optional.ofNullable(cachedRobotPose3d);
  }
  
  public double getMeasurementAge() {
    if (cachedMeasurementTimestamp < 0) return Double.POSITIVE_INFINITY;
    
    double currentTime = Timer.getFPGATimestamp();
    return currentTime - cachedMeasurementTimestamp;
  }
  
  public Rotation2d getRotation() {
    return getRobotPose()
        .map(Pose2d::getRotation)
        .orElse(ZERO_ROTATION);
  }
  
  public boolean isConnected() {
    return cachedConnected;
  }

  public boolean isTracking() {
    return cachedTracking;
  }

  private boolean checkConnectedRaw() {
    try {
      OptionalInt battery = questNav.getBatteryPercent();
      if (battery.isPresent()) return true;
      
      OptionalInt frameCount = questNav.getFrameCount();
      if (frameCount.isPresent() && frameCount.getAsInt() >= 0) return true;
      
      return questNav.isConnected();
    } catch (Exception e) {
      return false;
    }
  }
  
  private boolean checkTrackingRaw() {
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
      
      cachedRobotPose = null;
      cachedRobotPose3d = null;
      cachedMeasurementTimestamp = -1.0;
      cachedFrameCount = -1;
      lastConsumedFrameCount = -1;
      lastAcceptedFrameCount = -1;
      
      SmartLogger.logReplay("QuestNav/SeedToPose", fieldPose);
      SmartLogger.logConsole("QuestNav seeded to: " + SmartLogger.formatPose(fieldPose), "QuestNav");
    } catch (Exception e) {
      SmartLogger.logReplay("QuestNav/SeedError", e.getMessage());
      SmartLogger.logConsoleError("QuestNav seed failed: " + e.getMessage());
    }
  }
  
  public void enterResumeMode() {
    isSeeded = false;
    SmartLogger.logConsole("QuestNav in RESUME mode - using existing tracking", "QuestNav");
    SmartLogger.logReplay("QuestNav/ResumeMode", true);
  }

  /**
   * Forces the headset to restart its background VR tracking app using our WakeService.
   */
  public void triggerAppRestart() {
    wakeService.triggerAppRestart();
  }
  
  private void testConnection() {
    SmartLogger.logConsole("Testing QuestNav connection (USB > Ethernet > RoboRIO)", "QuestNav");
    
    boolean tracking = isTracking();
    SmartLogger.logConsole("Tracking: " + tracking, "QuestNav");
    
    int battery = getBatteryPercent();
    if (battery >= 0) {
      SmartLogger.logConsole("Battery: " + battery + "%", "QuestNav");
      if (battery < 20) {
        SmartLogger.logConsoleError("Quest battery low!");
      }
    }
    
    if (!tracking) {
      SmartLogger.logConsoleError("QuestNav NOT tracking - check USB/Ethernet connection");
    } else {
      SmartLogger.logConsole("QuestNav is tracking!", "QuestNav");
    }
    
    SmartLogger.logConsole("Pose data will be available after first periodic() cycle", "QuestNav");
  }
}