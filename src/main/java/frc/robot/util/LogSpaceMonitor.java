package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

/**
 * Monitors RoboRIO storage space for AdvantageKit logs
 * Warns when disk space is low to prevent logging failures
 */
public class LogSpaceMonitor {
  
  private static final double WARNING_THRESHOLD_GB = 1.0;  // Warn when < 1GB free
  private static final double CRITICAL_THRESHOLD_GB = 0.5; // Critical when < 500MB free
  private static final int CHECK_INTERVAL_SECONDS = 30;    // Check every 30s (not every loop)
  private static final double CLEANUP_EXTRA_GB = 0.25;
  private static final String LOG_EXTENSION = ".wpilog";
  
  private static double s_lastCheckTime = 0.0;
  private static boolean s_hasWarned = false;
  private static boolean s_hasCriticalWarned = false;
  private static final File ROOT = Filesystem.getOperatingDirectory();
  private static final File LOG_DIR = new File("/home/lvuser/logs");
  
  /**
   * Check disk space and log warnings if low (call in Robot.periodic())
   * Only checks every 30s to avoid performance impact
   */
  public static void periodic() {
    double currentTime = RobotController.getFPGATime() / 1_000_000.0; // Convert to seconds
    
    // Only check every 30 seconds
    if (currentTime - s_lastCheckTime < CHECK_INTERVAL_SECONDS) {
      return;
    }
    
    s_lastCheckTime = currentTime;
    
    long freeBytes = ROOT.getFreeSpace();
    long totalBytes = ROOT.getTotalSpace();
    double freeGB = toGB(freeBytes);
    double totalGB = toGB(totalBytes);

    if (freeGB < WARNING_THRESHOLD_GB) {
      cleanupOldLogs(totalGB);
      freeBytes = ROOT.getFreeSpace();
      totalBytes = ROOT.getTotalSpace();
      freeGB = toGB(freeBytes);
      totalGB = toGB(totalBytes);
    }

    long usedBytes = totalBytes - freeBytes;
    double usedGB = toGB(usedBytes);
    double usedPercent = (usedGB / totalGB) * 100.0;
    
    // Log to AdvantageKit
    SmartLogger.logReplay("LogSpace/FreeGB", freeGB);
    SmartLogger.logReplay("LogSpace/UsedGB", usedGB);
    SmartLogger.logReplay("LogSpace/TotalGB", totalGB);
    SmartLogger.logReplay("LogSpace/UsedPercent", usedPercent);
    
    // Check thresholds and warn
    if (freeGB < CRITICAL_THRESHOLD_GB) {
      if (!s_hasCriticalWarned) {
        String criticalMsg = String.format("CRITICAL: Only %.2fGB free! Logging may fail soon!", freeGB);
        SmartLogger.logConsoleError(criticalMsg);
        SmartLogger.logReplay("LogSpace/CriticalWarning", criticalMsg);
        s_hasCriticalWarned = true;
      }
      SmartLogger.logReplay("LogSpace/Status", "CRITICAL");
      
    } else if (freeGB < WARNING_THRESHOLD_GB) {
      if (!s_hasWarned) {
        String warnMsg = String.format("WARNING: Only %.2fGB free - consider clearing old logs", freeGB);
        SmartLogger.logConsole(warnMsg, "Low Disk Space", 5);
        SmartLogger.logReplay("LogSpace/Warning", warnMsg);
        s_hasWarned = true;
      }
      SmartLogger.logReplay("LogSpace/Status", "WARNING");
      
    } else {
      // Reset warnings when space recovered
      s_hasWarned = false;
      s_hasCriticalWarned = false;
      SmartLogger.logReplay("LogSpace/Status", "OK");
    }
    
    // Periodic console output (every 5 minutes = 10 checks)
    if ((int)(currentTime / CHECK_INTERVAL_SECONDS) % 10 == 0) {
      SmartLogger.logConsole(
          String.format("[LogSpace] %.2fGB / %.2fGB free (%.1f%% used)", freeGB, totalGB, usedPercent),
          "LogSpace",
          5);
    }
  }
  
  /**
   * Get human-readable disk space summary
   */
  public static String getStatusString() {
    long freeBytes = ROOT.getFreeSpace();
    long totalBytes = ROOT.getTotalSpace();
    
    double freeGB = toGB(freeBytes);
    double totalGB = toGB(totalBytes);
    double usedPercent = ((totalGB - freeGB) / totalGB) * 100.0;
    
    return String.format("%.2fGB / %.2fGB free (%.1f%% used)", freeGB, totalGB, usedPercent);
  }

  private static void cleanupOldLogs(double totalGB) {
    if (!LOG_DIR.exists() || !LOG_DIR.isDirectory()) {
      return;
    }

    File[] logFiles = LOG_DIR.listFiles((dir, name) -> name.endsWith(LOG_EXTENSION));
    if (logFiles == null || logFiles.length == 0) {
      return;
    }

    long newestModified = 0;
    for (File file : logFiles) {
      newestModified = Math.max(newestModified, file.lastModified());
    }

    Arrays.sort(logFiles, Comparator.comparingLong(File::lastModified));

    double targetGB = Math.min(WARNING_THRESHOLD_GB + CLEANUP_EXTRA_GB, totalGB - 0.1);
    targetGB = Math.max(WARNING_THRESHOLD_GB, targetGB);
    int deletedCount = 0;

    for (File file : logFiles) {
      if (file.lastModified() == newestModified) {
        continue;
      }

      if (file.delete()) {
        deletedCount++;
      }

      if (toGB(ROOT.getFreeSpace()) >= targetGB) {
        break;
      }
    }

    if (deletedCount > 0) {
      SmartLogger.logConsole("Deleted " + deletedCount + " old log files to free space", "LogSpace", 5);
    }
  }

  private static double toGB(long bytes) {
    return bytes / (1024.0 * 1024.0 * 1024.0);
  }
}
