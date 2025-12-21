package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;

/**
 * Monitors RoboRIO storage space for AdvantageKit logs
 * Warns when disk space is low to prevent logging failures
 */
public class LogSpaceMonitor {
  
  private static final double WARNING_THRESHOLD_GB = 1.0;  // Warn when < 1GB free
  private static final double CRITICAL_THRESHOLD_GB = 0.5; // Critical when < 500MB free
  private static final int CHECK_INTERVAL_SECONDS = 30;    // Check every 30s (not every loop)
  
  private static double s_lastCheckTime = 0.0;
  private static boolean s_hasWarned = false;
  private static boolean s_hasCriticalWarned = false;
  
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
    
    // Get RoboRIO root filesystem
    File root = Filesystem.getOperatingDirectory();
    
    // Get free space
    long freeBytes = root.getFreeSpace();
    long totalBytes = root.getTotalSpace();
    long usedBytes = totalBytes - freeBytes;
    
    // Convert to GB
    double freeGB = freeBytes / (1024.0 * 1024.0 * 1024.0);
    double totalGB = totalBytes / (1024.0 * 1024.0 * 1024.0);
    double usedGB = usedBytes / (1024.0 * 1024.0 * 1024.0);
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
      System.out.printf("[LogSpace] %.2fGB / %.2fGB free (%.1f%% used)%n", freeGB, totalGB, usedPercent);
    }
  }
  
  /**
   * Get human-readable disk space summary
   */
  public static String getStatusString() {
    File root = Filesystem.getOperatingDirectory();
    long freeBytes = root.getFreeSpace();
    long totalBytes = root.getTotalSpace();
    
    double freeGB = freeBytes / (1024.0 * 1024.0 * 1024.0);
    double totalGB = totalBytes / (1024.0 * 1024.0 * 1024.0);
    double usedPercent = ((totalGB - freeGB) / totalGB) * 100.0;
    
    return String.format("%.2fGB / %.2fGB free (%.1f%% used)", freeGB, totalGB, usedPercent);
  }
}
