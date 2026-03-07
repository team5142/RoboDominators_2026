package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

// Monitors roboRIO disk space and deletes old .wpilog files to stay healthy.
// Logs are flat .wpilog files stored directly in LOG_DIR.
// Cleanup deletes oldest files first until enough space is recovered.
// DS alerts are shown once per state transition — no log spam.
public class LogSpaceMonitor {

  private static final double WARNING_THRESHOLD_GB  = 1.5; // delete old logs below this
  private static final double CRITICAL_THRESHOLD_GB = 0.5; // DS error below this
  private static final double TARGET_FREE_GB        = 2.0; // recover to this after cleanup
  private static final int    CHECK_INTERVAL_SECONDS = 30;
  private static final String LOG_DIR_PATH = "/home/lvuser/logs";

  private enum DiskState { OK, WARNING, CRITICAL }
  private static DiskState s_lastState = DiskState.OK;

  private static double s_lastCheckTime = 0.0;
  private static final File ROOT    = new File("/"); // actual roboRIO root filesystem
  private static final File LOG_DIR = new File(LOG_DIR_PATH);

  public static void periodic() {
    double now = RobotController.getFPGATime() / 1_000_000.0;
    if (now - s_lastCheckTime < CHECK_INTERVAL_SECONDS) return;
    s_lastCheckTime = now;

    double freeGB = toGB(ROOT.getFreeSpace());

    if (freeGB < WARNING_THRESHOLD_GB) {
      cleanupOldSessions();
      freeGB = toGB(ROOT.getFreeSpace());
    }

    double totalGB  = toGB(ROOT.getTotalSpace());
    double usedGB   = totalGB - freeGB;
    double usedPct  = (usedGB / totalGB) * 100.0;

    SmartLogger.logReplay("LogSpace/FreeGB",      freeGB);
    SmartLogger.logReplay("LogSpace/UsedGB",      usedGB);
    SmartLogger.logReplay("LogSpace/TotalGB",     totalGB);
    SmartLogger.logReplay("LogSpace/UsedPercent", usedPct);

    // Compute new state and only alert on transitions
    DiskState newState;
    if (freeGB < CRITICAL_THRESHOLD_GB)      newState = DiskState.CRITICAL;
    else if (freeGB < WARNING_THRESHOLD_GB)  newState = DiskState.WARNING;
    else                                     newState = DiskState.OK;

    if (newState != s_lastState) {
      if (newState == DiskState.CRITICAL) {
        DriverStation.reportError(
            "!!! LOGS FULL - LOGGING WILL FAIL - DELETE OLD LOGS NOW !!!", false);
      } else if (newState == DiskState.WARNING) {
        DriverStation.reportWarning(
            "WARNING: LOGS FULL - CLEAR OLD LOG FILES FROM ROBORIO", false);
      }
      if (newState == DiskState.OK) {
        SmartLogger.logConsole("Disk space recovered - logging healthy", "LogSpace");
      }
      s_lastState = newState;
    }

    SmartLogger.logReplay("LogSpace/Status", newState.name());

    // Print to console once every 5 minutes (not every 30s)
    if ((int)(now / CHECK_INTERVAL_SECONDS) % 10 == 0) {
      SmartLogger.logConsole(
          String.format("[LogSpace] %.2fGB / %.2fGB free (%.1f%% used)", freeGB, totalGB, usedPct),
          "LogSpace", 5);
    }
  }

  public static String getStatusString() {
    double freeGB  = toGB(ROOT.getFreeSpace());
    double totalGB = toGB(ROOT.getTotalSpace());
    return String.format("%.2fGB / %.2fGB free (%.1f%% used)",
        freeGB, totalGB, ((totalGB - freeGB) / totalGB) * 100.0);
  }

  // Deletes oldest .wpilog files until TARGET_FREE_GB is reached.
  // Never deletes the single most-recently-modified file.
  private static void cleanupOldSessions() {
    if (!LOG_DIR.exists() || !LOG_DIR.isDirectory()) return;

    File[] logs = LOG_DIR.listFiles(f -> f.isFile() && f.getName().endsWith(".wpilog"));
    if (logs == null || logs.length <= 1) return;

    Arrays.sort(logs, Comparator.comparingLong(File::lastModified));
    long newestTime = logs[logs.length - 1].lastModified();

    int deleted = 0;
    for (File log : logs) {
      if (toGB(ROOT.getFreeSpace()) >= TARGET_FREE_GB) break;
      if (log.lastModified() == newestTime) continue;
      if (log.delete()) deleted++;
    }

    if (deleted > 0) {
      SmartLogger.logConsole(
          "Deleted " + deleted + " old log files to free space", "LogSpace", 5);
    }
  }

  private static double toGB(long bytes) {
    return bytes / (1024.0 * 1024.0 * 1024.0);
  }
}
