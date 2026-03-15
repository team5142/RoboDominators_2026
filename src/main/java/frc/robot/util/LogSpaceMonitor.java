package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

// Monitors roboRIO disk space and deletes old .hoot files to stay healthy.
// .wpilog (AKit) files are NEVER deleted — fetch them with fetch-logs.ps1 after each event.
// .hoot (CTRE SignalLogger) files are deleted oldest-first when space is low.
// SignalLogger.stop() in robotInit() prevents new .hoot files from being created,
// but this cleanup handles any that already exist on disk.
// DS alerts are shown once per state transition — no log spam.
public class LogSpaceMonitor {

  private static final double WARNING_THRESHOLD_GB  = 1.5;
  private static final double CRITICAL_THRESHOLD_GB = 0.5;
  private static final double TARGET_FREE_GB        = 2.0;
  private static final int    CHECK_INTERVAL_SECONDS = 30;
  private static final String LOG_DIR_PATH = "/home/lvuser/logs";

  private enum DiskState { OK, WARNING, CRITICAL }
  private static DiskState s_lastState = DiskState.OK;

  private static double s_lastCheckTime = 0.0;
  private static final File ROOT    = new File("/");
  private static final File LOG_DIR = new File(LOG_DIR_PATH);

  public static void periodic() {
    double now = RobotController.getFPGATime() / 1_000_000.0;
    if (now - s_lastCheckTime < CHECK_INTERVAL_SECONDS) return;
    s_lastCheckTime = now;

    double freeGB = toGB(ROOT.getFreeSpace());

    if (freeGB < WARNING_THRESHOLD_GB) {
      cleanupHootFiles();
      freeGB = toGB(ROOT.getFreeSpace());
    }

    double totalGB  = toGB(ROOT.getTotalSpace());
    double usedGB   = totalGB - freeGB;
    double usedPct  = (usedGB / totalGB) * 100.0;

    SmartLogger.logReplay("LogSpace/FreeGB",      freeGB);
    SmartLogger.logReplay("LogSpace/UsedGB",      usedGB);
    SmartLogger.logReplay("LogSpace/TotalGB",     totalGB);
    SmartLogger.logReplay("LogSpace/UsedPercent", usedPct);

    DiskState newState;
    if (freeGB < CRITICAL_THRESHOLD_GB)      newState = DiskState.CRITICAL;
    else if (freeGB < WARNING_THRESHOLD_GB)  newState = DiskState.WARNING;
    else                                     newState = DiskState.OK;

    if (newState != s_lastState) {
      if (newState == DiskState.CRITICAL) {
        DriverStation.reportError(
            "!!! DISK FULL - FETCH LOGS AND DELETE .HOOT FILES FROM ROBORIO !!!", false);
      } else if (newState == DiskState.WARNING) {
        DriverStation.reportWarning(
            "WARNING: DISK SPACE LOW - FETCH LOGS FROM ROBORIO SOON", false);
      }
      if (newState == DiskState.OK) {
        SmartLogger.logConsole("Disk space recovered - logging healthy", "LogSpace");
      }
      s_lastState = newState;
    }

    SmartLogger.logReplay("LogSpace/Status", newState.name());

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

  // Deletes .hoot files (oldest first) until TARGET_FREE_GB is reached.
  // .wpilog files are never touched — they are our match replay logs.
  private static void cleanupHootFiles() {
    if (!LOG_DIR.exists() || !LOG_DIR.isDirectory()) return;

    File[] hoots = LOG_DIR.listFiles(f -> f.isFile() && f.getName().endsWith(".hoot"));
    if (hoots == null || hoots.length == 0) return;

    Arrays.sort(hoots, Comparator.comparingLong(File::lastModified));

    int deleted = 0;
    for (File hoot : hoots) {
      if (toGB(ROOT.getFreeSpace()) >= TARGET_FREE_GB) break;
      if (hoot.delete()) deleted++;
    }

    if (deleted > 0) {
      SmartLogger.logConsole(
          "Deleted " + deleted + " .hoot files to free space (.wpilog files preserved)", "LogSpace", 5);
    }
  }

  private static double toGB(long bytes) {
    return bytes / (1024.0 * 1024.0 * 1024.0);
  }
}
