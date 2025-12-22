package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.Logger;

// Unified logging utility - routes to console (System.out) or AdvantageKit Logger
// Console output controlled by configure(), AdvantageKit always enabled
public class SmartLogger {
  
  private static boolean s_consoleEnabled = true;
  
  // Configure console logging (AdvantageKit always logs regardless)
  public static void configure(boolean enableConsole) {
    s_consoleEnabled = enableConsole;
    
    String configMessage = "Console: " + (s_consoleEnabled ? "ENABLED" : "DISABLED") + "\n" +
                          "Replay: ALWAYS ENABLED (controlled by Logger setup in Robot.java)";
    logConsole(configMessage, "SmartLogger Config");
  }
  
  // Console logging (controlled by flag)
  public static void logConsole(String message) {
    if (s_consoleEnabled) System.out.println(message);
  }
  
  public static void logConsole(String message, String sectionHeader) {
    logConsole(message, sectionHeader, 10);
  }
  
  public static void logConsole(String message, String sectionHeader, int equalsCount) {
    if (!s_consoleEnabled) return;
    
    String separator = "=".repeat(equalsCount) + " " + sectionHeader + " " + "=".repeat(equalsCount);
    System.out.println(separator);
    System.out.println(message);
  }
  
  public static void logConsoleError(String message) {
    if (s_consoleEnabled) System.err.println(message);
  }
  
  // Replay logging (ALWAYS logs to AdvantageKit - no flag check!)
  public static void logReplay(String key, String value) {
    Logger.recordOutput(key, value);
  }
  
  public static void logReplay(String key, double value) {
    Logger.recordOutput(key, value);
  }
  
  public static void logReplay(String key, boolean value) {
    Logger.recordOutput(key, value);
  }
  
  public static void logReplay(String key, Pose2d value) {
    Logger.recordOutput(key, value);
  }
  
  public static void logReplay(String key, Pose3d value) {
    Logger.recordOutput(key, value);
  }
  
  public static void logReplay(String key, double[] value) {
    Logger.recordOutput(key, value);
  }
}
