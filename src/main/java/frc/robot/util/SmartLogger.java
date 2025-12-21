package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;

// Unified logging utility - routes to console (System.out) or AdvantageKit Logger
// Use configure() to enable/disable each independently
public class SmartLogger {
  
  private static boolean s_consoleEnabled = true;  // System.out for live debugging
  private static boolean s_replayEnabled = true;   // AdvantageKit Logger for post-match analysis
  
  // Configure logging systems (call once in RobotContainer constructor)
  public static void configure(boolean enableConsole, boolean enableReplay) {
    s_consoleEnabled = enableConsole;
    s_replayEnabled = enableReplay;
    
    // Log config with visual separator
    String configMessage = "Console (System.out): " + (s_consoleEnabled ? "ENABLED" : "DISABLED") + "\n" +
                          "Replay (AdvantageKit): " + (s_replayEnabled ? "ENABLED" : "DISABLED");
    logConsole(configMessage, "SmartLogger Configuration");
    
    logReplay("SmartLogger/ConsoleEnabled", s_consoleEnabled);
    logReplay("SmartLogger/ReplayEnabled", s_replayEnabled);
  }
  
  // ===== CONSOLE LOGGING (System.out) - Live debugging =====
  
  public static void logConsole(String message) {
    if (s_consoleEnabled) System.out.println(message);
  }
  
  // Log with section header (default 10 equals per side)
  public static void logConsole(String message, String sectionHeader) {
    logConsole(message, sectionHeader, 10);
  }
  
  // Log with section header (custom equals count per side)
  public static void logConsole(String message, String sectionHeader, int equalsCount) {
    if (!s_consoleEnabled) return;
    
    // Build separator: "========== Header =========="
    String separator = "=".repeat(equalsCount) + " " + sectionHeader + " " + "=".repeat(equalsCount);
    
    System.out.println(separator);
    System.out.println(message);
  }
  
  public static void logConsoleError(String message) {
    if (s_consoleEnabled) System.err.println(message);
  }
  
  // ===== REPLAY LOGGING (AdvantageKit Logger) - Post-match analysis =====
  
  // String values
  public static void logReplay(String key, String value) {
    if (s_replayEnabled) Logger.recordOutput(key, value);
  }
  
  // Numbers
  public static void logReplay(String key, double value) {
    if (s_replayEnabled) Logger.recordOutput(key, value);
  }
  
  // Booleans
  public static void logReplay(String key, boolean value) {
    if (s_replayEnabled) Logger.recordOutput(key, value);
  }
  
  // Poses
  public static void logReplay(String key, Pose2d value) {
    if (s_replayEnabled) Logger.recordOutput(key, value);
  }
  
  // Arrays
  public static void logReplay(String key, double[] value) {
    if (s_replayEnabled) Logger.recordOutput(key, value);
  }
}
