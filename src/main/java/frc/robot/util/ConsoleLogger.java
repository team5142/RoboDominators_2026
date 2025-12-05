package frc.robot.util;

import org.littletonrobotics.junction.Logger;

/**
 * Mirrors console messages into AdvantageKit so they are visible in AdvantageScope.
 */
public class ConsoleLogger {
  // Fixed-size rolling buffer (very simple)
  private static final int MAX_LINES = 200;
  private static final StringBuilder buffer = new StringBuilder();
  private static int lineCount = 0;

  public static void log(String msg) {
    // Print to normal console
    System.out.println(msg);

    // Append to rolling buffer
    buffer.append(msg).append("\n");
    lineCount++;
    if (lineCount > MAX_LINES) {
      // Trim from the start when buffer gets large (simple heuristic)
      int firstNewline = buffer.indexOf("\n");
      if (firstNewline >= 0) {
        buffer.delete(0, firstNewline + 1);
        lineCount--;
      }
    }

    // Record to AdvantageKit
    Logger.recordOutput("ConsoleAK/LastMessage", msg);
    Logger.recordOutput("ConsoleAK/Log", buffer.toString());
  }

  public static void logError(String msg) {
    System.err.println(msg);
    log("[ERROR] " + msg);
  }
}
