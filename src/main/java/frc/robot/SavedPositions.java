package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;

/**
 * Stores custom field positions recorded during practice/matches
 */
public class SavedPositions {
  private static Pose2d customPosition = null;
  private static boolean hasCustomPosition = false;

  /**
   * Save the current robot position as a custom waypoint
   */
  public static void saveCustomPosition(Pose2d pose) {
    customPosition = pose;
    hasCustomPosition = true;
    
    Logger.recordOutput("SavedPositions/CustomPose", pose);
    Logger.recordOutput("SavedPositions/HasCustom", true);
    
    System.out.println("=== CUSTOM POSITION SAVED ===");
    System.out.println("X: " + String.format("%.2f", pose.getX()) + "m");
    System.out.println("Y: " + String.format("%.2f", pose.getY()) + "m");
    System.out.println("Rotation: " + String.format("%.1f", pose.getRotation().getDegrees()) + "°");
    System.out.println("=============================");
  }

  /**
   * Get the saved custom position
   */
  public static Pose2d getCustomPosition() {
    return customPosition;
  }

  /**
   * Check if a custom position has been saved
   */
  public static boolean hasCustomPosition() {
    return hasCustomPosition;
  }

  /**
   * Clear the custom position
   */
  public static void clearCustomPosition() {
    customPosition = null;
    hasCustomPosition = false;
    Logger.recordOutput("SavedPositions/HasCustom", false);
    System.out.println("Custom position cleared");
  }
}
