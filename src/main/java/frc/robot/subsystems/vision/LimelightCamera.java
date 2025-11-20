package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import java.util.List;

public class LimelightCamera implements VisionCamera {

  private final String name;
  private final NetworkTable table;
  private boolean connectionWarningShown = false; // Prevent spam

  public LimelightCamera(String name, int pipeline) {
    this.name = name;
    this.table = NetworkTableInstance.getDefault().getTable(name);
    table.getEntry("pipeline").setNumber(pipeline);
  }

  @Override
  public Optional<VisionResult> getLatestResult() {
    try {
      double tv = table.getEntry("tv").getDouble(0);
      if (tv == 0) {
        connectionWarningShown = false; // Reset warning when camera recovers
        return Optional.empty();
      }

      // Get alliance-specific botpose
      boolean isBlue = DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Blue)
          .orElse(true); // Default to blue for testing
      
      String botposeKey = isBlue ? "botpose_wpiblue" : "botpose_wpired";
      double[] botpose = table.getEntry(botposeKey).getDoubleArray(new double[7]);
      
      // Fallback to generic botpose if alliance-specific not available
      if (botpose.length < 7) {
        botpose = table.getEntry("botpose").getDoubleArray(new double[7]);
      }
      
      if (botpose.length < 7) {
        return Optional.empty();
      }

      // Debug logging
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/Alliance", isBlue ? "Blue" : "Red");
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/BotposeKey", botposeKey);
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/RawBotpose", botpose);

      Pose2d pose = new Pose2d(
          botpose[0], 
          botpose[1], 
          new edu.wpi.first.math.geometry.Rotation2d(Math.toRadians(botpose[5])));
      
      // SAFETY: Reject zero poses (0,0,0) - these are invalid
      if (Math.abs(pose.getX()) < 0.01 && Math.abs(pose.getY()) < 0.01) {
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/RejectedZeroPose", true);
        return Optional.empty();
      }
      
      double timestamp = botpose[6];
      int tagCount = (int) table.getEntry("tid").getDouble(0) > 0 ? 1 : 0;
      double distance = Math.hypot(botpose[0], botpose[1]);
      
      // Get tag ID (Limelight only reports one tag at a time)
      int tagId = (int) table.getEntry("tid").getDouble(-1);
      List<Integer> tagIds = tagId > 0 ? java.util.List.of(tagId) : java.util.List.of();

      connectionWarningShown = false; // Reset warning
      
      return Optional.of(new VisionResult(pose, timestamp, tagCount, distance, tagIds)); // NEW: Pass tag IDs
      
    } catch (Exception e) {
      // Limelight disconnected or NetworkTables error
      if (!connectionWarningShown) {
        System.err.println("[WARNING] Limelight '" + name + "' error: " + e.getMessage());
        connectionWarningShown = true;
      }
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/ConnectionError", e.getMessage());
      return Optional.empty();
    }
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public boolean hasTarget() {
    try {
      return table.getEntry("tv").getDouble(0) > 0;
    } catch (Exception e) {
      return false;
    }
  }
}
