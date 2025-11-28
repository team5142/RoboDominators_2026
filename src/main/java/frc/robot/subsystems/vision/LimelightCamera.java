package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import java.util.List;

// Limelight camera wrapper - reads pose from NetworkTables (Limelight 3's native protocol)
// Limelight does on-camera processing and publishes results to NT, we just read them
// Automatically switches between blue/red alliance botpose for correct field orientation
public class LimelightCamera implements VisionCamera {

  private final String name; // Camera hostname (e.g. "limelight-front")
  private final NetworkTable table; // NT table where Limelight publishes data
  private boolean connectionWarningShown = false; // Throttle error messages

  public LimelightCamera(String name, int pipeline) {
    this.name = name;
    this.table = NetworkTableInstance.getDefault().getTable(name);
    table.getEntry("pipeline").setNumber(pipeline); // Set AprilTag pipeline
  }

  @Override
  public Optional<VisionResult> getLatestResult() {
    try {
      // Check if Limelight sees any targets (tv = "target valid")
      double tv = table.getEntry("tv").getDouble(0);
      if (tv == 0) {
        connectionWarningShown = false; // Reset warning when camera recovers
        return Optional.empty();
      }

      // Get alliance-specific botpose (field-relative position)
      // Limelight provides separate poses for blue/red so we don't have to flip
      boolean isBlue = DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Blue)
          .orElse(true); // Default to blue if FMS not connected
      
      String botposeKey = isBlue ? "botpose_wpiblue" : "botpose_wpired";
      double[] botpose = table.getEntry(botposeKey).getDoubleArray(new double[7]);
      
      // Fallback to generic botpose if alliance-specific not available (LL2/LL2+)
      if (botpose.length < 7) {
        botpose = table.getEntry("botpose").getDoubleArray(new double[7]);
      }
      
      if (botpose.length < 7) {
        return Optional.empty(); // Invalid data
      }

      // Debug logging for alliance-specific pose selection
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/Alliance", isBlue ? "Blue" : "Red");
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/BotposeKey", botposeKey);
      org.littletonrobotics.junction.Logger.recordOutput(
          "Vision/" + name + "/RawBotpose", botpose);

      // Extract pose from Limelight array [x, y, z, roll, pitch, yaw, latency]
      Pose2d pose = new Pose2d(
          botpose[0], // X position (meters)
          botpose[1], // Y position (meters)
          new edu.wpi.first.math.geometry.Rotation2d(Math.toRadians(botpose[5]))); // Yaw (convert deg to rad)
      
      // SAFETY: Reject (0,0,0) poses - these are invalid/uninitialized
      if (Math.abs(pose.getX()) < 0.01 && Math.abs(pose.getY()) < 0.01) {
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/" + name + "/RejectedZeroPose", true);
        return Optional.empty();
      }
      
      double timestamp = botpose[6]; // Latency in milliseconds (convert to seconds)
      int tagCount = (int) table.getEntry("tid").getDouble(0) > 0 ? 1 : 0; // Limelight only reports 1 tag at a time
      double distance = Math.hypot(botpose[0], botpose[1]); // Rough distance estimate
      
      // Get tag ID - Limelight reports one primary tag (tid = "target ID")
      int tagId = (int) table.getEntry("tid").getDouble(-1);
      List<Integer> tagIds = tagId > 0 ? java.util.List.of(tagId) : java.util.List.of();

      connectionWarningShown = false; // Reset warning
      
      return Optional.of(new VisionResult(pose, timestamp, tagCount, distance, tagIds));
      
    } catch (Exception e) {
      // Limelight disconnected or NetworkTables error (network drop, camera reboot, etc.)
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
      return table.getEntry("tv").getDouble(0) > 0; // tv = 1 means target visible
    } catch (Exception e) {
      return false;
    }
  }
}
