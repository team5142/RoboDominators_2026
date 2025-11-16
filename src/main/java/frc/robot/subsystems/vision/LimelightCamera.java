
package frc.robot.subsystems.vision;



import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;



public class LimelightCamera implements VisionCamera {

  private final String name;

  private final NetworkTable table;



  public LimelightCamera(String name, int pipeline) {

    this.name = name;

    this.table = NetworkTableInstance.getDefault().getTable(name);

    table.getEntry("pipeline").setNumber(pipeline);

  }



  @Override

  public Optional<VisionResult> getLatestResult() {

    double tv = table.getEntry("tv").getDouble(0);

    if (tv == 0) {

      return Optional.empty();

    }



    // Get botpose from Limelight

    double[] botpose = table.getEntry("botpose").getDoubleArray(new double[7]);

    if (botpose.length < 7) {

      return Optional.empty();

    }



    Pose2d pose = new Pose2d(botpose[0], botpose[1], new edu.wpi.first.math.geometry.Rotation2d(Math.toRadians(botpose[5])));

    double timestamp = botpose[6];

    int tagCount = (int) table.getEntry("tid").getDouble(0) > 0 ? 1 : 0;

    double distance = Math.hypot(botpose[0], botpose[1]);



    return Optional.of(new VisionResult(pose, timestamp, tagCount, distance));

  }



  @Override

  public String getName() {

    return name;

  }



  @Override

  public boolean hasTarget() {

    return table.getEntry("tv").getDouble(0) > 0;

  }

}
