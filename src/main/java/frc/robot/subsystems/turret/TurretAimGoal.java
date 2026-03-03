package frc.robot.subsystems.turret;

// Desired turret aim goal in field-relative rotations
public class TurretAimGoal {
  public double turretRotations = 0.0;
  public double hoodRotations = 0.0;
  public double flywheelPercent = 0.0;
  public boolean enable = false;
  // True when the target bearing falls within the turret's physical travel range.
  // False = target is in the blind spot; robot needs to rotate to bring it in range.
  public boolean targetReachable = true;

  public void clear() {
    turretRotations = 0.0;
    hoodRotations = 0.0;
    flywheelPercent = 0.0;
    enable = false;
    targetReachable = true;
  }
}
