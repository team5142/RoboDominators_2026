package frc.robot.subsystems.turret;

// Desired turret aim goal in field-relative rotations
public class TurretAimGoal {
  public double turretRotations = 0.0;
  public double hoodRotations = 0.0;
  public double flywheelPercent = 0.0;
  public boolean enable = false;

  public void clear() {
    turretRotations = 0.0;
    hoodRotations = 0.0;
    flywheelPercent = 0.0;
    enable = false;
  }
}
