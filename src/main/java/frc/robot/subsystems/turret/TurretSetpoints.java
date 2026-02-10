package frc.robot.subsystems.turret;

// Desired turret outputs in open loop percent
public class TurretSetpoints {
  public double flywheelPercent = 0.0;
  public double hoodPercent = 0.0;
  public double turretPercent = 0.0;
  public double singulatorPercent = 0.0;

  public void clear() {
    flywheelPercent = 0.0;
    hoodPercent = 0.0;
    turretPercent = 0.0;
    singulatorPercent = 0.0;
  }
}
