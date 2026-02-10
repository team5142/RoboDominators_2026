package frc.robot.subsystems.turret;

// Final turret outputs after control logic
public class TurretOutput {
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
