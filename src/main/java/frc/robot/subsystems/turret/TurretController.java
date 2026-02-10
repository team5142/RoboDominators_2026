package frc.robot.subsystems.turret;

// Simple controller stub for turret outputs
public class TurretController {
  public void update(TurretState state, TurretSetpoints setpoints, TurretOutput outputs) {
    outputs.flywheelPercent = clamp(setpoints.flywheelPercent);
    outputs.hoodPercent = clamp(setpoints.hoodPercent);
    outputs.turretPercent = clamp(setpoints.turretPercent);
    outputs.singulatorPercent = clamp(setpoints.singulatorPercent);
  }

  private double clamp(double value) {
    if (value > 1.0) {
      return 1.0;
    }
    if (value < -1.0) {
      return -1.0;
    }
    return value;
  }
}
