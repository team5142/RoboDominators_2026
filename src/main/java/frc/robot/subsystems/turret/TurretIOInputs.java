package frc.robot.subsystems.turret;

// Snapshot of turret sensors and encoder data
public class TurretIOInputs {
  public boolean hoodBeamBreakRaw = false;
  public boolean hallLeftRaw = false;
  public boolean hallRightRaw = false;

  public double hoodAbsolutePositionRotations = 0.0;
  public double turretAbsolutePositionRotations = 0.0;
  public double flywheelVelocityRpm = 0.0; // front flywheel velocity in RPM
}
