package frc.robot.subsystems.turret;

// Snapshot of turret sensors and encoder data
public class TurretIOInputs {
  // Hood bottom limit switch — true when hood is at the 85 deg home position
  public boolean hoodLimitSwitchRaw = false;
  public boolean hallLeftRaw = false;
  public boolean hallRightRaw = false;

  // Hood position from motor encoder — 0 at home (85 deg), positive = up toward 35 deg.
  // TODO: replace with CANcoder absolute position once hardware is available.
  public double hoodMotorPositionRotations = 0.0;
  public double turretAbsolutePositionRotations = 0.0;
  public double flywheelVelocityRpm = 0.0; // front flywheel velocity in RPM
  public double turretMotorCurrentAmps = 0.0; // stator current — used for homing stall detection
}
