package frc.robot.subsystems.turret;

// Snapshot of turret sensors and encoder data
public class TurretIOInputs {
  // Hood bottom limit switch — true when hood is at the 85 deg home position
  public boolean hoodLimitSwitchRaw = false;
  public boolean hallCCWRaw = false;

  // Hood position from motor encoder — 0 at home (85 deg), positive = up toward 35 deg.
  // TODO: replace with CANcoder absolute position once hardware is available.
  public double hoodMotorPositionRotations = 0.0;
  public double turretAbsolutePositionRotations = 0.0;
  public double turretVelocityRps = 0.0; // motor rot/sec — watch vs MotionMagic setpoint velocity in AScope
  public double flywheelVelocityRpm = 0.0;     // front flywheel velocity in RPM
  public double flywheelBackVelocityRpm = 0.0; // back flywheel velocity in RPM
  public double turretMotorCurrentAmps = 0.0; // stator current — used for homing stall detection
  // True for one loop when the turret motor rebooted while the robot was enabled (brownout/power loss)
  public boolean turretBootDuringEn = false;
}
