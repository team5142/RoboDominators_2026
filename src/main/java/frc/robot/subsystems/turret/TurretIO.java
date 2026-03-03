package frc.robot.subsystems.turret;

// Hardware abstraction for turret mechanism IO
public interface TurretIO {
  default void updateInputs(TurretIOInputs inputs) {}

  default void setFlywheelPercent(double percent) {}

  default void setHoodPercent(double percent) {}

  default void setTurretPercent(double percent) {}

  // Direct voltage control — used only during SysId characterization
  default void setTurretVoltage(double volts) {}

  // MotionMagic position control — motor rotations from home (0 = CCW stop)
  default void setTurretPosition(double motorRotations) {}

  // Raises stator current limit for SysId (true = SysId active, false = restore normal limit)
  default void setSysIdActive(boolean active) {}

  // Sets encoder to the hall sensor offset so 0 = CCW hard stop.
  // Called when homing completes at the hall sensor.
  default void zeroTurretEncoder() {}

  // Zeros the hood motor encoder — called when homing completes at the bottom limit switch
  default void zeroHoodEncoder() {}
}
