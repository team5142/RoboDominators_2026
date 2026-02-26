package frc.robot.subsystems.turret;

// Hardware abstraction for turret mechanism IO
public interface TurretIO {
  default void updateInputs(TurretIOInputs inputs) {}

  default void setFlywheelPercent(double percent) {}

  default void setHoodPercent(double percent) {}

  default void setTurretPercent(double percent) {}

  // Zeros the turret encoder position — called when homing completes at the left hall sensor
  default void zeroTurretEncoder() {}
}
