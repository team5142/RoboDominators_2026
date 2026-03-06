package frc.robot.subsystems.turret;

// Routes setpoints to outputs. Turret uses MotionMagic position when useTurretPosition is set,
// otherwise falls back to open-loop percent (used during homing).
public class TurretController {
  public void update(TurretState state, TurretSetpoints setpoints, TurretOutput outputs) {
    outputs.flywheelPercent          = clamp(setpoints.flywheelPercent);
    outputs.flywheelFrontPercent     = clamp(setpoints.flywheelFrontPercent);
    outputs.flywheelBackPercent      = clamp(setpoints.flywheelBackPercent);
    outputs.useIndependentFlywheel   = setpoints.useIndependentFlywheel;
    outputs.hoodPercent              = clamp(setpoints.hoodPercent);
    outputs.useHoodPosition          = setpoints.useHoodPosition;
    outputs.hoodPositionMotorRotations = setpoints.hoodPositionMotorRotations;
    outputs.turretPercent            = clamp(setpoints.turretPercent);
    outputs.useTurretPosition        = setpoints.useTurretPosition;
    outputs.turretPositionMotorRotations = setpoints.turretPositionMotorRotations;
  }

  private double clamp(double value) {
    return Math.max(-1.0, Math.min(1.0, value));
  }
}
