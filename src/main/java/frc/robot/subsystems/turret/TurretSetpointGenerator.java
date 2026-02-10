package frc.robot.subsystems.turret;

// Converts aim goals into open loop setpoints
public class TurretSetpointGenerator {
  public void update(TurretState state, TurretAimGoal goal, TurretSetpoints setpoints) {
    if (!goal.enable) {
      return;
    }

    setpoints.turretPercent = clamp(goal.turretRotations - state.turretAbsolutePositionRotations);
    setpoints.hoodPercent = clamp(goal.hoodRotations - state.hoodAbsolutePositionRotations);
    setpoints.flywheelPercent = clamp(goal.flywheelPercent);
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
