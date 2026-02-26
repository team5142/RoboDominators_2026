package frc.robot.subsystems.turret;

// Converts aim goals into open loop setpoints
// TODO (robot session): tune TURRET_KP and HOOD_KP - current values of 1.0 will likely overshoot.
// Lower values (e.g. 0.3) give slower but safer response for first spin.
public class TurretSetpointGenerator {
  // Scale factor: rotations of error * kP = percent output. Tune on robot.
  // Very conservative first-spin values - increase once direction + limits confirmed.
  private static final double TURRET_KP = 0.15;
  private static final double HOOD_KP   = 0.15;

  public void update(TurretState state, TurretAimGoal goal, TurretSetpoints setpoints) {
    if (!goal.enable) {
      return;
    }

    setpoints.turretPercent = clamp((goal.turretRotations - state.turretAbsolutePositionRotations) * TURRET_KP);
    setpoints.hoodPercent   = clamp((goal.hoodRotations   - state.hoodAbsolutePositionRotations)   * HOOD_KP);
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
