package frc.robot.subsystems.turret;

import frc.robot.Constants;

// Converts aim goals into motor setpoints.
// Turret rotation uses MotionMagic position control — goal is in turret rotations,
// converted to motor rotations via gear ratio before being sent to the motor.
public class TurretSetpointGenerator {
  private static final double HOOD_KP = 0.15; // hood is still open-loop percent

  public void update(TurretState state, TurretAimGoal goal, TurretSetpoints setpoints) {
    if (!goal.enable) {
      return;
    }

    // goal.turretRotations is robot-relative: 0 = forward, +0.25 = left, -0.25 = right.
    // Add TURRET_FORWARD_MOTOR_ROT to translate into the encoder frame (0 = CCW hard stop).
    double motorRot = goal.turretRotations * Constants.Turret.TURRET_GEAR_RATIO
        + Constants.Turret.TURRET_FORWARD_MOTOR_ROT;
    // Clamp to soft limits so the tracking path can never command into the hard stops.
    motorRot = Math.max(Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT,
                Math.min(motorRot, Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT));
    setpoints.turretPositionMotorRotations = motorRot;
    setpoints.useTurretPosition = true;

    setpoints.hoodPercent     = clamp((goal.hoodRotations - state.hoodMotorPositionRotations) * HOOD_KP);
    setpoints.flywheelPercent = clamp(goal.flywheelPercent);
  }

  private double clamp(double value) {
    return Math.max(-1.0, Math.min(1.0, value));
  }
}
