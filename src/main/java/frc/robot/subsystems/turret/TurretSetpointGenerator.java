package frc.robot.subsystems.turret;

import frc.robot.Constants;

// Converts aim goals into motor setpoints.
// Turret rotation uses MotionMagic position control — goal is in turret rotations,
// converted to motor rotations via gear ratio before being sent to the motor.
// Hood uses MotionMagic position control. Flywheel uses independent front/back RPS when useRps=true.
public class TurretSetpointGenerator {

  public void update(TurretState state, TurretAimGoal goal, TurretSetpoints setpoints) {
    if (!goal.enable) {
      // No active goal — drop to open-loop zero so turret holds its current position via brake mode.
      // Do not leave a stale position setpoint active or MotionMagic will keep driving toward it.
      setpoints.useTurretPosition = false;
      setpoints.turretPercent = 0.0;
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

    setpoints.useHoodPosition              = true;
    setpoints.hoodPositionMotorRotations   = goal.hoodRotations;

    // Flywheels are not set here — the operator controls them independently via setFlywheelFrontRps/BackRps.
    // The aim goal carries the correct RPS values for the current distance, but applying them
    // is gated by the operator (right bumper) so they never spin without a deliberate command.
  }
}
