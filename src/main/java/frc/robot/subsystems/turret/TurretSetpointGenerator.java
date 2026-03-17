package frc.robot.subsystems.turret;

import frc.robot.Constants;

// Converts an aim goal into turret/hood motor setpoints.
// Turret and hood both use MotionMagic position control.
// Flywheel RPS is applied separately by the operator — not set here.
public class TurretSetpointGenerator {

  public void update(TurretAimGoal goal, TurretSetpoints setpoints) {
    if (!goal.enable) {
      // No active goal — release position control so brake mode holds the current position.
      setpoints.useTurretPosition = false;
      setpoints.turretPercent = 0.0;
      return;
    }

    // Convert robot-relative turret rotations (0 = forward) to motor encoder frame (0 = CCW hard stop).
    double motorRot = goal.turretRotations * Constants.Turret.TURRET_GEAR_RATIO
        + Constants.Turret.TURRET_FORWARD_MOTOR_ROT;
    motorRot = Math.max(Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT,
                Math.min(motorRot, Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT));
    setpoints.turretPositionMotorRotations = motorRot;
    setpoints.useTurretPosition            = true;

    setpoints.useHoodPosition            = true;
    setpoints.hoodPositionMotorRotations = goal.hoodRotations;
  }
}
