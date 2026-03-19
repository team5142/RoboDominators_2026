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

    // Phase 1: turret is locked forward manually — don't command MotionMagic position so D-pad
    // jogging isn't immediately overwritten. Brake mode holds position between D-pad inputs.
    // Hood and flywheel still update from the aim goal unless a manual hood override is active.
    if (Constants.Turret.CURRENT_PHASE == Constants.Turret.TurretPhase.PHASE_1_STATIC) {
      setpoints.useTurretPosition      = false;
      // Only update hood from aim goal if no manual override is active.
      // When the operator steps the hood via D-pad, manualPositionOverride stays true
      // until tracking is explicitly re-enabled, so this block is skipped.
      if (!setpoints.manualHoodOverride) {
        setpoints.useHoodPosition            = true;
        setpoints.hoodPositionMotorRotations = goal.hoodRotations;
      }
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
