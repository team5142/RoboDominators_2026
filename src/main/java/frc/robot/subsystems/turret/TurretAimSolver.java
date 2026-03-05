package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.util.SmartLogger;

// Converts robot pose + target pose into a turret aim goal, gated by the current TurretPhase.
//
// PHASE_1: solver is a no-op; TurretSubsystem uses fixed/manual setpoints only.
// PHASE_2: computes bearing from robot to target; requires near-zero chassis speed to enable goal.
// PHASE_3: same bearing math; removes the stationary requirement; fire gated by chassis speed.
// PHASE_4: stub — falls through to PHASE_3 behavior (velocity compensation not yet implemented).
public class TurretAimSolver {

  public void solve(TurretAimInputs inputs, TurretAimGoal goal) {
    Constants.Turret.TurretPhase phase = Constants.Turret.CURRENT_PHASE;

    // Phase 1: no automatic aim — caller sets goal directly via setAimGoal() or open loop
    if (phase == Constants.Turret.TurretPhase.PHASE_1_STATIC) {
      goal.enable = false;
      return;
    }

    // All phases above 1 require a valid pose
    if (inputs.robotPose == null || inputs.targetPose == null) {
      goal.enable = false;
      SmartLogger.logReplay("Turret/AimSolver/WhyNotReady", "null pose");
      return;
    }

    // Compute turret pivot position in field frame by rotating the robot-relative offset
    // by the robot's current heading and adding it to the robot center pose.
    double headingRad = inputs.robotPose.getRotation().getRadians();
    double cosH = Math.cos(headingRad);
    double sinH = Math.sin(headingRad);
    double pivotFieldX = inputs.robotPose.getX()
        + cosH * Constants.Turret.TURRET_PIVOT_OFFSET_X_METERS
        - sinH * Constants.Turret.TURRET_PIVOT_OFFSET_Y_METERS;
    double pivotFieldY = inputs.robotPose.getY()
        + sinH * Constants.Turret.TURRET_PIVOT_OFFSET_X_METERS
        + cosH * Constants.Turret.TURRET_PIVOT_OFFSET_Y_METERS;

    // Bearing and distance from turret pivot (not robot center) to target
    double dx = inputs.targetPose.getX() - pivotFieldX;
    double dy = inputs.targetPose.getY() - pivotFieldY;
    double targetBearingRad = Math.atan2(dy, dx);

    // Convert field-relative bearing to turret-relative rotation.
    // Negate because WPILib heading is CCW-positive but the turret encoder is CW-positive.
    double turretRelativeRad = -(targetBearingRad - headingRad);
    // Normalize to [-pi, pi]
    turretRelativeRad = Rotation2d.fromRadians(turretRelativeRad).getRadians();

    // Convert radians to rotations for the CANcoder/TalonFX position target
    double turretTargetRotations = turretRelativeRad / (2.0 * Math.PI);

    // Check if the target falls within the physical travel range before applying gear ratio offset.
    // Motor position = turretTargetRotations * GEAR_RATIO + FORWARD_OFFSET.
    double motorTarget = turretTargetRotations * Constants.Turret.TURRET_GEAR_RATIO
        + Constants.Turret.TURRET_FORWARD_MOTOR_ROT;
    goal.targetReachable = motorTarget >= Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT
        && motorTarget <= Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT;
    SmartLogger.logReplay("Turret/AimSolver/TargetReachable", goal.targetReachable);
    SmartLogger.logReplay("Turret/AimSolver/MotorTarget", motorTarget);

    // Compute distance for shot table lookup
    double distance = Math.hypot(dx, dy);
    TurretShotProfile shot = TurretShotProfile.getForDistance(distance);

    goal.turretRotations = turretTargetRotations;
    goal.hoodRotations   = shot.hoodRotations;
    goal.flywheelPercent = shot.flywheelPercent;

    // Phase 2: only enable goal when robot is effectively stationary
    if (phase == Constants.Turret.TurretPhase.PHASE_2_TRACKING) {
      boolean stationary = inputs.robotSpeedMetersPerSecond
          < Constants.Turret.CHASSIS_SPEED_FIRE_THRESHOLD_MPS;
      goal.enable = stationary;
      if (!stationary) {
        SmartLogger.logReplay("Turret/AimSolver/WhyNotReady", "chassis moving (phase 2)");
      }
      return;
    }

    // Phase 3 and 4: tracking runs continuously; fire gate is handled by isReadyToShoot()
    goal.enable = true;

    SmartLogger.logReplay("Turret/AimSolver/TargetDistanceM", distance);
    SmartLogger.logReplay("Turret/AimSolver/TargetBearingDeg", Math.toDegrees(targetBearingRad));
    SmartLogger.logReplay("Turret/AimSolver/TurretTargetRot", turretTargetRotations);
    SmartLogger.logReplay("Turret/AimSolver/FlywheelPct", shot.flywheelPercent);
    SmartLogger.logReplay("Turret/AimSolver/HoodRot", shot.hoodRotations);
  }
}
