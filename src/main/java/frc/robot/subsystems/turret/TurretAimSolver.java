package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.util.SmartLogger;

// Converts robot pose + target pose into a turret aim goal, gated by the current TurretPhase.
//
// PHASE_1: solver is a no-op; TurretSubsystem uses fixed/manual setpoints only.
// PHASE_2: computes bearing from robot to target; requires near-zero chassis speed to enable goal.
// PHASE_3: same bearing math; removes the stationary requirement; fire gated by chassis speed.
// PHASE_4: lead compensation — offsets target by robot velocity * TOF so ball lands on hub while moving.
public class TurretAimSolver {

  // When the robot is stationary, latch the heading, turret target, and distance so QuestNav
  // position/heading drift doesn't cause the turret or hood to oscillate while holding a shot.
  // Values update freely while moving, then freeze once speed drops below threshold.
  private double latchedTurretRotations = 0.0;
  private double latchedDistance        = 0.0;
  private boolean latchInitialized = false;

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

    boolean stationary = inputs.robotSpeedMetersPerSecond
        < Constants.Turret.CHASSIS_SPEED_FIRE_THRESHOLD_MPS;
    boolean effectivelyStopped = inputs.robotSpeedMetersPerSecond
        < Constants.Turret.CHASSIS_SPEED_LATCH_THRESHOLD_MPS;

    // Compute raw heading and bearing from current pose
    double rawHeadingRad = inputs.robotPose.getRotation().getRadians();
    double cosH = Math.cos(rawHeadingRad);
    double sinH = Math.sin(rawHeadingRad);
    double pivotFieldX = inputs.robotPose.getX()
        + cosH * Constants.Turret.TURRET_PIVOT_OFFSET_X_METERS
        - sinH * Constants.Turret.TURRET_PIVOT_OFFSET_Y_METERS;
    double pivotFieldY = inputs.robotPose.getY()
        + sinH * Constants.Turret.TURRET_PIVOT_OFFSET_X_METERS
        + cosH * Constants.Turret.TURRET_PIVOT_OFFSET_Y_METERS;

    double dx = inputs.targetPose.getX() - pivotFieldX;
    double dy = inputs.targetPose.getY() - pivotFieldY;
    double distance = Math.hypot(dx, dy);

    // Phase 4: offset the target by robot velocity * TOF so the ball meets the hub while moving.
    // Uses a first-pass distance to pick TOF, then recomputes bearing to the lead-adjusted point.
    if (phase == Constants.Turret.TurretPhase.PHASE_4_ON_THE_MOVE) {
      TurretShotProfile firstPass = TurretShotProfile.getForDistance(distance);
      double tof = firstPass.timeOfFlightSeconds;
      double leadX = inputs.robotFieldVxMetersPerSecond * tof;
      double leadY = inputs.robotFieldVyMetersPerSecond * tof;
      // Shift pivot toward the lead point (robot moves, so pivot will be at pivot+lead when ball arrives)
      dx = inputs.targetPose.getX() - (pivotFieldX + leadX);
      dy = inputs.targetPose.getY() - (pivotFieldY + leadY);
      distance = Math.hypot(dx, dy);
      SmartLogger.logReplay("Turret/AimSolver/LeadOffsetM", Math.hypot(leadX, leadY));
    }

    double targetBearingRad = Math.atan2(dy, dx);
    double turretRelativeRad = Rotation2d.fromRadians(-(targetBearingRad - rawHeadingRad)).getRadians();
    double rawTurretRotations = turretRelativeRad / (2.0 * Math.PI);

    double motorTargetRaw = rawTurretRotations * Constants.Turret.TURRET_GEAR_RATIO
        + Constants.Turret.TURRET_FORWARD_MOTOR_ROT;
    boolean rawReachable = motorTargetRaw >= Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT
        && motorTargetRaw <= Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT;

    // Freeze the latch when effectively stopped AND target is reachable — suppresses QuestNav drift.
    // Uses a tighter threshold (0.1 m/s) than the fire gate so the latch updates freely while moving.
    // If target is in the deadzone, always update so the latch escapes as soon as robot rotates out.
    boolean shouldFreeze = effectivelyStopped && rawReachable;
    if (!latchInitialized || !shouldFreeze) {
      latchedTurretRotations = rawTurretRotations;
      latchedDistance        = distance;
      latchInitialized       = true;
    }

    SmartLogger.logReplay("Turret/AimSolver/HeadingDeg",             Math.toDegrees(inputs.robotPose.getRotation().getRadians())); // filtered
    SmartLogger.logReplay("Turret/AimSolver/HeadingRawDeg",          Math.toDegrees(rawHeadingRad)); // raw — for comparison
    SmartLogger.logReplay("Turret/AimSolver/TurretTargetRot",        rawTurretRotations);
    SmartLogger.logReplay("Turret/AimSolver/TurretTargetRotLatched",  latchedTurretRotations);
    SmartLogger.logReplay("Turret/AimSolver/DistanceM",              distance);
    SmartLogger.logReplay("Turret/AimSolver/DistanceLatched",        latchedDistance);

    double motorTarget = latchedTurretRotations * Constants.Turret.TURRET_GEAR_RATIO
        + Constants.Turret.TURRET_FORWARD_MOTOR_ROT;
    goal.targetReachable = motorTarget >= Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT
        && motorTarget <= Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT;
    SmartLogger.logReplay("Turret/AimSolver/TargetReachable", goal.targetReachable);
    SmartLogger.logReplay("Turret/AimSolver/MotorTarget", motorTarget);

    TurretShotProfile shot = TurretShotProfile.getForDistance(latchedDistance);

    goal.turretRotations  = latchedTurretRotations;
    goal.hoodRotations    = shot.hoodRotations;
    goal.useRps           = true;
    goal.flywheelFrontRps = shot.flywheelFrontRps;
    goal.flywheelBackRps  = shot.flywheelBackRps;
    goal.flywheelPercent  = 0.0; // unused when useRps=true

    // Phase 2: only enable goal when robot is effectively stationary
    if (phase == Constants.Turret.TurretPhase.PHASE_2_TRACKING) {
      goal.enable = stationary && goal.targetReachable;
      if (!stationary) {
        SmartLogger.logReplay("Turret/AimSolver/WhyNotReady", "chassis moving (phase 2)");
      } else if (!goal.targetReachable) {
        SmartLogger.logReplay("Turret/AimSolver/WhyNotReady", "target in deadzone");
      }
      return;
    }

    // Phase 3 and 4: disable goal if target is in the deadzone — prevents turret slamming to soft limit.
    if (!goal.targetReachable) {
      goal.enable = false;
      SmartLogger.logReplay("Turret/AimSolver/WhyNotReady", "target in deadzone");
      return;
    }

    // Phase 3: fire gate is chassis speed < threshold (slow shoot-on-the-move).
    // Phase 4: no speed gate — lead comp handles the offset, fire any time.
    if (phase == Constants.Turret.TurretPhase.PHASE_3_DECEL_SHOOT && !stationary) {
      goal.enable = true; // turret still tracks, isReadyToShoot() applies the speed gate
    } else {
      goal.enable = true;
    }

    SmartLogger.logReplay("Turret/AimSolver/TargetDistanceM", latchedDistance);
    SmartLogger.logReplay("Turret/AimSolver/TargetBearingDeg", Math.toDegrees(targetBearingRad));
    SmartLogger.logReplay("Turret/AimSolver/FlywheelFrontRps", shot.flywheelFrontRps);
    SmartLogger.logReplay("Turret/AimSolver/FlywheelBackRps",  shot.flywheelBackRps);
    SmartLogger.logReplay("Turret/AimSolver/HoodRot", shot.hoodRotations);
  }
}
