package frc.robot.subsystems.turret;

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
  private boolean latchInitialized      = false;
  private boolean wasInDeadzone         = false; // true last cycle — forces one free latch update on re-entry

  // Call whenever the pipeline restarts (enable transition, new auto) so stale latched values
  // from a previous run don't feed into the first solve() call.
  public void resetLatch() {
    latchInitialized = false;
    wasInDeadzone    = false;
  }

  public void solve(TurretAimInputs inputs, TurretAimGoal goal) {
    Constants.Turret.TurretPhase phase = Constants.Turret.CURRENT_PHASE;

    // Phase 1: turret locked forward (0 rot), but hood+flywheel still auto-adjust by distance.
    // QuestNav still runs for odometry — we just don't rotate the turret.
    if (phase == Constants.Turret.TurretPhase.PHASE_1_STATIC) {
      if (inputs.robotPose == null || inputs.targetPose == null) {
        goal.enable = false;
        return;
      }
      double dist1 = pivotToTargetDistance(inputs);
      TurretShotProfile shot1 = TurretShotProfile.getForDistance(dist1);
      goal.turretRotations  = 0.0;
      goal.hoodRotations    = shot1.hoodRotations;
      goal.flywheelFrontRps = shot1.flywheelFrontRps;
      goal.flywheelBackRps  = shot1.flywheelBackRps;
      goal.targetReachable  = true;
      goal.chassisSpeedMps  = inputs.robotSpeedMetersPerSecond;
      goal.enable           = true;
      SmartLogger.logReplay("Turret/AimSolver/DistanceM", dist1);
      SmartLogger.logReplay("Turret/AimSolver/HoodRot", shot1.hoodRotations);
      SmartLogger.logReplay("Turret/AimSolver/FlywheelFrontRps", shot1.flywheelFrontRps);
      SmartLogger.logReplay("Turret/AimSolver/FlywheelBackRps",  shot1.flywheelBackRps);
      return;
    }

    // All phases above 1 require a valid pose
    if (inputs.robotPose == null || inputs.targetPose == null) {
      goal.enable = false;
      SmartLogger.logReplay("Turret/AimSolver/WhyNotReady", "null pose");
      return;
    }

    // If the pose was just reseeded (new alliance, new auto start), drop the stale latch
    // so the first solve after seeding uses the fresh bearing, not the previous frozen value.
    if (inputs.poseReseeded) {
      resetLatch();
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
    // Normalize the turret-relative angle to [-π, π] so rawTurretRotations stays in [-0.5, +0.5].
    // Rotation2d.fromRadians().getRadians() does NOT normalize — it returns the raw value,
    // which exceeds ±π when robot heading is ~180° (Red alliance), producing bogus latch values.
    double relativeRad = -(targetBearingRad - rawHeadingRad);
    double turretRelativeRad = Math.atan2(Math.sin(relativeRad), Math.cos(relativeRad));
    double rawTurretRotations = turretRelativeRad / (2.0 * Math.PI);

    double motorTargetRaw = rawTurretRotations * Constants.Turret.TURRET_GEAR_RATIO
        + Constants.Turret.TURRET_FORWARD_MOTOR_ROT;
    boolean rawReachable = motorTargetRaw >= Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT
        && motorTargetRaw <= Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT;

    // Freeze the latch when fully stopped (translation AND rotation) AND target is reachable.
    // If we just exited the deadzone, force one free update so the latch picks up the live
    // bearing before it can freeze — prevents locking onto the stale wrap-edge position.
    boolean justExitedDeadzone = wasInDeadzone && rawReachable;
    wasInDeadzone = !rawReachable;
    boolean fullyStationary = effectivelyStopped
        && inputs.robotOmegaRadPerSecond < Constants.Turret.CHASSIS_OMEGA_LATCH_THRESHOLD_RPS;
    boolean shouldFreeze = fullyStationary && rawReachable && !justExitedDeadzone;
    if (!latchInitialized || !shouldFreeze) {
      latchedTurretRotations = rawTurretRotations;
      latchedDistance        = distance;
      latchInitialized       = true;
    }

    SmartLogger.logReplay("Turret/AimSolver/HeadingDeg",             Math.toDegrees(inputs.robotPose.getRotation().getRadians())); // filtered
    SmartLogger.logReplay("Turret/AimSolver/HeadingRawDeg",          Math.toDegrees(rawHeadingRad)); // raw — for comparison
    SmartLogger.logReplay("Turret/AimSolver/OmegaRadPerSec",         inputs.robotOmegaRadPerSecond);
    SmartLogger.logReplay("Turret/AimSolver/TurretTargetRot",        rawTurretRotations);
    SmartLogger.logReplay("Turret/AimSolver/TurretTargetRotLatched",  latchedTurretRotations);
    SmartLogger.logReplay("Turret/AimSolver/DistanceM",              distance);
    SmartLogger.logReplay("Turret/AimSolver/DistanceLatched",        latchedDistance);

    // Through-360 diagnostics — watch these together in AScope to diagnose wrap/re-acquire issues
    SmartLogger.logReplay("Turret/Through360/RawTurretRot",          rawTurretRotations);
    SmartLogger.logReplay("Turret/Through360/LatchedTurretRot",      latchedTurretRotations);
    SmartLogger.logReplay("Turret/Through360/RawReachable",          rawReachable);
    SmartLogger.logReplay("Turret/Through360/GoalTargetReachable",   goal.targetReachable);
    SmartLogger.logReplay("Turret/Through360/WasInDeadzone",         wasInDeadzone);
    SmartLogger.logReplay("Turret/Through360/JustExitedDeadzone",    justExitedDeadzone);
    SmartLogger.logReplay("Turret/Through360/FullyStationary",       fullyStationary);
    SmartLogger.logReplay("Turret/Through360/ShouldFreeze",          shouldFreeze);
    SmartLogger.logReplay("Turret/Through360/MotorTargetRaw",        motorTargetRaw);
    SmartLogger.logReplay("Turret/Through360/OmegaRadPerSec",        inputs.robotOmegaRadPerSecond);

    double motorTarget = latchedTurretRotations * Constants.Turret.TURRET_GEAR_RATIO
        + Constants.Turret.TURRET_FORWARD_MOTOR_ROT;
    goal.targetReachable = rawReachable; // use live bearing for reachability — latch is stale during wrap
    SmartLogger.logReplay("Turret/AimSolver/TargetReachable", goal.targetReachable);
    SmartLogger.logReplay("Turret/AimSolver/MotorTarget", motorTarget);

    TurretShotProfile shot = TurretShotProfile.getForDistance(latchedDistance);

    goal.turretRotations  = latchedTurretRotations;
    goal.hoodRotations    = shot.hoodRotations;
    goal.flywheelFrontRps = shot.flywheelFrontRps;
    goal.flywheelBackRps  = shot.flywheelBackRps;
    goal.chassisSpeedMps  = inputs.robotSpeedMetersPerSecond;

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

    // Phase 3 and 4: target is in the deadzone — pre-position to the closer soft limit edge
    // so the turret is already swung to the correct side when the target re-emerges.
    // Edge values must be in robot-relative turret rotations (same frame as goal.turretRotations):
    // robotRelative = (motorLimit - TURRET_FORWARD_MOTOR_ROT) / TURRET_GEAR_RATIO
    if (!goal.targetReachable) {
      double leftEdge  = (Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT
                          - Constants.Turret.TURRET_FORWARD_MOTOR_ROT) / Constants.Turret.TURRET_GEAR_RATIO;
      double rightEdge = (Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT
                          - Constants.Turret.TURRET_FORWARD_MOTOR_ROT) / Constants.Turret.TURRET_GEAR_RATIO;
      double distToLeft  = Math.abs(rawTurretRotations - leftEdge);
      double distToRight = Math.abs(rawTurretRotations - rightEdge);
      goal.turretRotations = (distToLeft <= distToRight) ? leftEdge : rightEdge;
      goal.enable = true;
      SmartLogger.logReplay("Turret/AimSolver/WhyNotReady", "target in deadzone — wrapping");
      SmartLogger.logReplay("Turret/Through360/WrapEdgeRot", goal.turretRotations);
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

  // Distance from the turret pivot to the target, using the pivot-shifted robot pose.
  private static double pivotToTargetDistance(TurretAimInputs inputs) {
    return Math.hypot(
        inputs.targetPose.getX() - inputs.robotPose.getX(),
        inputs.targetPose.getY() - inputs.robotPose.getY());
  }
}
