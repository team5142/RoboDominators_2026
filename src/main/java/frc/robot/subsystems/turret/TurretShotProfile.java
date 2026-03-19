package frc.robot.subsystems.turret;

import frc.robot.Constants;

// Shot profile interpolated from the locked-in named shot table in TurretTargets.
// Uses turret-pivot distances (not robot-center) for the confirmed positions:
//   HUBCLOSE: 1.45m, RIGHT_BUMP: 2.03m, MIDRANGE: 3.38m, RIGHT_CORNER: 4.59m, OUTPOST: 5.70m
// LEFT_BUMP shares the same distance and profile as RIGHT_BUMP — not a separate entry.
// Front and back RPS are kept independent — do not average them.
public class TurretShotProfile {
  public final double flywheelFrontRps;
  public final double flywheelBackRps;
  public final double hoodRotations;
  // Ball time-of-flight in seconds — used for moving-target lead compensation.
  public final double timeOfFlightSeconds;

  public TurretShotProfile(double flywheelFrontRps, double flywheelBackRps, double hoodRotations, double timeOfFlightSeconds) {
    this.flywheelFrontRps    = flywheelFrontRps;
    this.flywheelBackRps     = flywheelBackRps;
    this.hoodRotations       = hoodRotations;
    this.timeOfFlightSeconds = timeOfFlightSeconds;
  }

  // Pivot distances for each named shot position (meters, turret-pivot to hub)
  private static final double[] DISTANCES = { 1.45, 2.03, 3.38, 4.59, 5.70 };
  private static final double[] FRONT_RPS = {
    Constants.TurretTargets.HUBCLOSE_FRONT_RPS,
    Constants.TurretTargets.RIGHT_BUMP_FRONT_RPS,
    Constants.TurretTargets.MIDRANGE_FRONT_RPS,
    Constants.TurretTargets.RIGHT_CORNER_FRONT_RPS,
    Constants.TurretTargets.OUTPOST_FRONT_RPS
  };
  private static final double[] BACK_RPS = {
    Constants.TurretTargets.HUBCLOSE_BACK_RPS,
    Constants.TurretTargets.RIGHT_BUMP_BACK_RPS,
    Constants.TurretTargets.MIDRANGE_BACK_RPS,
    Constants.TurretTargets.RIGHT_CORNER_BACK_RPS,
    Constants.TurretTargets.OUTPOST_BACK_RPS
  };
  private static final double[] HOODS = {
    Constants.TurretTargets.HUBCLOSE_HOOD_ROT,
    Constants.TurretTargets.RIGHT_BUMP_HOOD_ROT,
    Constants.TurretTargets.MIDRANGE_HOOD_ROT,
    Constants.TurretTargets.RIGHT_CORNER_HOOD_ROT,
    Constants.TurretTargets.OUTPOST_HOOD_ROT
  };
  private static final double[] TOFS = {
    Constants.TurretTargets.HUBCLOSE_TOF_SECONDS,
    Constants.TurretTargets.RIGHT_BUMP_TOF_SECONDS,
    Constants.TurretTargets.MIDRANGE_TOF_SECONDS,
    Constants.TurretTargets.RIGHT_CORNER_TOF_SECONDS,
    Constants.TurretTargets.OUTPOST_TOF_SECONDS
  };

  // Global RPS scale factor — reduce to shoot shorter, increase to shoot farther.
  // Applied to both front and back RPS at interpolation time. Constants stay as measured reference.
  private static final double RPS_SCALE = 0.93; // -7% 2026-03-18: consistently overshooting when stationary

  // Global TOF scale factor — affects lead compensation only (stationary shots unaffected).
  // Reduce if moving shots overshoot in direction of travel; increase if they undershoot.
  private static final double TOF_SCALE = 0.75; // -25% 2026-03-18: overshooting while moving

  // Returns an interpolated shot profile for a given pivot-to-hub distance in meters.
  // Clamps to nearest table edge if outside range.
  public static TurretShotProfile getForDistance(double distanceMeters) {
    if (distanceMeters <= DISTANCES[0]) {
      return new TurretShotProfile(FRONT_RPS[0] * RPS_SCALE, BACK_RPS[0] * RPS_SCALE, HOODS[0], TOFS[0] * TOF_SCALE);
    }
    int last = DISTANCES.length - 1;
    if (distanceMeters >= DISTANCES[last]) {
      return new TurretShotProfile(FRONT_RPS[last] * RPS_SCALE, BACK_RPS[last] * RPS_SCALE, HOODS[last], TOFS[last] * TOF_SCALE);
    }
    for (int i = 0; i < last; i++) {
      if (distanceMeters <= DISTANCES[i + 1]) {
        double t = (distanceMeters - DISTANCES[i]) / (DISTANCES[i + 1] - DISTANCES[i]);
        return new TurretShotProfile(
          lerp(FRONT_RPS[i], FRONT_RPS[i + 1], t) * RPS_SCALE,
          lerp(BACK_RPS[i],  BACK_RPS[i + 1],  t) * RPS_SCALE,
          lerp(HOODS[i],     HOODS[i + 1],      t),
          lerp(TOFS[i],      TOFS[i + 1],       t) * TOF_SCALE);
      }
    }
    return new TurretShotProfile(FRONT_RPS[last] * RPS_SCALE, BACK_RPS[last] * RPS_SCALE, HOODS[last], TOFS[last] * TOF_SCALE);
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }
}

