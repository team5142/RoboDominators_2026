package frc.robot.subsystems.turret;

import frc.robot.Constants;

// Shot profile interpolated from the locked-in named shot table in TurretTargets.
// Uses turret-pivot distances (not robot-center) for the three confirmed positions:
//   HUBCLOSE: 1.45m, MIDRANGE: 3.38m, OUTPOST: 5.70m
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
  private static final double[] DISTANCES = { 1.45, 3.38, 5.70 };
  private static final double[] FRONT_RPS = {
    Constants.TurretTargets.HUBCLOSE_FRONT_RPS,
    Constants.TurretTargets.MIDRANGE_FRONT_RPS,
    Constants.TurretTargets.OUTPOST_FRONT_RPS
  };
  private static final double[] BACK_RPS = {
    Constants.TurretTargets.HUBCLOSE_BACK_RPS,
    Constants.TurretTargets.MIDRANGE_BACK_RPS,
    Constants.TurretTargets.OUTPOST_BACK_RPS
  };
  private static final double[] HOODS = {
    Constants.TurretTargets.HUBCLOSE_HOOD_ROT,
    Constants.TurretTargets.MIDRANGE_HOOD_ROT,
    Constants.TurretTargets.OUTPOST_HOOD_ROT
  };
  private static final double[] TOFS = {
    Constants.TurretTargets.HUBCLOSE_TOF_SECONDS,
    Constants.TurretTargets.MIDRANGE_TOF_SECONDS,
    Constants.TurretTargets.OUTPOST_TOF_SECONDS
  };

  // Returns an interpolated shot profile for a given pivot-to-hub distance in meters.
  // Clamps to nearest table edge if outside range.
  public static TurretShotProfile getForDistance(double distanceMeters) {
    if (distanceMeters <= DISTANCES[0]) {
      return new TurretShotProfile(FRONT_RPS[0], BACK_RPS[0], HOODS[0], TOFS[0]);
    }
    int last = DISTANCES.length - 1;
    if (distanceMeters >= DISTANCES[last]) {
      return new TurretShotProfile(FRONT_RPS[last], BACK_RPS[last], HOODS[last], TOFS[last]);
    }
    for (int i = 0; i < last; i++) {
      if (distanceMeters <= DISTANCES[i + 1]) {
        double t = (distanceMeters - DISTANCES[i]) / (DISTANCES[i + 1] - DISTANCES[i]);
        return new TurretShotProfile(
          lerp(FRONT_RPS[i], FRONT_RPS[i + 1], t),
          lerp(BACK_RPS[i],  BACK_RPS[i + 1],  t),
          lerp(HOODS[i],     HOODS[i + 1],      t),
          lerp(TOFS[i],      TOFS[i + 1],       t));
      }
    }
    return new TurretShotProfile(FRONT_RPS[last], BACK_RPS[last], HOODS[last], TOFS[last]);
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }
}

