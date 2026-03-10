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

  // Pivot distances for each named shot position (meters, turret-pivot to hub)
  private static final double[] DISTANCES = { 1.45, 3.38, 5.70 };

  public TurretShotProfile(double flywheelFrontRps, double flywheelBackRps, double hoodRotations, double timeOfFlightSeconds) {
    this.flywheelFrontRps    = flywheelFrontRps;
    this.flywheelBackRps     = flywheelBackRps;
    this.hoodRotations       = hoodRotations;
    this.timeOfFlightSeconds = timeOfFlightSeconds;
  }

  // Returns an interpolated shot profile for a given pivot-to-hub distance in meters.
  // Clamps to nearest table edge if outside range.
  public static TurretShotProfile getForDistance(double distanceMeters) {
    double[] frontRps = {
      Constants.TurretTargets.HUBCLOSE_FRONT_RPS,
      Constants.TurretTargets.MIDRANGE_FRONT_RPS,
      Constants.TurretTargets.OUTPOST_FRONT_RPS
    };
    double[] backRps = {
      Constants.TurretTargets.HUBCLOSE_BACK_RPS,
      Constants.TurretTargets.MIDRANGE_BACK_RPS,
      Constants.TurretTargets.OUTPOST_BACK_RPS
    };
    double[] hoods = {
      Constants.TurretTargets.HUBCLOSE_HOOD_ROT,
      Constants.TurretTargets.MIDRANGE_HOOD_ROT,
      Constants.TurretTargets.OUTPOST_HOOD_ROT
    };
    double[] tofs = {
      Constants.TurretTargets.HUBCLOSE_TOF_SECONDS,
      Constants.TurretTargets.MIDRANGE_TOF_SECONDS,
      Constants.TurretTargets.OUTPOST_TOF_SECONDS
    };

    if (distanceMeters <= DISTANCES[0]) {
      return new TurretShotProfile(frontRps[0], backRps[0], hoods[0], tofs[0]);
    }
    int last = DISTANCES.length - 1;
    if (distanceMeters >= DISTANCES[last]) {
      return new TurretShotProfile(frontRps[last], backRps[last], hoods[last], tofs[last]);
    }
    for (int i = 0; i < last; i++) {
      if (distanceMeters <= DISTANCES[i + 1]) {
        double t = (distanceMeters - DISTANCES[i]) / (DISTANCES[i + 1] - DISTANCES[i]);
        return new TurretShotProfile(
          lerp(frontRps[i], frontRps[i + 1], t),
          lerp(backRps[i],  backRps[i + 1],  t),
          lerp(hoods[i],    hoods[i + 1],    t),
          lerp(tofs[i],     tofs[i + 1],     t));
      }
    }
    return new TurretShotProfile(frontRps[last], backRps[last], hoods[last], tofs[last]);
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }
}

