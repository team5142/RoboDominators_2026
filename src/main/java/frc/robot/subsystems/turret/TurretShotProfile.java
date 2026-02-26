package frc.robot.subsystems.turret;

import frc.robot.Constants;

// Preset turret shot settings (flywheel percent + hood rotations for a given distance).
// Seed values are ChatGPT estimates from our flywheel build — replace each row after
// measuring on hardware. See Constants.Turret.SHOT_TABLE_* for the raw data.
public class TurretShotProfile {
  public final double flywheelPercent; // average of front+back percents
  public final double hoodRotations;
  public final double turretRotations;

  public TurretShotProfile(double flywheelPercent, double hoodRotations, double turretRotations) {
    this.flywheelPercent = flywheelPercent;
    this.hoodRotations = hoodRotations;
    this.turretRotations = turretRotations;
  }

  public TurretAimGoal toAimGoal() {
    TurretAimGoal goal = new TurretAimGoal();
    goal.flywheelPercent = flywheelPercent;
    goal.hoodRotations = hoodRotations;
    goal.turretRotations = turretRotations;
    goal.enable = true;
    return goal;
  }

  // Returns an interpolated shot profile for a given distance in meters.
  // Clamps to the nearest table edge if distance is outside the table range.
  // Front and back flywheel percents are averaged; tune separately once on hardware.
  public static TurretShotProfile getForDistance(double distanceMeters) {
    double[] distances = Constants.Turret.SHOT_TABLE_DISTANCES_M;
    double[] frontPcts = Constants.Turret.SHOT_TABLE_FLYWHEEL_FRONT_PCT;
    double[] backPcts  = Constants.Turret.SHOT_TABLE_FLYWHEEL_BACK_PCT;
    double[] hoods     = Constants.Turret.SHOT_TABLE_HOOD_ROTATIONS;

    // Clamp below minimum
    if (distanceMeters <= distances[0]) {
      return new TurretShotProfile((frontPcts[0] + backPcts[0]) / 2.0, hoods[0], 0.0);
    }
    // Clamp above maximum
    int last = distances.length - 1;
    if (distanceMeters >= distances[last]) {
      return new TurretShotProfile((frontPcts[last] + backPcts[last]) / 2.0, hoods[last], 0.0);
    }

    // Find the two surrounding rows and linearly interpolate
    for (int i = 0; i < last; i++) {
      if (distanceMeters <= distances[i + 1]) {
        double t = (distanceMeters - distances[i]) / (distances[i + 1] - distances[i]);
        double flywheel = lerp((frontPcts[i] + backPcts[i]) / 2.0,
                               (frontPcts[i + 1] + backPcts[i + 1]) / 2.0, t);
        double hood = lerp(hoods[i], hoods[i + 1], t);
        return new TurretShotProfile(flywheel, hood, 0.0);
      }
    }

    // Should never reach here given the clamps above
    return new TurretShotProfile(0.0, 0.0, 0.0);
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }
}
