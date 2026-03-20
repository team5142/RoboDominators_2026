package frc.robot.subsystems.turret;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;

// Shot profile interpolated from the measured shot table using WPILib InterpolatingDoubleTreeMap.
// All entries measured 2026-03-19. Add new distance/value pairs to populate() to extend the table.
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

  private static final InterpolatingDoubleTreeMap FRONT_RPS_MAP = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap BACK_RPS_MAP  = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap HOOD_MAP      = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap TOF_MAP       = new InterpolatingDoubleTreeMap();

  static {
    populate(1.28, Constants.TurretTargets.HUBCLOSE_FRONT_RPS,   Constants.TurretTargets.HUBCLOSE_BACK_RPS,   Constants.TurretTargets.HUBCLOSE_HOOD_ROT,   Constants.TurretTargets.HUBCLOSE_TOF_SECONDS);
    populate(1.67, Constants.TurretTargets.HUB1_7M_FRONT_RPS,   Constants.TurretTargets.HUB1_7M_BACK_RPS,   Constants.TurretTargets.HUB1_7M_HOOD_ROT,   Constants.TurretTargets.HUB1_7M_TOF_SECONDS);
    populate(2.03, Constants.TurretTargets.RIGHT_BUMP_FRONT_RPS, Constants.TurretTargets.RIGHT_BUMP_BACK_RPS, Constants.TurretTargets.RIGHT_BUMP_HOOD_ROT, Constants.TurretTargets.RIGHT_BUMP_TOF_SECONDS);
    populate(2.40, Constants.TurretTargets.CENTER_2_4M_FRONT_RPS, Constants.TurretTargets.CENTER_2_4M_BACK_RPS, Constants.TurretTargets.CENTER_2_4M_HOOD_ROT, Constants.TurretTargets.CENTER_2_4M_TOF_SECONDS);
    populate(3.12, Constants.TurretTargets.LEFT_3_1M_FRONT_RPS,  Constants.TurretTargets.LEFT_3_1M_BACK_RPS,  Constants.TurretTargets.LEFT_3_1M_HOOD_ROT,  Constants.TurretTargets.LEFT_3_1M_TOF_SECONDS);
    populate(3.49, Constants.TurretTargets.LEFT_3_5M_FRONT_RPS,  Constants.TurretTargets.LEFT_3_5M_BACK_RPS,  Constants.TurretTargets.LEFT_3_5M_HOOD_ROT,  Constants.TurretTargets.LEFT_3_5M_TOF_SECONDS);
    populate(3.53, Constants.TurretTargets.RIGHT_3_5M_FRONT_RPS, Constants.TurretTargets.RIGHT_3_5M_BACK_RPS, Constants.TurretTargets.RIGHT_3_5M_HOOD_ROT, Constants.TurretTargets.RIGHT_3_5M_TOF_SECONDS);
    populate(3.99, Constants.TurretTargets.LEFT_4_0M_FRONT_RPS,  Constants.TurretTargets.LEFT_4_0M_BACK_RPS,  Constants.TurretTargets.LEFT_4_0M_HOOD_ROT,  Constants.TurretTargets.LEFT_4_0M_TOF_SECONDS);
    populate(4.41, Constants.TurretTargets.RIGHT_4_4M_FRONT_RPS, Constants.TurretTargets.RIGHT_4_4M_BACK_RPS, Constants.TurretTargets.RIGHT_4_4M_HOOD_ROT, Constants.TurretTargets.RIGHT_4_4M_TOF_SECONDS);
    populate(5.48, Constants.TurretTargets.LEFT_5_5M_FRONT_RPS,  Constants.TurretTargets.LEFT_5_5M_BACK_RPS,  Constants.TurretTargets.LEFT_5_5M_HOOD_ROT,  Constants.TurretTargets.LEFT_5_5M_TOF_SECONDS);
  }

  private static void populate(double distM, double frontRps, double backRps, double hood, double tof) {
    FRONT_RPS_MAP.put(distM, frontRps);
    BACK_RPS_MAP.put(distM, backRps);
    HOOD_MAP.put(distM, hood);
    TOF_MAP.put(distM, tof);
  }

  // Global RPS scale factor — reduce to shoot shorter, increase to shoot farther.
  // Applied to both front and back RPS at lookup time. Constants stay as measured reference.
  private static final double RPS_SCALE = 1.0; // 1.0 = no scaling — constants are measured actual RPS

  // Global TOF scale factor — affects lead compensation only (stationary shots unaffected).
  // Reduce if moving shots overshoot in direction of travel; increase if they undershoot.
  private static final double TOF_SCALE = 0.75; // -25% 2026-03-18: overshooting while moving

  // Returns an interpolated shot profile for a given pivot-to-hub distance in meters.
  // Clamps to nearest table edge if outside range (WPILib behavior).
  public static TurretShotProfile getForDistance(double distanceMeters) {
    return new TurretShotProfile(
        FRONT_RPS_MAP.get(distanceMeters) * RPS_SCALE,
        BACK_RPS_MAP.get(distanceMeters)  * RPS_SCALE,
        HOOD_MAP.get(distanceMeters),
        TOF_MAP.get(distanceMeters)       * TOF_SCALE);
  }
}
