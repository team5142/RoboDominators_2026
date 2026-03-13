package frc.robot.subsystems.turret;

// Desired turret aim goal in field-relative rotations
public class TurretAimGoal {
  public double turretRotations = 0.0;
  public double hoodRotations = 0.0;
  public double flywheelPercent = 0.0;   // legacy percent mode (unused when useRps=true)
  public boolean useRps = false;         // when true, front/back RPS are used instead of flywheelPercent
  public double flywheelFrontRps = 0.0;
  public double flywheelBackRps  = 0.0;
  public boolean enable = false;
  // True when the target bearing falls within the turret's physical travel range.
  // False = target is in the blind spot; robot needs to rotate to bring it in range.
  public boolean targetReachable = true;
  // Last chassis speed from the aim pipeline — used by isReadyToShoot() speed gate.
  public double chassisSpeedMps = 0.0;

  public void clear() {
    turretRotations = 0.0;
    hoodRotations = 0.0;
    flywheelPercent = 0.0;
    useRps = false;
    flywheelFrontRps = 0.0;
    flywheelBackRps  = 0.0;
    enable = false;
    targetReachable = true;
    chassisSpeedMps = 0.0;
  }
}
