package frc.robot.subsystems.turret;

// Preset turret shot settings
public class TurretShotProfile {
  public final double flywheelPercent;
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
}
