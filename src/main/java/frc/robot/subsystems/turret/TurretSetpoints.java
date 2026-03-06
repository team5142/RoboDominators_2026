package frc.robot.subsystems.turret;

// Desired turret outputs — either open loop percent or closed-loop position (MotionMagic)
public class TurretSetpoints {
  public double flywheelPercent = 0.0;      // both motors together (normal shot mode)
  public double flywheelFrontPercent = 0.0; // front motor override (commissioning/test)
  public double flywheelBackPercent  = 0.0; // back motor override  (commissioning/test)
  public boolean useIndependentFlywheel = false; // when true, front/back override flywheelPercent
  public double hoodPercent = 0.0;
  public boolean useHoodPosition = false;
  public double hoodPositionMotorRotations = 0.0;
  public double turretPercent = 0.0;

  // When true, turretPositionMotorRotations is used instead of turretPercent
  public boolean useTurretPosition = false;
  // Target in motor rotations (0 = CCW home, positive = CW). Set by TurretSetpointGenerator.
  public double turretPositionMotorRotations = 0.0;

  public void clear() {
    flywheelPercent = 0.0;
    flywheelFrontPercent = 0.0;
    flywheelBackPercent  = 0.0;
    useIndependentFlywheel = false;
    hoodPercent = 0.0;
    useHoodPosition = false;
    hoodPositionMotorRotations = 0.0;
    turretPercent = 0.0;
    useTurretPosition = false;
    turretPositionMotorRotations = 0.0;
  }
}
