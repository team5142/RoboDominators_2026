package frc.robot.subsystems.turret;

// Final turret outputs after control logic
public class TurretOutput {
  public double flywheelPercent = 0.0;      // both motors together (normal shot mode)
  public double flywheelFrontPercent = 0.0; // front motor override (commissioning/test)
  public double flywheelBackPercent  = 0.0; // back motor override  (commissioning/test)
  public boolean useIndependentFlywheel = false; // when true, front/back override flywheelPercent
  public double hoodPercent = 0.0;
  public boolean useHoodPosition = false;
  public double hoodPositionMotorRotations = 0.0;
  public double turretPercent = 0.0;

  // When true, turretPositionMotorRotations drives MotionMagic instead of turretPercent
  public boolean useTurretPosition = false;
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
