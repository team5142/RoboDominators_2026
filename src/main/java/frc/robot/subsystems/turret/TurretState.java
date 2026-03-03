package frc.robot.subsystems.turret;

// Turret sensor snapshot for controller logic
public class TurretState {
  public boolean hoodLimitSwitchRaw = false;
  public boolean hallCCWRaw = false;

  public double hoodMotorPositionRotations = 0.0;
  public double turretAbsolutePositionRotations = 0.0;

  public void updateFromInputs(TurretIOInputs inputs) {
    hoodLimitSwitchRaw = inputs.hoodLimitSwitchRaw;
    hallCCWRaw = inputs.hallCCWRaw;

    hoodMotorPositionRotations = inputs.hoodMotorPositionRotations;
    turretAbsolutePositionRotations = inputs.turretAbsolutePositionRotations;
  }
}
