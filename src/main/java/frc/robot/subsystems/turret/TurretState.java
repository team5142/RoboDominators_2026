package frc.robot.subsystems.turret;

// Turret sensor snapshot for controller logic
public class TurretState {
  public boolean hoodLimitSwitchRaw = false;
  public boolean hallLeftRaw = false;
  public boolean hallRightRaw = false;

  public double hoodMotorPositionRotations = 0.0;
  public double turretAbsolutePositionRotations = 0.0;

  public void updateFromInputs(TurretIOInputs inputs) {
    hoodLimitSwitchRaw = inputs.hoodLimitSwitchRaw;
    hallLeftRaw = inputs.hallLeftRaw;
    hallRightRaw = inputs.hallRightRaw;

    hoodMotorPositionRotations = inputs.hoodMotorPositionRotations;
    turretAbsolutePositionRotations = inputs.turretAbsolutePositionRotations;
  }
}
