package frc.robot.subsystems.turret;

// Turret sensor snapshot for controller logic
public class TurretState {
  public boolean hoodBeamBreakRaw = false;
  public boolean hallLeftRaw = false;
  public boolean hallRightRaw = false;

  public double hoodAbsolutePositionRotations = 0.0;
  public double turretAbsolutePositionRotations = 0.0;

  public void updateFromInputs(TurretIOInputs inputs) {
    hoodBeamBreakRaw = inputs.hoodBeamBreakRaw;
    hallLeftRaw = inputs.hallLeftRaw;
    hallRightRaw = inputs.hallRightRaw;

    hoodAbsolutePositionRotations = inputs.hoodAbsolutePositionRotations;
    turretAbsolutePositionRotations = inputs.turretAbsolutePositionRotations;
  }
}
