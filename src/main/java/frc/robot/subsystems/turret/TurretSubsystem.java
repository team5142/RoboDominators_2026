package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

// Turret subsystem facade for the rest of the robot
public class TurretSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final TurretIO io;
  private final TurretIOInputs inputs = new TurretIOInputs();
  private final TurretState state = new TurretState();
  private final TurretSetpoints setpoints = new TurretSetpoints();
  private final TurretOutput outputs = new TurretOutput();
  private final TurretController controller = new TurretController();
  private final TurretAimGoal aimGoal = new TurretAimGoal();
  private final TurretAimGoal providerGoal = new TurretAimGoal();
  private final TurretSetpointGenerator setpointGenerator = new TurretSetpointGenerator();
  private static final double ACTIVE_PERCENT_THRESHOLD = 0.02;

  public TurretSubsystem(RobotState robotState, TurretIO io) {
    this.robotState = robotState;
    this.io = io;

    SmartLogger.logConsole("Turret subsystem ready", "Turret");
  }

  public void setFlywheelPercent(double percent) {
    setpoints.flywheelPercent = percent;
  }

  public void setHoodPercent(double percent) {
    setpoints.hoodPercent = percent;
  }

  public void setTurretPercent(double percent) {
    setpoints.turretPercent = percent;
  }

  public void setSingulatorPercent(double percent) {
    setpoints.singulatorPercent = percent;
  }

  public void setOpenLoopPercents(
      double flywheelPercent,
      double hoodPercent,
      double turretPercent,
      double singulatorPercent) {
    setpoints.flywheelPercent = flywheelPercent;
    setpoints.hoodPercent = hoodPercent;
    setpoints.turretPercent = turretPercent;
    setpoints.singulatorPercent = singulatorPercent;
    aimGoal.clear();
  }

  public void setAimGoal(TurretAimGoal goal) {
    aimGoal.turretRotations = goal.turretRotations;
    aimGoal.hoodRotations = goal.hoodRotations;
    aimGoal.flywheelPercent = goal.flywheelPercent;
    aimGoal.enable = goal.enable;
  }

  public boolean updateAimFromProvider(TurretAimProvider provider) {
    if (provider == null) {
      return false;
    }

    boolean valid = provider.update(providerGoal);
    if (valid) {
      setAimGoal(providerGoal);
    }
    return valid;
  }

  public void stopAll() {
    aimGoal.clear();
    setpoints.clear();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    state.updateFromInputs(inputs);
    setpointGenerator.update(state, aimGoal, setpoints);
    controller.update(state, setpoints, outputs);

    io.setFlywheelPercent(outputs.flywheelPercent);
    io.setHoodPercent(outputs.hoodPercent);
    io.setTurretPercent(outputs.turretPercent);
    io.setSingulatorPercent(outputs.singulatorPercent);

    robotState.setTurretFlywheelPercent(outputs.flywheelPercent);
    robotState.setTurretHoodPercent(outputs.hoodPercent);
    robotState.setTurretRotationPercent(outputs.turretPercent);
    robotState.setTurretSingulatorPercent(outputs.singulatorPercent);

    boolean active = Math.abs(outputs.flywheelPercent) > ACTIVE_PERCENT_THRESHOLD
        || Math.abs(outputs.hoodPercent) > ACTIVE_PERCENT_THRESHOLD
        || Math.abs(outputs.turretPercent) > ACTIVE_PERCENT_THRESHOLD
        || Math.abs(outputs.singulatorPercent) > ACTIVE_PERCENT_THRESHOLD;
    robotState.setTurretState(active ? RobotState.TurretState.ACTIVE : RobotState.TurretState.IDLE);

    robotState.setTurretSingulatorBeamBreakRaw(inputs.singulatorBeamBreakRaw);
    robotState.setTurretHoodBeamBreakRaw(inputs.hoodBeamBreakRaw);
    robotState.setTurretHallLeftRaw(inputs.hallLeftRaw);
    robotState.setTurretHallRightRaw(inputs.hallRightRaw);

    robotState.setTurretHoodAbsolutePositionRotations(inputs.hoodAbsolutePositionRotations);
    robotState.setTurretRotationAbsolutePositionRotations(inputs.turretAbsolutePositionRotations);
  }
}
