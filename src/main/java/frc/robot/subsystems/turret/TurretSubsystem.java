// Turret subsystem - flywheel shooter, adjustable hood, and rotating turret base.
// Supports phased enablement from static fixed shots (Phase 1) through on-the-move
// tracking (Phase 4). Advance phases by changing Constants.Turret.CURRENT_PHASE.
//
// HOMING: on first enable (Phase 2+), the turret drives slowly left until the left
// hall sensor fires, then zeros the encoder. Until homed, isReadyToShoot() returns false.
//
// FIRE INTERLOCK: the operator must call enableFire() explicitly each match.
// isReadyToShoot() checks: fireEnabled, turret on target, flywheel on target, hood on target.
//
// HARDWARE CHECKLISTS: see TurretIOCTRE.java for per-mechanism checklists
// (flywheels, hood, turret rotation). Complete all three before the system checklist below.
//
// TODO - SYSTEM COMMISSIONING CHECKLIST (after all TurretIOCTRE checklists are done):
// [ ] 1. Set ENABLE_TURRET = true in RobotContainer and deploy. Confirm no CAN/DIO faults.
// [ ] 2. Run home(): watch turret creep left, confirm it stops on hall sensor,
//        verify Turret/Homed = true in AdvantageScope and encoder reads near 0.
// [ ] 3. Fire a test shot at each of the 3 table distances (CLOSE/MID/FAR).
//        Note actual landing and adjust SHOT_TABLE_* values in Constants.
// [ ] 4. Confirm full cycle: intake -> singulate -> fire works end-to-end at PHASE_1_STATIC.
// [ ] 5. (Phase 2 advance) Confirm pose estimator is accurate, then set
//        CURRENT_PHASE = PHASE_2_TRACKING. Verify turret tracks while stationary.

package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

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

  // Fire interlock — must be explicitly enabled by operator each match
  private boolean fireEnabled = false;

  // Homing state — turret must home before Phase 2+ aim solve is trusted
  private boolean homed = false;
  private boolean homing = false;

  public TurretSubsystem(RobotState robotState, TurretIO io) {
    this.robotState = robotState;
    this.io = io;
    SmartLogger.logConsole("Turret ready (phase: " + Constants.Turret.CURRENT_PHASE + ")", "Turret");
  }

  // ---- Fire interlock ----

  // Must be called by the operator (e.g. hold button) before any shot is allowed
  public void enableFire()  { fireEnabled = true; }
  public void disableFire() { fireEnabled = false; }

  // Returns true only when all conditions are satisfied for a safe shot
  public boolean isReadyToShoot() {
    if (!fireEnabled) {
      SmartLogger.logReplay("Turret/ReadyToShoot", false);
      SmartLogger.logReplay("Turret/WhyNotReady", "fire not enabled");
      return false;
    }
    if (!homed && Constants.Turret.CURRENT_PHASE != Constants.Turret.TurretPhase.PHASE_1_STATIC) {
      SmartLogger.logReplay("Turret/ReadyToShoot", false);
      SmartLogger.logReplay("Turret/WhyNotReady", "not homed");
      return false;
    }
    if (!isTurretOnTarget()) {
      SmartLogger.logReplay("Turret/ReadyToShoot", false);
      SmartLogger.logReplay("Turret/WhyNotReady", "turret not on target");
      return false;
    }
    if (!isFlywheelOnTarget()) {
      SmartLogger.logReplay("Turret/ReadyToShoot", false);
      SmartLogger.logReplay("Turret/WhyNotReady", "flywheel not up to speed");
      return false;
    }
    if (!isHoodOnTarget()) {
      SmartLogger.logReplay("Turret/ReadyToShoot", false);
      SmartLogger.logReplay("Turret/WhyNotReady", "hood not on target");
      return false;
    }
    SmartLogger.logReplay("Turret/ReadyToShoot", true);
    SmartLogger.logReplay("Turret/WhyNotReady", "");
    return true;
  }

  private boolean isTurretOnTarget() {
    if (!aimGoal.enable) return true; // open loop — no target to check against
    double error = Math.abs(aimGoal.turretRotations - inputs.turretAbsolutePositionRotations);
    return error < Constants.Turret.TURRET_ON_TARGET_TOLERANCE_ROT;
  }

  private boolean isFlywheelOnTarget() {
    if (Math.abs(aimGoal.flywheelPercent) < ACTIVE_PERCENT_THRESHOLD) return true;
    double error = Math.abs(aimGoal.flywheelPercent - outputs.flywheelPercent);
    return error < Constants.Turret.FLYWHEEL_ON_TARGET_TOLERANCE_PCT;
  }

  private boolean isHoodOnTarget() {
    if (!aimGoal.enable) return true;
    double error = Math.abs(aimGoal.hoodRotations - inputs.hoodAbsolutePositionRotations);
    return error < Constants.Turret.HOOD_ON_TARGET_TOLERANCE_ROT;
  }

  // ---- Homing ----

  // Starts a homing sweep: drives turret slowly left until the left hall sensor fires.
  // Required before Phase 2+ aim solve is trusted. Safe to call in Phase 1 (encoder only).
  public void home() {
    if (homed) return;
    homing = true;
    setpoints.turretPercent = -Constants.Turret.TURRET_HOME_SPEED_PERCENT;
    SmartLogger.logConsole("Turret homing started", "Turret");
  }

  public boolean isHomed() { return homed; }

  // ---- Open loop / manual setpoints ----

  public void setFlywheelPercent(double percent)  { setpoints.flywheelPercent = percent; }
  public void setHoodPercent(double percent)       { setpoints.hoodPercent = percent; }
  public void setTurretPercent(double percent)     { setpoints.turretPercent = percent; }

  public void setOpenLoopPercents(double flywheelPercent, double hoodPercent, double turretPercent) {
    setpoints.flywheelPercent = flywheelPercent;
    setpoints.hoodPercent = hoodPercent;
    setpoints.turretPercent = turretPercent;
    aimGoal.clear();
  }

  // ---- Aim goal (closed loop) ----

  public void setAimGoal(TurretAimGoal goal) {
    aimGoal.turretRotations = goal.turretRotations;
    aimGoal.hoodRotations   = goal.hoodRotations;
    aimGoal.flywheelPercent = goal.flywheelPercent;
    aimGoal.enable          = goal.enable;
  }

  public boolean updateAimFromProvider(TurretAimProvider provider) {
    if (provider == null) return false;
    boolean valid = provider.update(providerGoal);
    if (valid) setAimGoal(providerGoal);
    return valid;
  }

  public void stopAll() {
    fireEnabled = false;
    homing = false;
    aimGoal.clear();
    setpoints.clear();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    state.updateFromInputs(inputs);

    // Complete homing when left hall sensor fires
    if (homing && inputs.hallLeftRaw) {
      homing = false;
      homed  = true;
      setpoints.turretPercent = 0.0;
      io.zeroTurretEncoder();
      SmartLogger.logConsole("Turret homed — encoder zeroed", "Turret");
    }

    // If not actively homing, run the normal aim pipeline
    if (!homing) {
      setpointGenerator.update(state, aimGoal, setpoints);
      controller.update(state, setpoints, outputs);
    }

    io.setFlywheelPercent(outputs.flywheelPercent);
    io.setHoodPercent(outputs.hoodPercent);
    io.setTurretPercent(homing ? -Constants.Turret.TURRET_HOME_SPEED_PERCENT : outputs.turretPercent);

    robotState.setTurretFlywheelPercent(outputs.flywheelPercent);
    robotState.setTurretHoodPercent(outputs.hoodPercent);
    robotState.setTurretRotationPercent(outputs.turretPercent);

    boolean active = Math.abs(outputs.flywheelPercent) > ACTIVE_PERCENT_THRESHOLD
        || Math.abs(outputs.hoodPercent) > ACTIVE_PERCENT_THRESHOLD
        || Math.abs(outputs.turretPercent) > ACTIVE_PERCENT_THRESHOLD;
    robotState.setTurretState(active ? RobotState.TurretState.ACTIVE : RobotState.TurretState.IDLE);

    robotState.setTurretHoodBeamBreakRaw(inputs.hoodBeamBreakRaw);
    robotState.setTurretHallLeftRaw(inputs.hallLeftRaw);
    robotState.setTurretHallRightRaw(inputs.hallRightRaw);
    robotState.setTurretHoodAbsolutePositionRotations(inputs.hoodAbsolutePositionRotations);
    robotState.setTurretRotationAbsolutePositionRotations(inputs.turretAbsolutePositionRotations);

    SmartLogger.logReplay("Turret/Phase", Constants.Turret.CURRENT_PHASE.toString());
    SmartLogger.logReplay("Turret/Homed", homed);
    SmartLogger.logReplay("Turret/FireEnabled", fireEnabled);

    // Publish human-readable values to SmartDashboard for Elastic dashboard widgets.
    // CANcoder absolute position is already in output-shaft rotations, so * 360 = degrees.
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/RotationDeg", inputs.turretAbsolutePositionRotations * 360.0);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/HoodDeg", inputs.hoodAbsolutePositionRotations * 360.0);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelRpm", inputs.flywheelVelocityRpm);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Turret/ReadyToShoot", isReadyToShoot());
  }
}
