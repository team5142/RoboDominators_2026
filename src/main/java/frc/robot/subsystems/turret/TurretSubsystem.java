// Turret subsystem - flywheel shooter, adjustable hood, and rotating turret base.
// Supports phased enablement from static fixed shots (Phase 1) through on-the-move
// tracking (Phase 4). Advance phases by changing Constants.Turret.CURRENT_PHASE.
//
// HOMING: home() must be called before Phase 2+ tracking is trusted. The turret drives
// slowly CCW until the hall sensor fires, then zeros the encoder.
//
// MOTIONMAGIC TUNING: after homing, call setTurretPositionTarget(turretRotations) to bounce
// the turret between two points and tune kP/kS/kV in Constants via AdvantageScope.
// Watch Turret/TargetMotorRot vs Turret/RotationMotorRot for tracking quality.
//
// FIRE INTERLOCK: the operator must call enableFire() explicitly each match.
// isReadyToShoot() checks: fireEnabled, turret on target, flywheel on target, hood on target.

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

  // Turret homing state — turret must home before Phase 2+ aim solve is trusted
  private boolean homed = false;
  private boolean homing = false;
  private int homingStallLoopCount = 0;

  // Hood homing state — hood creeps down until bottom limit switch fires, then zeroes encoder
  private boolean hoodHomed  = false;
  private boolean hoodHoming = false;

  // Edge detection for hall sensor — log once on rising edge for console visibility
  private boolean lastHallCCW = false;

  public TurretSubsystem(RobotState robotState, TurretIO io) {
    this.robotState = robotState;
    this.io = io;
    SmartLogger.logConsole("Turret ready (phase: " + Constants.Turret.CURRENT_PHASE + ")", "Turret");
  }

  // ---- Fire interlock ----

  // Must be called by the operator (e.g. hold button) before any shot is allowed
  public void enableFire()  { fireEnabled = true; }
  public void disableFire() { fireEnabled = false; }

  // True when the current target bearing is within the turret's physical travel range.
  // False = target is in the ~28 deg blind spot; robot must rotate to bring it in range.
  public boolean isTurretReachable() { return aimGoal.targetReachable; }

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
    double error = Math.abs(aimGoal.hoodRotations - inputs.hoodMotorPositionRotations);
    return error < Constants.Turret.HOOD_ON_TARGET_TOLERANCE_ROT;
  }

  // ---- Homing ----

  // Starts turret homing sweep — drives slowly CCW until CCW hall sensor fires.
  // Required before Phase 2+ aim solve is trusted. Safe to call in Phase 1 (encoder only).
  public void home() {
    if (homed) return;
    homing = true;
    setpoints.turretPercent = -Constants.Turret.TURRET_HOME_SPEED_PERCENT;
    SmartLogger.logConsole("Turret homing started", "Turret");
  }

  // Cancels an in-progress homing sweep and stops the motor — called when the trigger is released.
  public void cancelHoming() {
    if (!homing) return;
    homing = false;
    homingStallLoopCount = 0;
    setpoints.turretPercent = 0.0;
    SmartLogger.logConsole("Turret homing cancelled", "Turret");
  }

  // Starts hood homing — creeps downward until the bottom limit switch fires, then zeroes encoder.
  // Safe to call if already homed (no-op).
  public void hoodHome() {
    if (hoodHomed) return;
    hoodHoming = true;
    setpoints.hoodPercent = -Constants.Turret.HOOD_HOME_SPEED_PERCENT;
    SmartLogger.logConsole("Hood homing started - moving to bottom limit switch", "Turret");
  }

  public boolean isHomed()     { return homed; }
  public boolean isHoodHomed() { return hoodHomed; }

  // ---- Open loop / manual setpoints ----

  public void setFlywheelPercent(double percent)  { setpoints.flywheelPercent = percent; }
  public void setHoodPercent(double percent)       { setpoints.hoodPercent = percent; }
  public void setTurretPercent(double percent)     { setpoints.turretPercent = percent; }

  // Open-loop turret move that respects soft limits — use this for manual jogging after homing.
  // Stops the motor if the position would exceed either limit in the commanded direction.
  public void setTurretPercentSoftLimited(double percent) {
    if (!homed) {
      setpoints.turretPercent = 0.0;
      return;
    }
    double pos = inputs.turretAbsolutePositionRotations;
    if (percent < 0.0 && pos <= Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT) {
      setpoints.turretPercent = 0.0;
      return;
    }
    if (percent > 0.0 && pos >= Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT) {
      setpoints.turretPercent = 0.0;
      return;
    }
    setpoints.turretPercent = percent;
  }

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

  // Returns the current turret motor encoder position in motor rotations — for calibration only.
  public double getTurretMotorRotations() {
    return inputs.turretAbsolutePositionRotations;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    state.updateFromInputs(inputs);

    // Log hall sensor rising edges to console for live hardware verification
    boolean hallRisingEdge = inputs.hallCCWRaw && !lastHallCCW;
    if (hallRisingEdge)
      SmartLogger.logConsole("HallCCW TRIGGERED — rotations: " + String.format("%.3f", inputs.turretAbsolutePositionRotations), "Turret");
    lastHallCCW = inputs.hallCCWRaw;

    // Complete turret homing only on the rising edge — prevents instant completion if turret
    // starts the deploy already sitting on the hall sensor.
    if (homing && hallRisingEdge) {
      homing = false;
      homed  = true;
      homingStallLoopCount = 0;
      setpoints.turretPercent = 0.0;
      io.zeroTurretEncoder();
      SmartLogger.logConsole("Turret homed — encoder zeroed", "Turret");
    }

    // Stall abort: if turret hits the hard stop before the hall sensor fires, current spikes.
    // Abort homing to avoid grinding against the stop.
    if (homing && inputs.turretMotorCurrentAmps >= Constants.Turret.TURRET_HOMING_STALL_CURRENT_AMPS) {
      homingStallLoopCount++;
      if (homingStallLoopCount >= Constants.Turret.TURRET_HOMING_STALL_LOOP_THRESHOLD) {
        homing = false;
        homingStallLoopCount = 0;
        setpoints.turretPercent = 0.0;
        SmartLogger.logConsole("Turret homing aborted — stall detected, hall sensor may be missing or misaligned", "Turret");
      }
    } else {
      homingStallLoopCount = 0;
    }

    // Complete hood homing when bottom limit switch fires
    if (hoodHoming && inputs.hoodLimitSwitchRaw) {
      hoodHoming = false;
      hoodHomed  = true;
      setpoints.hoodPercent = 0.0;
      io.zeroHoodEncoder();
      SmartLogger.logConsole("Hood homed — encoder zeroed at bottom stop", "Turret");
    }

    // Hood soft limit: stop upward movement at the top of the travel range.
    // No physical hard stop at the top — this is the only protection against over-travel.
    // Clears both the output and the setpoint so the controller doesn't fight it next loop.
    if (!hoodHoming && inputs.hoodMotorPositionRotations >= Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS
        && outputs.hoodPercent > 0.0) {
      outputs.hoodPercent  = 0.0;
      setpoints.hoodPercent = 0.0;
    }

    // Turret soft limits: block open-loop percent commands that would drive past either end of travel.
    // Also clamps any active MotionMagic position target so it can never be commanded past the limits.
    // Only enforced after homing so the limits are relative to a known zero.
    if (homed) {
      double turretPos = inputs.turretAbsolutePositionRotations;
      if (turretPos <= Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT && outputs.turretPercent < 0.0) {
        outputs.turretPercent  = 0.0;
        setpoints.turretPercent = 0.0;
      }
      if (turretPos >= Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT && outputs.turretPercent > 0.0) {
        outputs.turretPercent  = 0.0;
        setpoints.turretPercent = 0.0;
      }
      // Clamp the MotionMagic target so overshoots or stale setpoints can't command past the limits.
      if (outputs.useTurretPosition) {
        outputs.turretPositionMotorRotations = Math.max(
            Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT,
            Math.min(outputs.turretPositionMotorRotations, Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT));
      }
    }

    // If not actively homing, run the normal aim pipeline
    if (!homing) {
      setpointGenerator.update(state, aimGoal, setpoints);
      controller.update(state, setpoints, outputs);
    }

    io.setFlywheelPercent(outputs.flywheelPercent);
    io.setHoodPercent(hoodHoming ? -Constants.Turret.HOOD_HOME_SPEED_PERCENT : outputs.hoodPercent);

    // Homing uses open-loop percent. After homing, MotionMagic takes over when a position
    // target is active, otherwise open-loop percent is used (e.g. manual joystick).
    if (homing) {
      io.setTurretPercent(-Constants.Turret.TURRET_HOME_SPEED_PERCENT);
    } else if (outputs.useTurretPosition) {
      io.setTurretPosition(outputs.turretPositionMotorRotations);
    } else {
      io.setTurretPercent(outputs.turretPercent);
    }

    robotState.setTurretFlywheelPercent(outputs.flywheelPercent);
    robotState.setTurretHoodPercent(outputs.hoodPercent);
    robotState.setTurretRotationPercent(outputs.turretPercent);

    boolean active = Math.abs(outputs.flywheelPercent) > ACTIVE_PERCENT_THRESHOLD
        || Math.abs(outputs.hoodPercent) > ACTIVE_PERCENT_THRESHOLD
        || Math.abs(outputs.turretPercent) > ACTIVE_PERCENT_THRESHOLD;
    robotState.setTurretState(active ? RobotState.TurretState.ACTIVE : RobotState.TurretState.IDLE);

    robotState.setTurretHoodLimitSwitchRaw(inputs.hoodLimitSwitchRaw);
    robotState.setTurretHallCCWRaw(inputs.hallCCWRaw);
    robotState.setTurretHoodMotorPositionRotations(inputs.hoodMotorPositionRotations);
    robotState.setTurretRotationAbsolutePositionRotations(inputs.turretAbsolutePositionRotations);

    SmartLogger.logReplay("Turret/Phase", Constants.Turret.CURRENT_PHASE.toString());
    SmartLogger.logReplay("Turret/Homed", homed);
    SmartLogger.logReplay("Turret/HoodHomed", hoodHomed);
    SmartLogger.logReplay("Turret/FireEnabled", fireEnabled);

    SmartLogger.logReplay("Turret/TargetMotorRot", outputs.turretPositionMotorRotations);
    SmartLogger.logReplay("Turret/RotationMotorRot", inputs.turretAbsolutePositionRotations); // motor rotations, same units as TargetMotorRot
    SmartLogger.logReplay("Turret/VelocityRps", inputs.turretVelocityRps);
    SmartLogger.logReplay("Turret/CurrentAmps", inputs.turretMotorCurrentAmps);

    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/RotationDeg", inputs.turretAbsolutePositionRotations * 360.0);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/HoodRotations", inputs.hoodMotorPositionRotations);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Turret/HoodHomed", hoodHomed);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Turret/HallCCW", inputs.hallCCWRaw);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelRpm", inputs.flywheelVelocityRpm);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Turret/ReadyToShoot", isReadyToShoot());
    SmartLogger.logReplay("Turret/TargetReachable", aimGoal.targetReachable);
  }

  // Sets a closed-loop position target in motor rotations (0 = CCW hard stop).
  // Clamped to safe travel range. Requires homing to be complete.
  public void setTurretPositionTarget(double motorRotations) {
    if (!homed) return;
    double clamped = Math.max(Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT,
                              Math.min(motorRotations, Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT));
    aimGoal.turretRotations = clamped / Constants.Turret.TURRET_GEAR_RATIO; // SetpointGenerator expects turret rotations
    aimGoal.enable = true;
  }
}

