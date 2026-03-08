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

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

import static edu.wpi.first.units.Units.Volts;

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

  // Hood step positions: 10% increments of travel (0%, 10%, 20%, ... 100%)
  private static final double[] HOOD_STEPS = {
    0.0,
    Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS * 0.10,
    Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS * 0.20,
    Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS * 0.30,
    Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS * 0.40,
    Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS * 0.50,
    Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS * 0.60,
    Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS * 0.70,
    Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS * 0.80,
    Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS * 0.90,
    Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS
  };
  // Up button ping-pongs 0→1→2→3→2→1→0... Down button toggles 0↔3
  private int hoodStepIndex = 0;
  private boolean hoodStepDirectionUp = true;

  // Turret homing state — turret must home before Phase 2+ aim solve is trusted
  private boolean homed = false;
  private boolean homing = false;
  private boolean manualPositionOverride = false; // when true, skip aim pipeline and hold setpoints.turretPositionMotorRotations
  // Two-phase homing removed — now homes on leading edge (sensor turns on) at fast speed only.
  private int homingStallLoopCount = 0;
  // Set true once the debounced leading edge fires — trailing edge watch begins after this.
  private boolean homingSeenLeadingEdge = false;
  // Debounce counters for hall sensor — require N consecutive matching reads before acting
  private int hallHighCount = 0;
  private int hallLowCount  = 0;

  // Hood homing state — hood creeps down until stall is detected at the hard stop, then zeroes encoder
  private boolean hoodHomed  = false;
  private boolean hoodHoming = false;
  private int hoodHomingStallLoopCount = 0;

  // Edge detection for hall sensor — log once on rising edge for console visibility
  private boolean lastHallCCW = false;

  // Last known encoder position — updated each loop after homing, used to restore encoder
  // if the motor controller reboots mid-match (brownout/power loss) while the RIO stays up.
  private double lastKnownPosition = 0.0;
  private static final double POSITION_SAVE_THRESHOLD = 0.05; // only update if moved this many motor rotations

  // SysId routines — one per flywheel motor, used for kV/kS/kA characterization only.
  // Call sysIdFrontQuasistatic/Dynamic or sysIdBackQuasistatic/Dynamic from RobotContainer.
  private final SysIdRoutine sysIdFront;
  private final SysIdRoutine sysIdBack;

  // Flywheel RPM settle recorder — samples RPM 3s after each bumper press, prints table when all 8 done.
  private static final int SETTLE_LOOPS = 150; // 3s at 50Hz
  private static final int RPM_TABLE_SIZE = 8;
  private final double[] rpmTablePct      = new double[RPM_TABLE_SIZE];
  private final double[] rpmTableFrontRpm = new double[RPM_TABLE_SIZE];
  private final double[] rpmTableBackRpm  = new double[RPM_TABLE_SIZE];
  private int rpmTableCount   = 0; // how many steps have been recorded
  private int settleLoopsLeft = -1; // -1 = idle, >0 = counting down
  private double pendingPct   = 0.0;

  public TurretSubsystem(RobotState robotState, TurretIO io) {
    this.robotState = robotState;
    this.io = io;
    SmartLogger.logConsole("Turret ready (phase: " + Constants.Turret.CURRENT_PHASE + ")", "Turret");

    sysIdFront = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null,
            state -> com.ctre.phoenix6.SignalLogger.writeString("sysid-test-state", state.toString())),
        new SysIdRoutine.Mechanism(
            (Voltage v) -> io.setFlywheelFrontVoltage(v.in(Volts)),
            null, this));

    sysIdBack = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null,
            state -> com.ctre.phoenix6.SignalLogger.writeString("sysid-test-state", state.toString())),
        new SysIdRoutine.Mechanism(
            (Voltage v) -> io.setFlywheelBackVoltage(v.in(Volts)),
            null, this));

    if (!Constants.Turret.REQUIRE_TURRET_FORWARD_CONFIRM) {
      homeForward();
    }
    hoodHome(); // always auto-home the hood at boot — safe regardless of turret position
  }

  // Seeds the turret encoder to TURRET_FORWARD_MOTOR_ROT and marks homed — no sweep needed.
  // Safe to call any time the turret is physically pointing forward.
  // In tournament mode this is called automatically at boot. In testing mode, operator
  // confirms with LT+RT+A after visually verifying the turret is forward.
  public void homeForward() {
    io.restoreTurretEncoder(Constants.Turret.TURRET_FORWARD_MOTOR_ROT);
    homed = true;
    homing = false;
    lastKnownPosition = Constants.Turret.TURRET_FORWARD_MOTOR_ROT;
    SmartLogger.logConsole("Turret zeroed to forward position (" + Constants.Turret.TURRET_FORWARD_MOTOR_ROT + " rot)", "Turret");
  }

  public Command sysIdFrontQuasistatic(SysIdRoutine.Direction dir) { return sysIdFront.quasistatic(dir); }
  public Command sysIdFrontDynamic(SysIdRoutine.Direction dir)     { return sysIdFront.dynamic(dir); }
  public Command sysIdBackQuasistatic(SysIdRoutine.Direction dir)  { return sysIdBack.quasistatic(dir); }
  public Command sysIdBackDynamic(SysIdRoutine.Direction dir)      { return sysIdBack.dynamic(dir); }

  // ---- Flywheel RPM settle recorder ----

  // Call right after bumping the flywheel speed. Samples both RPMs 3s later, then
  // prints a formatted table to the console once all RPM_TABLE_SIZE steps are done.
  public void scheduleFlywheelRpmRecord(double pct) {
    pendingPct = pct;
    settleLoopsLeft = SETTLE_LOOPS;
    System.out.println("[RPM Recorder] Scheduled step " + (rpmTableCount + 1) + "/8 at " + (int)(pct * 100) + "% - sampling in 3s");
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
    if (homing) return; // already in progress
    homed = false; // allow re-homing if called again
    homing = true;
    homingSeenLeadingEdge = false;
    hallHighCount = 0;
    hallLowCount  = 0;
    lastHallCCW   = false; // force rising edge to fire cleanly when debounce completes
    SmartLogger.logConsole("Turret homing started", "Turret");
  }

  // Clears homed state without starting a new sweep — lets you jog the turret CW and re-home via Start.
  public void resetHomed() {
    homed = false;
    homing = false;
    homingSeenLeadingEdge = false;
  }

  // Cancels an in-progress homing sweep and stops the motor — called when the trigger is released.
  public void cancelHoming() {
    if (!homing) return;
    homing = false;
    homingSeenLeadingEdge = false;
    homingStallLoopCount = 0;
    setpoints.turretPercent = 0.0;
    SmartLogger.logConsole("Turret homing cancelled", "Turret");
  }

  // Starts hood homing — creeps downward until stall current is detected at the hard stop, then zeroes encoder.
  // When the limit switch is wired, replace the stall check in periodic() with hoodLimitSwitchRaw.
  public void hoodHome() {
    if (hoodHoming) return; // already in progress
    hoodHomed = false; // allow re-homing if called again
    hoodHoming = true;
    hoodHomingStallLoopCount = 0;
    setpoints.hoodPercent = -Constants.Turret.HOOD_HOME_SPEED_PERCENT;
    SmartLogger.logConsole("Hood homing started - creeping to hard stop", "Turret");
  }

  public boolean isHomed()     { return homed; }
  public boolean isHoodHomed() { return hoodHomed; }

  // ---- Open loop / manual setpoints ----

  public void setFlywheelPercent(double percent)  {
    setpoints.flywheelPercent = percent;
    setpoints.useIndependentFlywheel = false;
  }
  // Drive each flywheel independently — for commissioning: verify each motor's direction and RPM separately
  public void setFlywheelFrontPercent(double percent) {
    setpoints.useIndependentFlywheel = true;
    setpoints.useFlywheelRps = false;
    setpoints.flywheelFrontPercent = percent;
  }
  public void setFlywheelBackPercent(double percent)  {
    setpoints.useIndependentFlywheel = true;
    setpoints.useFlywheelRps = false;
    setpoints.flywheelBackPercent = percent;
  }
  // Closed-loop velocity control — both motors independently in RPS (motor shaft rot/sec).
  // Measured 2026-03-07: 80% -> front=78.10 RPS, back=74.58 RPS. See Constants.Turret.MEASURED_*_RPS.
  public void setFlywheelFrontRps(double rps) {
    setpoints.useIndependentFlywheel = true;
    setpoints.useFlywheelRps = true;
    setpoints.flywheelFrontRps = rps;
  }
  public void setFlywheelBackRps(double rps) {
    setpoints.useIndependentFlywheel = true;
    setpoints.useFlywheelRps = true;
    setpoints.flywheelBackRps = rps;
  }
  public void setHoodPercent(double percent)       { setpoints.hoodPercent = percent; setpoints.useHoodPosition = false; }

  // Commands hood to a specific position. Requires hoodHomed. Clamps to soft limits.
  public void setHoodPositionTarget(double motorRotations) {
    if (!hoodHomed) return;
    double clamped = Math.max(0.0, Math.min(motorRotations, Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS));
    setpoints.useHoodPosition = true;
    setpoints.hoodPositionMotorRotations = clamped;
  }

  // Advances hood one step up (ping-pong: 0→1→2→3→2→1→0...)
  public void hoodStepUp() {
    if (hoodStepDirectionUp) {
      hoodStepIndex++;
      if (hoodStepIndex >= HOOD_STEPS.length - 1) { hoodStepIndex = HOOD_STEPS.length - 1; hoodStepDirectionUp = false; }
    } else {
      hoodStepIndex--;
      if (hoodStepIndex <= 0) { hoodStepIndex = 0; hoodStepDirectionUp = true; }
    }
    setHoodPositionTarget(HOOD_STEPS[hoodStepIndex]);
  }

  // Toggles hood between bottom (0) and top (HOOD_SOFT_LIMIT_TOP_ROTATIONS)
  public void hoodStepDown() {
    hoodStepIndex = (hoodStepIndex == 0) ? HOOD_STEPS.length - 1 : 0;
    hoodStepDirectionUp = (hoodStepIndex == 0);
    setHoodPositionTarget(HOOD_STEPS[hoodStepIndex]);
  }
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
    if (valid) {
      manualPositionOverride = false; // aim pipeline takes over when a real target is available
      setAimGoal(providerGoal);
    }
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

    // RPM settle recorder: count down after bumper press, then sample.
    // Ignores duplicate pct values (wrapping past 100% restarts from 30%).
    // Prints the table as soon as the 100% step is recorded.
    if (settleLoopsLeft > 0) {
      settleLoopsLeft--;
    } else if (settleLoopsLeft == 0) {
      settleLoopsLeft = -1;
      // Skip if we already recorded this pct (handles wrap-around extra presses)
      boolean alreadyRecorded = false;
      for (int i = 0; i < rpmTableCount; i++) {
        if (Math.abs(rpmTablePct[i] - pendingPct) < 0.001) { alreadyRecorded = true; break; }
      }
      if (!alreadyRecorded && rpmTableCount < RPM_TABLE_SIZE) {
        rpmTablePct[rpmTableCount]      = pendingPct;
        rpmTableFrontRpm[rpmTableCount] = inputs.flywheelVelocityRpm;
        rpmTableBackRpm[rpmTableCount]  = inputs.flywheelBackVelocityRpm;
        System.out.println(String.format("[RPM Recorder] Step %d: %.0f%% -> front=%.1f back=%.1f",
            rpmTableCount + 1, pendingPct * 100,
            rpmTableFrontRpm[rpmTableCount], rpmTableBackRpm[rpmTableCount]));
        SmartLogger.logReplay("Turret/RpmRecorder/Step",     (double)(rpmTableCount + 1));
        SmartLogger.logReplay("Turret/RpmRecorder/FrontRpm", rpmTableFrontRpm[rpmTableCount]);
        SmartLogger.logReplay("Turret/RpmRecorder/BackRpm",  rpmTableBackRpm[rpmTableCount]);
        rpmTableCount++;
        // Print table as soon as 100% (the last step) is recorded
        if (Math.abs(pendingPct - 1.00) < 0.001) {
          StringBuilder sb = new StringBuilder("\n=== FLYWHEEL RPM TABLE ===\n");
          sb.append(String.format("%-6s  %-10s  %-10s%n", "PCT", "FRONT_RPM", "BACK_RPM"));
          for (int i = 0; i < rpmTableCount; i++) {
            sb.append(String.format("%-6.0f  %-10.1f  %-10.1f%n",
                rpmTablePct[i] * 100, rpmTableFrontRpm[i], rpmTableBackRpm[i]));
          }
          sb.append("==========================");
          String tableStr = sb.toString();
          System.out.println(tableStr);
          SmartLogger.logReplay("Turret/RpmRecorder/Table", tableStr);
          rpmTableCount = 0; // reset for re-run
        }
      }
    }

    // If the motor rebooted mid-match (brownout), restore the encoder from our saved position.
    // The sticky fault fires for exactly one loop then is cleared by TurretIOCTRE.
    // Also patch inputs so the rest of this loop uses the correct position, not the reset zero.
    if (homed && inputs.turretBootDuringEn) {
      io.restoreTurretEncoder(lastKnownPosition);
      inputs.turretAbsolutePositionRotations = lastKnownPosition;
      SmartLogger.logConsole("Turret motor reboot detected — encoder restored to " +
          String.format("%.3f", lastKnownPosition) + " rot", "Turret");
    }

    // Save position each loop after homing, but only if it changed meaningfully
    if (homed && Math.abs(inputs.turretAbsolutePositionRotations - lastKnownPosition) > POSITION_SAVE_THRESHOLD) {
      lastKnownPosition = inputs.turretAbsolutePositionRotations;
    }

    // Debounce the hall sensor — rising edge uses 3 loops (60ms) to filter noise on entry.
    // Falling edge is checked raw (no debounce) during homing — only 0.075 rot to hard stop after trailing edge.
    if (inputs.hallCCWRaw) {
      hallHighCount = Math.min(hallHighCount + 1, Constants.Turret.TURRET_HALL_DEBOUNCE_LOOPS);
      hallLowCount  = 0;
    } else {
      hallLowCount  = Math.min(hallLowCount  + 1, Constants.Turret.TURRET_HALL_DEBOUNCE_LOOPS_OFF);
      hallHighCount = 0;
    }
    boolean hallDebounced    = hallHighCount >= Constants.Turret.TURRET_HALL_DEBOUNCE_LOOPS;
    boolean hallOffDebounced = hallLowCount  >= Constants.Turret.TURRET_HALL_DEBOUNCE_LOOPS_OFF;

    // Rising edge: sensor just debounced ON. Falling edge: sensor just debounced OFF.
    // lastHallCCW tracks the last stable debounced state — only update it when a side fully debounces.
    boolean hallRisingEdge  = hallDebounced    && !lastHallCCW;
    boolean hallFallingEdge = hallOffDebounced && lastHallCCW;
    if (hallRisingEdge)
      SmartLogger.logConsole("HallCCW ON  — rotations: " + String.format("%.3f", inputs.turretAbsolutePositionRotations), "Turret");
    if (hallFallingEdge)
      SmartLogger.logConsole("HallCCW OFF — rotations: " + String.format("%.3f", inputs.turretAbsolutePositionRotations), "Turret");
    if (hallDebounced)    lastHallCCW = true;
    if (hallOffDebounced) lastHallCCW = false;

    if (homing) {
      if (!homingSeenLeadingEdge) {
        // Waiting for debounced leading edge — confirms we are solidly inside the sensor window.
        if (hallRisingEdge) {
          homingSeenLeadingEdge = true;
          SmartLogger.logConsole("Homing: leading edge confirmed, watching for trailing edge", "Turret");
        }
      } else {
        // Leading edge seen — zero on the FIRST loop the raw sensor goes low (no debounce).
        // No margin between trailing edge and hard stop, so we cannot wait even one extra loop.
        if (!inputs.hallCCWRaw) {
          homing = false;
          homed  = true;
          homingStallLoopCount = 0;
          io.zeroTurretEncoder();
          SmartLogger.logConsole("Turret homed — encoder zeroed at trailing edge (raw)", "Turret");
        }
      }
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

    // Complete hood homing via stall detection at the hard stop.
    // Requires sustained stall current for several loops to avoid false triggers.
    if (hoodHoming) {
      if (inputs.hoodMotorCurrentAmps >= Constants.Turret.HOOD_HOMING_STALL_CURRENT_AMPS) {
        hoodHomingStallLoopCount++;
        if (hoodHomingStallLoopCount >= Constants.Turret.HOOD_HOMING_STALL_LOOP_THRESHOLD) {
          hoodHoming = false;
          hoodHomed  = true;
          hoodHomingStallLoopCount = 0;
          io.zeroHoodEncoder();
          // Back off slightly so the motor isn't sitting against the hard stop.
          setpoints.hoodPositionMotorRotations = Constants.Turret.HOOD_HOME_BACKOFF_ROTATIONS;
          setpoints.useHoodPosition = true;
          setpoints.hoodPercent = 0.0;
          SmartLogger.logConsole("Hood homed — encoder zeroed at hard stop (stall detected)", "Turret");
        }
      } else {
        hoodHomingStallLoopCount = 0;
      }
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

    // If not actively homing, run the normal aim pipeline unless a manual position override is active
    if (!homing) {
      if (!manualPositionOverride) {
        setpointGenerator.update(state, aimGoal, setpoints);
      }
      controller.update(state, setpoints, outputs);
    }

    if (outputs.useIndependentFlywheel) {
      if (outputs.useFlywheelRps) {
        io.setFlywheelFrontRps(outputs.flywheelFrontRps);
        io.setFlywheelBackRps(outputs.flywheelBackRps);
      } else {
        io.setFlywheelFrontPercent(outputs.flywheelFrontPercent);
        io.setFlywheelBackPercent(outputs.flywheelBackPercent);
      }
    } else {
      io.setFlywheelPercent(outputs.flywheelPercent);
    }
    if (hoodHoming) {
      io.setHoodPercent(-Constants.Turret.HOOD_HOME_SPEED_PERCENT);
    } else if (outputs.useHoodPosition) {
      io.setHoodPosition(outputs.hoodPositionMotorRotations);
    } else {
      io.setHoodPercent(outputs.hoodPercent);
    }

    // Homing uses open-loop percent. After homing, MotionMagic takes over when a position
    // target is active, otherwise open-loop percent is used (e.g. manual joystick).
    if (homing) {
      io.setTurretPercent(-Constants.Turret.TURRET_HOME_SPEED_FAST_PERCENT);
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
    SmartLogger.logReplay("Turret/RotationMotorRot", inputs.turretAbsolutePositionRotations);
    SmartLogger.logReplay("Turret/VelocityRps", inputs.turretVelocityRps);
    SmartLogger.logReplay("Turret/CurrentAmps", inputs.turretMotorCurrentAmps);

    // Hood PID tuning signals — plot HoodTargetRot vs HoodActualRot to tune kP/kV/kS
    SmartLogger.logReplay("Turret/HoodTargetRot", outputs.hoodPositionMotorRotations);
    SmartLogger.logReplay("Turret/HoodActualRot", inputs.hoodMotorPositionRotations);
    SmartLogger.logReplay("Turret/HoodErrorRot",  outputs.hoodPositionMotorRotations - inputs.hoodMotorPositionRotations);
    SmartLogger.logReplay("Turret/HoodCurrentAmps", inputs.hoodMotorCurrentAmps);

    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/RotationDeg", inputs.turretAbsolutePositionRotations * 360.0);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/HoodRotations", inputs.hoodMotorPositionRotations);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Turret/HoodHomed", hoodHomed);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Turret/HallCCW", inputs.hallCCWRaw);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelFrontRpm", inputs.flywheelVelocityRpm);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelBackRpm", inputs.flywheelBackVelocityRpm);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelFrontTargetRps", outputs.flywheelFrontRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelBackTargetRps", outputs.flywheelBackRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelFrontActualRps", inputs.flywheelVelocityRpm / 60.0);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelBackActualRps", inputs.flywheelBackVelocityRpm / 60.0);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelSetpointPct", outputs.flywheelFrontPercent);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Turret/ReadyToShoot", isReadyToShoot());
    SmartLogger.logReplay("Turret/TargetReachable", aimGoal.targetReachable);
  }

  // Sets a closed-loop position target in motor rotations (0 = CCW hard stop).
  // Clamped to safe travel range. Requires homing to be complete.
  // Bypasses the aim pipeline — writes directly to setpoints so TURRET_FORWARD_MOTOR_ROT is not added.
  public void setTurretPositionTarget(double motorRotations) {
    if (!homed) return;
    double clamped = Math.max(Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT,
                              Math.min(motorRotations, Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT));
    manualPositionOverride = true;
    setpoints.turretPositionMotorRotations = clamped;
    setpoints.useTurretPosition = true;
  }
}

