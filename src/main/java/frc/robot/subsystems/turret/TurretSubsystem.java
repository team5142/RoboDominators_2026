package frc.robot.subsystems.turret;

// Turret subsystem - flywheel shooter, adjustable hood, and rotating turret base.
// Uses an IO layer (TurretIO/TurretIOCTRE) to isolate hardware calls from logic.
//
// STARTUP SEQUENCE:
//   1. homeForward() if turret is physically pointing forward (seeds encoder, no sweep).
//   2. home() for a full hall-sensor sweep if encoder position is unknown.
//   3. hoodHome() runs automatically at boot.
//
// SHOOTING: set flywheel RPS, wait for isFlywheelSpinningFast(), then feed balls.

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartLogger;

public class TurretSubsystem extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputs inputs = new TurretIOInputs();

  // Flywheel
  private double flywheelPercent  = 0.0;
  private double flywheelFrontRps = 0.0;
  private double flywheelBackRps  = 0.0;
  private boolean useFlywheelRps  = false;

  // Turret rotation
  private double turretPercent        = 0.0;
  private double turretPositionTarget = 0.0;
  private boolean useTurretPosition   = false;
  private boolean homed               = false;
  private boolean homing              = false;
  private boolean homingSeenLeadingEdge = false;
  private int homingStallLoopCount    = 0;

  // Hood
  private double hoodPercent          = 0.0;
  private double hoodPositionTarget   = 0.0;
  private boolean useHoodPosition     = false;
  private boolean hoodHomed           = false;
  private boolean hoodHoming          = false;
  private int hoodHomingStallLoopCount = 0;

  // Hood D-pad step positions (10% increments of full travel range)
  private int hoodStepIndex = 0;
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

  public TurretSubsystem(TurretIO io) {
    this.io = io;
    hoodHome();
    SmartLogger.logConsole("Turret ready", "Turret");
  }

  // Seeds encoder to the forward position without doing a hall sweep.
  // Safe to call any time the turret is physically pointing forward.
  public void homeForward() {
    io.restoreTurretEncoder(Constants.Turret.TURRET_FORWARD_MOTOR_ROT);
    homed = true;
    homing = false;
    SmartLogger.logConsole("Turret zeroed to forward", "Turret");
  }

  // Starts a CCW hall-sensor homing sweep.
  // Drives slowly CCW until the hall sensor fires, then zeros the encoder.
  public void home() {
    if (homing) return;
    homed = false;
    homing = true;
    homingSeenLeadingEdge = false;
    homingStallLoopCount = 0;
    SmartLogger.logConsole("Turret homing started", "Turret");
  }

  // Creeps hood down to the limit switch and zeros the encoder.
  // If the switch is already pressed at boot, zeros immediately without moving.
  public void hoodHome() {
    if (hoodHoming) return;
    if (inputs.hoodLimitSwitchRaw) {
      hoodHomed = true;
      io.zeroHoodEncoder();
      SmartLogger.logConsole("Hood already at home - zeroed", "Turret");
      return;
    }
    hoodHomed = false;
    hoodHoming = true;
    hoodHomingStallLoopCount = 0;
    SmartLogger.logConsole("Hood homing started", "Turret");
  }

  public boolean isHomed()     { return homed; }
  public boolean isHoodHomed() { return hoodHomed; }

  // ---- Flywheel ----

  // Set both flywheels to the same duty cycle (0.0-1.0)
  public void setFlywheelPercent(double percent) {
    useFlywheelRps = false;
    flywheelPercent = percent;
  }

  // Closed-loop velocity control (motor shaft rotations/sec)
  public void setFlywheelFrontRps(double rps) { useFlywheelRps = true; flywheelFrontRps = rps; }
  public void setFlywheelBackRps(double rps)  { useFlywheelRps = true; flywheelBackRps  = rps; }

  // True when both flywheels have reached their commanded speed (within 5%)
  public boolean isFlywheelSpinningFast() {
    if (!useFlywheelRps || flywheelFrontRps <= 0 || flywheelBackRps <= 0) {
      return inputs.flywheelVelocityRpm    >= Constants.Turret.FLYWHEEL_SPINUP_MIN_RPM
          && inputs.flywheelBackVelocityRpm >= Constants.Turret.FLYWHEEL_SPINUP_MIN_RPM;
    }
    return (inputs.flywheelVelocityRpm    / 60.0) >= flywheelFrontRps * 0.95
        && (inputs.flywheelBackVelocityRpm / 60.0) >= flywheelBackRps  * 0.95;
  }

  // True when homed and flywheels are at speed - safe to feed balls
  public boolean isReadyToShoot() {
    return homed && isFlywheelSpinningFast();
  }

  // ---- Turret rotation ----

  // Open-loop rotation  stops at soft limits once homed
  public void setTurretPercent(double percent) {
    useTurretPosition = false;
    turretPercent = percent;
  }

  // MotionMagic closed-loop position (motor rotations from home). Requires homing.
  public void setTurretPositionTarget(double motorRotations) {
    if (!homed) return;
    turretPositionTarget = Math.max(Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT,
        Math.min(motorRotations, Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT));
    useTurretPosition = true;
  }

  // Switch from open-loop jog to MotionMagic hold at the current position.
  // Call on D-pad release after manual jogging.
  public void snapTurretToCurrentPosition() {
    turretPositionTarget = inputs.turretAbsolutePositionRotations;
    useTurretPosition = true;
  }

  // ---- Hood ----

  public void setHoodPercent(double percent) {
    useHoodPosition = false;
    hoodPercent = percent;
  }

  // MotionMagic closed-loop position for the hood. Requires hoodHomed.
  public void setHoodPositionTarget(double motorRotations) {
    if (!hoodHomed) return;
    hoodPositionTarget = Math.max(0.0,
        Math.min(motorRotations, Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS));
    useHoodPosition = true;
  }

  // D-pad up: step hood one position up (10% of range per press)
  public void hoodStepUp() {
    hoodStepIndex = Math.min(hoodStepIndex + 1, HOOD_STEPS.length - 1);
    hoodPositionTarget = HOOD_STEPS[hoodStepIndex];
    useHoodPosition = true;
  }

  // D-pad down: step hood one position down (10% of range per press)
  public void hoodStepDown() {
    hoodStepIndex = Math.max(hoodStepIndex - 1, 0);
    hoodPositionTarget = HOOD_STEPS[hoodStepIndex];
    useHoodPosition = true;
  }

  public void stopAll() {
    flywheelPercent = 0.0; flywheelFrontRps = 0.0; flywheelBackRps = 0.0; useFlywheelRps = false;
    turretPercent = 0.0; useTurretPosition = false;
    hoodPercent = 0.0; useHoodPosition = false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Turret homing: drive CCW slowly, zero encoder on the trailing edge of the hall sensor
    if (homing) {
      if (!homingSeenLeadingEdge) {
        if (inputs.hallCCWRaw) homingSeenLeadingEdge = true;
      } else {
        if (!inputs.hallCCWRaw) {
          homing = false;
          homed  = true;
          io.zeroTurretEncoder();
          SmartLogger.logConsole("Turret homed - encoder zeroed", "Turret");
        }
      }
      if (inputs.turretMotorCurrentAmps >= Constants.Turret.TURRET_HOMING_STALL_CURRENT_AMPS) {
        if (++homingStallLoopCount >= Constants.Turret.TURRET_HOMING_STALL_LOOP_THRESHOLD) {
          homing = false; homingStallLoopCount = 0; turretPercent = 0.0;
          SmartLogger.logConsoleError("Turret homing aborted - stall detected");
        }
      } else { homingStallLoopCount = 0; }
    }

    // Hood homing: creep down until limit switch trips (or stall fallback)
    if (hoodHoming) {
      if (inputs.hoodLimitSwitchRaw
          || (inputs.hoodMotorCurrentAmps >= Constants.Turret.HOOD_HOMING_STALL_CURRENT_AMPS
              && ++hoodHomingStallLoopCount >= Constants.Turret.HOOD_HOMING_STALL_LOOP_THRESHOLD)) {
        hoodHoming = false;
        hoodHomed  = true;
        io.zeroHoodEncoder();
        hoodPositionTarget = Constants.Turret.HOOD_HOME_BACKOFF_ROTATIONS;
        useHoodPosition = true;
        SmartLogger.logConsole("Hood homed", "Turret");
      }
    } else { hoodHomingStallLoopCount = 0; }

    // Turret soft limits - block open-loop commands that would exceed physical travel
    if (homed) {
      double pos = inputs.turretAbsolutePositionRotations;
      if (pos <= Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT  && turretPercent < 0.0) turretPercent = 0.0;
      if (pos >= Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT && turretPercent > 0.0) turretPercent = 0.0;
      if (useTurretPosition) {
        turretPositionTarget = Math.max(Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT,
            Math.min(turretPositionTarget, Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT));
      }
    }

    // Hood soft limit - stop upward movement at the top of travel
    if (!hoodHoming && inputs.hoodMotorPositionRotations >= Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS
        && hoodPercent > 0.0) {
      hoodPercent = 0.0;
    }

    // Send outputs to hardware
    if (useFlywheelRps) {
      io.setFlywheelFrontRps(flywheelFrontRps);
      io.setFlywheelBackRps(flywheelBackRps);
    } else {
      io.setFlywheelPercent(flywheelPercent);
    }

    if (hoodHoming) {
      io.setHoodPercent(-Constants.Turret.HOOD_HOME_SPEED_PERCENT);
    } else if (useHoodPosition) {
      io.setHoodPosition(hoodPositionTarget);
    } else {
      io.setHoodPercent(hoodPercent);
    }

    if (homing) {
      io.setTurretPercent(-Constants.Turret.TURRET_HOME_SPEED_FAST_PERCENT);
    } else if (useTurretPosition) {
      io.setTurretPosition(turretPositionTarget);
    } else {
      io.setTurretPercent(turretPercent);
    }

    SmartLogger.logReplay("Turret/Homed", homed);
    SmartLogger.logReplay("Turret/HoodHomed", hoodHomed);
    SmartLogger.logReplay("Turret/RotationMotorRot", inputs.turretAbsolutePositionRotations);
    SmartLogger.logReplay("Turret/TargetMotorRot", turretPositionTarget);
    SmartLogger.logReplay("Turret/FlywheelFrontRpm", inputs.flywheelVelocityRpm);
    SmartLogger.logReplay("Turret/FlywheelBackRpm", inputs.flywheelBackVelocityRpm);
    SmartLogger.logReplay("Turret/HoodActualRot", inputs.hoodMotorPositionRotations);
    SmartLogger.logReplay("Turret/HoodTargetRot", hoodPositionTarget);
    SmartLogger.logReplay("Turret/ReadyToShoot", isReadyToShoot());
  }
}