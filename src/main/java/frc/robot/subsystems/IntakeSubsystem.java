package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.IntakePosition;
import frc.robot.RobotState.IntakeRollerState;
import frc.robot.util.SmartLogger;

// Over-bumper intake - one motor extends/retracts the arm, one motor spins the rollers.
// On first enable, the arm homes upward until both limit switches trip, then zeroes the encoder.
// No limit switch at full extension - stops at a fixed rotation target.
//
// HOMING: call startHoming() at teleopInit/autoInit. Arm creeps upward; both switches must
// trip together to confirm fully retracted. If stall is detected before both switches fire,
// state transitions to HOMING_FAILED and all motion is blocked until startHoming() is called again.
//
// STALL PROTECTION: current-based. If stator current exceeds EXTENSION_STALL_CURRENT_AMPS for
// STALL_LOOP_THRESHOLD consecutive loops (~200ms), the motor stops and extensionStalled=true.
//
// COMMISSIONING CHECKLIST:
// [x] 1. Confirm CAN IDs: INTAKE_ROLLER_MOTOR_ID (40, SparkMax), INTAKE_EXTENSION_MOTOR_ID (41, Kraken/Canivore)
// [ ] 2. Confirm DIO port: RETRACT_LIMIT_SWITCH_DIO (confirmed DIO 1).
//        In Test mode, manually trip the switch and verify Intake/LimitSwitch reads true in AdvantageScope.
// [x] 3. Extension motor direction confirmed (EXTENSION_MOTOR_INVERTED = false).
//        Re-verify before SysId: positive voltage must = arm extending (away from home). If reversed, forward test hits home limit immediately.
// [x] 4. Roller motor direction confirmed (ROLLER_MOTOR_INVERTED = false)
// [x] 5. Position readings: HOME: 0.158  OUT: 12.95
// [ ] 6. Confirm limit switch trips cleanly at full retract — watch Intake/LimitSwitch in AdvantageScope
// [ ] 7. Run a full home + extend + retract cycle. Confirm encoder zeroes on home.
// [ ] 8. Confirm stall detection fires when arm is manually blocked mid-travel (must be done in normal teleop mode, not SysId mode — stall is bypassed during SysId).
// [ ] 9. Tune EXTEND_SPEED and RETRACT_SPEED for smooth travel without belt slip or slamming.
// [ ] 10. Measure actual gear ratio from mechanical team — changed from original, needed for SensorToMechanismRatio in MotionMagic.
// [ ] 11. Run SysId before enabling MotionMagic (call configureSysIdBindings in RobotContainer, run all 4 tests).
//         Results go into EXTENSION_kS, kV, kA in Constants. See sysIdQuasistatic/sysIdDynamic methods below.
public class IntakeSubsystem extends SubsystemBase {
  private final RobotState robotState;

  private final TalonFX extensionMotor;
  private final SparkMax rollerMotor; // REV NEO on SparkMax

  private final DigitalInput limitSwitch; // single switch at full retract (DIO 1)

  private final DutyCycleOut extensionOut = new DutyCycleOut(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0); // Used only during SysId

  // Pre-subscribed signals — avoids blocking CAN reads in periodic()
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> currentSignal;

  // SysId routine — characterizes kS, kV, kA for the extension motor.
  // Run before switching to MotionMagic. Results feed into Constants.Intake EXTENSION_kS/kV/kA.
  // TODO: Once MotionMagic is ready, set SensorToMechanismRatio to the confirmed gear ratio.
  private final SysIdRoutine sysIdRoutine;

  private boolean extensionStalled = false;
  private int stallLoopCount = 0;
  private static final int STALL_LOOP_THRESHOLD = 30; // ~600ms at 50Hz — long enough to ignore momentary ball compression

  // Set true during SysId tests to bypass stall detection (high current is expected during characterization)
  private boolean sysIdActive = false;

  public IntakeSubsystem(RobotState robotState) {
    this.robotState = robotState;

    extensionMotor = new TalonFX(Constants.Intake.INTAKE_EXTENSION_MOTOR_ID, new CANBus(Constants.Swerve.CAN_BUS_NAME));

    // Extension: Kraken X60 with 4:1 gear ratio, CTRE stator + supply limits
    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs extLimits = extensionConfig.CurrentLimits;
    extLimits.StatorCurrentLimit       = Constants.Intake.EXTENSION_STATOR_LIMIT_AMPS;
    extLimits.StatorCurrentLimitEnable = true;
    extLimits.SupplyCurrentLimit       = Constants.Intake.EXTENSION_SUPPLY_LIMIT_AMPS;
    extLimits.SupplyCurrentLimitEnable = true;
    MotorOutputConfigs extOutput = extensionConfig.MotorOutput;
    extOutput.Inverted = Constants.Intake.EXTENSION_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    extOutput.NeutralMode = NeutralModeValue.Brake;
    extensionMotor.getConfigurator().apply(extensionConfig);

    // Pre-subscribe signals at 50Hz so periodic() reads cached values without blocking CAN
    positionSignal = extensionMotor.getPosition();
    currentSignal  = extensionMotor.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50, positionSignal, currentSignal);
    extensionMotor.optimizeBusUtilization();

    // SysId: ramps 0→4V over 2s, logs state to CTRE SignalLogger (same format as drivetrain)
    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(2).per(Second),  // 2 V/s ramp — slow enough for a short-travel arm
            Volts.of(4),              // 4V max to stay well below stall
            null,                     // Default 10s timeout
            state -> SignalLogger.writeString("SysIdIntakeExtension_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> extensionMotor.setControl(voltageOut.withOutput(volts.in(Volts))),
            null,
            this
        )
    );

    // Roller: REV NEO on SparkMax - smart current limit only
    rollerMotor = new SparkMax(Constants.Intake.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.inverted(Constants.Intake.ROLLER_MOTOR_INVERTED);
    rollerConfig.smartCurrentLimit(Constants.Intake.ROLLER_CURRENT_LIMIT_AMPS);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    limitSwitch = new DigitalInput(Constants.Intake.RETRACT_LIMIT_SWITCH_DIO);

    // Start in a known safe state - do not move motors until commanded
    stopAll();
    SmartLogger.logConsole("IntakeSubsystem initialized - call startHoming() on enable", "Intake");
  }

  // Begin homing sequence - creeps upward until the limit switch trips.
  // Safe to call if already RETRACTED (no-op) or HOMING_FAILED (retries).
  public void startHoming() {
    IntakePosition pos = robotState.getIntakePosition();
    if (pos == IntakePosition.RETRACTED) return;
    if (!limitSwitch.get()) {
      // Already at home - just zero and mark retracted
      extensionMotor.setPosition(Constants.Intake.EXTENSION_HOME_ROTATIONS);
      robotState.setIntakePosition(IntakePosition.RETRACTED);
      SmartLogger.logConsole("Intake already at home on enable - zeroed", "Intake");
      return;
    }
    extensionStalled = false;
    stallLoopCount = 0;
    extensionMotor.setControl(extensionOut.withOutput(Constants.Intake.RETRACT_SPEED));
    robotState.setIntakePosition(IntakePosition.HOMING);
    SmartLogger.logConsole("Intake homing started - moving to limit switch", "Intake");
  }

  // Extend the intake arm out over the bumper.
  // Blocked until homing completes successfully.
  public void extend() {
    IntakePosition pos = robotState.getIntakePosition();
    if (pos == IntakePosition.HOMING || pos == IntakePosition.HOMING_FAILED) return;
    if (pos == IntakePosition.EXTENDED) return;
    extensionStalled = false;
    extensionMotor.setControl(extensionOut.withOutput(Constants.Intake.EXTEND_SPEED));
    robotState.setIntakePosition(IntakePosition.EXTENDING);
  }

  // Retract the intake arm back inside the frame.
  // Blocked until homing completes successfully.
  public void retract() {
    IntakePosition pos = robotState.getIntakePosition();
    if (pos == IntakePosition.HOMING || pos == IntakePosition.HOMING_FAILED) return;
    if (pos == IntakePosition.RETRACTED) return;
    extensionStalled = false;
    extensionMotor.setControl(extensionOut.withOutput(Constants.Intake.RETRACT_SPEED));
    robotState.setIntakePosition(IntakePosition.RETRACTING);
  }

  public void stopExtension() {
    extensionMotor.setControl(extensionOut.withOutput(0.0));
    // Keep position state as-is so callers know where we stopped
  }

  // Spin rollers to intake game pieces
  public void spinIn() {
    rollerMotor.set(Constants.Intake.ROLLER_INTAKE_SPEED);
    robotState.setIntakeRollerState(IntakeRollerState.INTAKING);
  }

  // Reverse rollers to eject
  public void spinOut() {
    rollerMotor.set(Constants.Intake.ROLLER_REVERSE_SPEED);
    robotState.setIntakeRollerState(IntakeRollerState.REVERSING);
  }

  public void stopRollers() {
    rollerMotor.set(0.0);
    robotState.setIntakeRollerState(IntakeRollerState.STOPPED);
  }

  public void stopAll() {
    stopExtension();
    stopRollers();
  }

  public boolean isExtensionStalled() { return extensionStalled; }

  // Returns true once homing has completed successfully
  public boolean isHomed() {
    IntakePosition pos = robotState.getIntakePosition();
    return pos != IntakePosition.HOMING && pos != IntakePosition.HOMING_FAILED;
  }

  // Returns true only when the arm is fully at the extended position (safe to run rollers)
  public boolean isExtended() {
    return robotState.getIntakePosition() == IntakePosition.EXTENDED;
  }

  // Clear stall flag after the operator has acknowledged and moved the arm clear
  public void clearStall() { extensionStalled = false; }

  // SysId commands — wire into configureSysIdBindings() in RobotContainer.
  // Run quasistatic fwd/rev first (slowly ramps voltage), then dynamic fwd/rev (step input).
  // Arm must be homed and at RETRACTED before running forward tests.
  // Stall detection is bypassed while running — high current is expected during characterization.
  // Position limits are enforced: forward stops at TARGET, reverse stops at HOME.
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    boolean forward = direction == SysIdRoutine.Direction.kForward;
    return sysIdRoutine.quasistatic(direction)
        .beforeStarting(() -> sysIdActive = true)
        .until(() -> forward
            ? positionSignal.getValueAsDouble() >= Constants.Intake.EXTENSION_TARGET_ROTATIONS
            : positionSignal.getValueAsDouble() <= Constants.Intake.EXTENSION_HOME_ROTATIONS)
        .finallyDo(() -> { sysIdActive = false; stopExtension(); });
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    boolean forward = direction == SysIdRoutine.Direction.kForward;
    return sysIdRoutine.dynamic(direction)
        .beforeStarting(() -> sysIdActive = true)
        .until(() -> forward
            ? positionSignal.getValueAsDouble() >= Constants.Intake.EXTENSION_TARGET_ROTATIONS
            : positionSignal.getValueAsDouble() <= Constants.Intake.EXTENSION_HOME_ROTATIONS)
        .finallyDo(() -> { sysIdActive = false; stopExtension(); });
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(positionSignal, currentSignal);
    double rotations   = positionSignal.getValueAsDouble();
    double currentAmps = currentSignal.getValueAsDouble();

    boolean atHome = !limitSwitch.get(); // active-low — true when switch is pressed
    robotState.setIntakeLimitSwitch(atHome);

    IntakePosition pos = robotState.getIntakePosition();

    // Homing: stop and zero as soon as the limit switch trips
    if (pos == IntakePosition.HOMING) {
      if (atHome) {
        stopExtension();
        extensionMotor.setPosition(Constants.Intake.EXTENSION_HOME_ROTATIONS);
        robotState.setIntakePosition(IntakePosition.RETRACTED);
        SmartLogger.logConsole("Intake homing complete - encoder zeroed", "Intake");
      }
    }

    // Normal retract: limit switch trips OR soft limit reached
    if (pos == IntakePosition.RETRACTING) {
      if (atHome || rotations <= Constants.Intake.EXTENSION_HOME_ROTATIONS) {
        stopExtension();
        extensionMotor.setPosition(Constants.Intake.EXTENSION_HOME_ROTATIONS);
        robotState.setIntakePosition(IntakePosition.RETRACTED);
      }
    }

    // Soft limit: stop extending at target position
    if (pos == IntakePosition.EXTENDING && rotations >= Constants.Intake.EXTENSION_TARGET_ROTATIONS) {
      stopExtension();
      robotState.setIntakePosition(IntakePosition.EXTENDED);
    }

    // Current-based stall detection — skipped during SysId (high current is expected there)
    if (!sysIdActive && (pos == IntakePosition.HOMING || pos == IntakePosition.EXTENDING || pos == IntakePosition.RETRACTING)) {
      if (currentAmps > Constants.Intake.EXTENSION_STALL_CURRENT_AMPS) {
        stallLoopCount++;
        if (stallLoopCount >= STALL_LOOP_THRESHOLD) {
          stopExtension();
          extensionStalled = true;
          stallLoopCount = 0;
          if (pos == IntakePosition.HOMING) {
            robotState.setIntakePosition(IntakePosition.HOMING_FAILED);
            SmartLogger.logConsole("Intake homing FAILED - stall before limit switch - check arm", "Intake");
          } else {
            SmartLogger.logConsole("Intake extension stalled - hard stop or jam", "Intake");
          }
        }
      } else {
        stallLoopCount = 0;
      }
    } else {
      stallLoopCount = 0;
    }

    SmartLogger.logReplay("Intake/ExtensionRotations", rotations);
    SmartLogger.logReplay("Intake/ExtensionCurrentAmps", currentAmps);
    SmartLogger.logReplay("Intake/LimitSwitch", atHome);
    SmartLogger.logReplay("Intake/Stalled", extensionStalled);
    SmartLogger.logReplay("Intake/Homed", isHomed());

    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Intake/ExtensionRotations", rotations);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Intake/Position", pos.toString());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Intake/Stalled", extensionStalled);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Intake/Homed", isHomed());
  }
}

