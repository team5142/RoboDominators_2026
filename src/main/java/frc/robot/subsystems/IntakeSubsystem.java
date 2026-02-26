package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.IntakePosition;
import frc.robot.RobotState.IntakeRollerState;
import frc.robot.util.SmartLogger;

// Over-bumper intake - one motor extends/retracts the arm, one motor spins the rollers.
// The arm is homed against two limit switches at full retract (encoder zeroed there).
// No limit switch at full extension - stops at a fixed rotation target (tune in TunerX).
//
// STALL PROTECTION: velocity-based. If the motor is commanded but stays near zero for
// ~200ms, it stops and sets extensionStalled=true (likely a ball jammed under the arm).
// Clear by calling stopExtension(), then clear the flag with clearStall(), then re-command.
//
// TODO - COMMISSIONING CHECKLIST (complete in order before enabling in RobotContainer):
// [ ] 1. Confirm CAN IDs: INTAKE_ROLLER_MOTOR_ID (40) and INTAKE_EXTENSION_MOTOR_ID (41)
//        in TunerX - verify both motors appear and respond.
// [ ] 2. Confirm DIO ports: RETRACT_LIMIT_SWITCH_A_DIO and _B_DIO (currently 0 and 1).
//        In Test mode, manually trip each switch and verify the DigitalInput reads true.
// [ ] 3. Check extension motor direction: command a small extend (hold test button).
//        If the arm retracts instead, set EXTENSION_MOTOR_INVERTED = true in Constants.
// [ ] 4. Check roller motor direction: command spinIn().
//        If it ejects instead, set ROLLER_MOTOR_INVERTED = true in Constants.
// [ ] 5. Manually extend arm to full out position, read motor rotation in TunerX,
//        update EXTENSION_TARGET_ROTATIONS in Constants.
// [ ] 6. Verify limit switches both trip cleanly at full retract (watch Intake/LimitA
//        and Intake/LimitB in AdvantageScope or SmartDashboard).
// [ ] 7. Run a full extend + retract cycle with the test bindings in RobotContainer.
//        Confirm it stops at the right places and the encoder zeroes on retract home.
// [ ] 8. Confirm stall detection fires when you manually block the arm mid-travel.
//        Watch Intake/Stalled in AdvantageScope.
// [ ] 9. Tune EXTEND_SPEED and RETRACT_SPEED so the arm moves smoothly without slamming.
// [ ] 10. Once gear ratio is known, replace DutyCycleOut with MotionMagicVoltage for
//         smooth deceleration near the target (see TODO in periodic()).
public class IntakeSubsystem extends SubsystemBase {
  private final RobotState robotState;

  private final TalonFX extensionMotor;
  private final TalonFX rollerMotor;

  private final DigitalInput limitSwitchA;
  private final DigitalInput limitSwitchB;

  private final DutyCycleOut extensionOut = new DutyCycleOut(0.0);
  private final DutyCycleOut rollerOut    = new DutyCycleOut(0.0);

  // Tracks whether a stall was detected so the caller can respond
  private boolean extensionStalled = false;
  // Consecutive loops at near-zero velocity while moving before declaring a stall
  private int stallLoopCount = 0;
  private static final int STALL_LOOP_THRESHOLD = 10; // ~200ms at 50Hz

  public IntakeSubsystem(RobotState robotState) {
    this.robotState = robotState;

    extensionMotor = new TalonFX(Constants.Intake.INTAKE_EXTENSION_MOTOR_ID);
    rollerMotor    = new TalonFX(Constants.Intake.INTAKE_ROLLER_MOTOR_ID);

    // Stator limit caps motor torque current (prevents overheating and aids stall detection).
    // Supply limit caps battery draw (primary brownout protection).
    // Extension is a slow positioning motor - conservative limits are fine.
    // Roller sees short current peaks on ball contact, so limits are a bit higher.
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
    extensionMotor.getConfigurator().apply(extensionConfig);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs rollerLimits = rollerConfig.CurrentLimits;
    rollerLimits.StatorCurrentLimit       = Constants.Intake.ROLLER_STATOR_LIMIT_AMPS;
    rollerLimits.StatorCurrentLimitEnable = true;
    rollerLimits.SupplyCurrentLimit       = Constants.Intake.ROLLER_SUPPLY_LIMIT_AMPS;
    rollerLimits.SupplyCurrentLimitEnable = true;
    MotorOutputConfigs rollerOutput = rollerConfig.MotorOutput;
    rollerOutput.Inverted = Constants.Intake.ROLLER_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    rollerMotor.getConfigurator().apply(rollerConfig);

    limitSwitchA = new DigitalInput(Constants.Intake.RETRACT_LIMIT_SWITCH_A_DIO);
    limitSwitchB = new DigitalInput(Constants.Intake.RETRACT_LIMIT_SWITCH_B_DIO);

    // Start in a known safe state - do not move motors until commanded
    stopAll();
    SmartLogger.logConsole("IntakeSubsystem initialized", "Intake");
  }

  // Extend the intake arm out over the bumper.
  // Stops automatically once the extension motor reaches the target rotation (see periodic).
  public void extend() {
    if (robotState.getIntakePosition() == IntakePosition.EXTENDED) return;
    extensionStalled = false;
    extensionMotor.setControl(extensionOut.withOutput(Constants.Intake.EXTEND_SPEED));
    robotState.setIntakePosition(IntakePosition.EXTENDING);
  }

  // Retract the intake arm back inside the frame.
  // Stops automatically when both limit switches trip (see periodic).
  public void retract() {
    if (robotState.getIntakePosition() == IntakePosition.RETRACTED) return;
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
    rollerMotor.setControl(rollerOut.withOutput(Constants.Intake.ROLLER_INTAKE_SPEED));
    robotState.setIntakeRollerState(IntakeRollerState.INTAKING);
  }

  // Reverse rollers to eject
  public void spinOut() {
    rollerMotor.setControl(rollerOut.withOutput(Constants.Intake.ROLLER_REVERSE_SPEED));
    robotState.setIntakeRollerState(IntakeRollerState.REVERSING);
  }

  public void stopRollers() {
    rollerMotor.setControl(rollerOut.withOutput(0.0));
    robotState.setIntakeRollerState(IntakeRollerState.STOPPED);
  }

  public void stopAll() {
    stopExtension();
    stopRollers();
  }

  public boolean isExtensionStalled() { return extensionStalled; }

  // Returns true only when the arm is fully at the extended position (safe to run rollers)
  public boolean isExtended() {
    return robotState.getIntakePosition() == IntakePosition.EXTENDED;
  }

  // Clear stall flag after the operator has acknowledged and moved the arm clear
  public void clearStall() { extensionStalled = false; }

  @Override
  public void periodic() {
    boolean a = !limitSwitchA.get(); // DigitalInput returns false when switch is closed
    boolean b = !limitSwitchB.get();
    robotState.setIntakeLimitSwitches(a, b);

    IntakePosition pos = robotState.getIntakePosition();

    // Auto-stop retraction when both limit switches trip; zero the encoder here so
    // extension target rotations are always measured from the retracted home position.
    if (pos == IntakePosition.RETRACTING && a && b) {
      stopExtension();
      extensionMotor.setPosition(0.0);
      robotState.setIntakePosition(IntakePosition.RETRACTED);
    }

    // Auto-stop extension when the motor reaches the target rotation.
    // TODO: replace with MotionMagic position control once gear ratio is confirmed in TunerX.
    if (pos == IntakePosition.EXTENDING) {
      double rotations = extensionMotor.getPosition().getValueAsDouble();
      if (rotations >= Constants.Intake.EXTENSION_TARGET_ROTATIONS) {
        stopExtension();
        robotState.setIntakePosition(IntakePosition.EXTENDED);
      }
    }

    // Stall detection: motor commanded but velocity stays near zero for several loops.
    // Current-based detection is unreliable here because the stator limit (20A) clamps
    // current before it can spike high enough to distinguish a stall from normal load.
    // Velocity going to zero while commanded is a much more reliable indicator.
    if (pos == IntakePosition.EXTENDING || pos == IntakePosition.RETRACTING) {
      double velocityRps = Math.abs(extensionMotor.getVelocity().getValueAsDouble());
      if (velocityRps < Constants.Intake.EXTENSION_STALL_VELOCITY_RPS) {
        stallLoopCount++;
        if (stallLoopCount >= STALL_LOOP_THRESHOLD) {
          stopExtension();
          extensionStalled = true;
          stallLoopCount = 0;
          SmartLogger.logConsole("Intake extension stalled (vel near zero) - possible ball jam", "Intake");
        }
      } else {
        stallLoopCount = 0;
      }
    } else {
      stallLoopCount = 0;
    }

    SmartLogger.logReplay("Intake/ExtensionRotations",
      extensionMotor.getPosition().getValueAsDouble());
    SmartLogger.logReplay("Intake/ExtensionCurrentAmps",
      extensionMotor.getSupplyCurrent().getValueAsDouble());
    SmartLogger.logReplay("Intake/LimitA", a);
    SmartLogger.logReplay("Intake/LimitB", b);
    SmartLogger.logReplay("Intake/Stalled", extensionStalled);

    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Intake/ExtensionRotations", extensionMotor.getPosition().getValueAsDouble());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(
        "Intake/Position", robotState.getIntakePosition().toString());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Intake/Stalled", extensionStalled);
  }
}

