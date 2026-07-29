package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

/*
 * TASK 22a - Declare and Configure the Intake Motors
 * -----------------------------------------------------------------------
 * The Intake is an over-bumper arm that collects game pieces.
 * It has three motors:
 *   - Two TalonFX (Kraken) arm motors: left (ID 41) and right (ID 42)
 *     The right motor runs opposite to the left because it is mounted mirrored.
 *   - One SparkMax roller motor (ID 40) that spins to pull balls in.
 *
 * There is also a limit switch on a RoboRIO DIO port that tells us when
 * the arm is fully retracted.
 *
 * TalonFX motors use a different API than SparkMax - instead of motor.set(),
 * you create a DutyCycleOut request and apply it with motor.setControl().
 * A DutyCycleOut is already declared for you below as extensionOut.
 * To command a TalonFX: motor.setControl(extensionOut.withOutput(speed))
 *
 * The TalonFX also needs a TalonFXConfiguration applied via getConfigurator().apply().
 * Look at the constructor body pre-built below for the configuration structure.
 *
 * Steps:
 *   1. Add imports for SparkMax, MotorType, SparkMaxConfig, ResetMode, PersistMode.
 *   2. Declare private final fields:
 *        TalonFX extensionMotorLeft
 *        TalonFX extensionMotorRight
 *        SparkMax rollerMotor
 *        DigitalInput limitSwitch
 *   3. In the constructor, create all four with their CAN/DIO IDs from Constants.Intake.
 *   4. The TalonFX configuration is pre-built below - read through it, then call
 *      extensionMotorLeft.getConfigurator().apply(config) and the same for right.
 *   5. Configure the SparkMax roller with inverted and smartCurrentLimit.
 *   6. Set up BaseStatusSignal for the position and current signals (pre-built below).
 *
 * When done: compile and move to Task 23a.
 * -----------------------------------------------------------------------
 */

/*
 * TASK 23a - Add Arm and Roller Control Methods
 * -----------------------------------------------------------------------
 * The intake needs these methods:
 *
 *   extend()       - start moving the arm outward at Constants.Intake.EXTEND_SPEED
 *                    set RobotState.IntakePosition to EXTENDING
 *                    skip if state is HOMING or HOMING_FAILED (arm not ready)
 *                    skip if already EXTENDED
 *
 *   retract()      - start moving the arm inward at Constants.Intake.RETRACT_SPEED
 *                    set state to RETRACTING
 *                    same guards as extend()
 *                    also call stopRollers() on retract
 *
 *   spinIn()       - run rollers at Constants.Intake.ROLLER_INTAKE_SPEED, state INTAKING
 *   spinOut()      - run rollers at Constants.Intake.ROLLER_REVERSE_SPEED, state REVERSING
 *   stopRollers()  - stop roller motor, state STOPPED
 *   stopAll()      - stop both arm and rollers
 *
 *   isExtended()   - returns true if IntakePosition is EXTENDED
 *   isHomed()      - returns true if position is not HOMING or HOMING_FAILED
 *
 * To command a TalonFX arm motor use the private helper already written below:
 *   setExtensionOutput(double output)
 * It handles the left/right mirroring automatically.
 *
 * When done: compile and move to Task 23b in RobotContainer.java.
 * -----------------------------------------------------------------------
 */

// Over-bumper intake with two TalonFX arm motors and a SparkMax roller.
// The homing sequence and stall detection in periodic() are pre-built.
public class IntakeSubsystem extends SubsystemBase {

  private final RobotState robotState;

  private final DutyCycleOut extensionOut = new DutyCycleOut(0.0);

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> currentSignal;

  private boolean extensionStalled = false;
  private int stallLoopCount = 0;
  private static final int STALL_LOOP_THRESHOLD = 30;

  public IntakeSubsystem(RobotState robotState) {
    this.robotState = robotState;

    // TalonFX configuration for both arm motors
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs limits = config.CurrentLimits;
    limits.StatorCurrentLimit       = Constants.Intake.EXTENSION_STATOR_LIMIT_AMPS;
    limits.StatorCurrentLimitEnable = true;
    limits.SupplyCurrentLimit       = Constants.Intake.EXTENSION_SUPPLY_LIMIT_AMPS;
    limits.SupplyCurrentLimitEnable = true;
    MotorOutputConfigs output = config.MotorOutput;
    output.Inverted    = InvertedValue.CounterClockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    // TODO (Task 22): create extensionMotorLeft, extensionMotorRight, rollerMotor, limitSwitch
    // then apply config to both TalonFX motors and configure the SparkMax roller
    // then set up the status signals below

    // Status signals - read position and current from the left motor at 50Hz
    // Replace "null" with your actual extensionMotorLeft once declared
    positionSignal = null; // extensionMotorLeft.getPosition();
    currentSignal  = null; // extensionMotorLeft.getStatorCurrent();
    // BaseStatusSignal.setUpdateFrequencyForAll(50, positionSignal, currentSignal);
    // extensionMotorLeft.optimizeBusUtilization();
    // extensionMotorRight.optimizeBusUtilization();

    // Task 23 unlock: stopAll();
    // Task 19 unlock: startHoming();
  }

  // Retract until the limit switch trips, then zero encoders. Called automatically at boot.
  // This is pre-built - do not modify.
  public void startHoming() {
    extensionStalled = false;
    stallLoopCount = 0;
    setExtensionOutput(Constants.Intake.RETRACT_SPEED);
    // Task 19 unlock: robotState.setIntakePosition(RobotState.IntakePosition.HOMING);
    SmartLogger.logConsole("Intake homing started", "Intake");
  }

  // TODO (Task 23): add extend(), retract(), spinIn(), spinOut(), stopRollers(),
  //                 stopAll(), isExtended(), isHomed() here

  // Commands both arm motors - right is negated because it is mounted mirrored.
  // Use this inside extend(), retract(), and stopAll() instead of calling motors directly.
  private void setExtensionOutput(double output) {
    // TODO (Task 22): replace with actual motor calls once motors are declared
    // extensionMotorLeft.setControl(extensionOut.withOutput(output));
    // extensionMotorRight.setControl(extensionOut.withOutput(-output));
  }

  private void zeroEncoders() {
    // TODO (Task 22): uncomment once motors are declared
    // extensionMotorLeft.setPosition(Constants.Intake.EXTENSION_HOME_ROTATIONS);
    // extensionMotorRight.setPosition(Constants.Intake.EXTENSION_HOME_ROTATIONS_RIGHT);
  }

  // periodic() handles homing, position tracking, and stall detection.
  // This is pre-built - read through it to understand how it works.
  // Task 19 unlock: un-comment all the RobotState lines below once you add IntakePosition.
  @Override
  public void periodic() {
    if (positionSignal == null || currentSignal == null) return;
    BaseStatusSignal.refreshAll(positionSignal, currentSignal);
    double rotations   = positionSignal.getValueAsDouble();
    double currentAmps = currentSignal.getValueAsDouble();
    boolean switchRaw  = false; // TODO (Task 22): replace with limitSwitch.get()
    boolean atHome = switchRaw
        && rotations <= Constants.Intake.EXTENSION_HOME_ROTATIONS + Constants.Intake.LIMIT_SWITCH_VALID_WINDOW_ROTATIONS;
    // Task 19 unlock: robotState.setIntakeLimitSwitch(atHome);

    // Task 19 unlock: RobotState.IntakePosition pos = robotState.getIntakePosition();

    // Task 19 unlock: if (pos == RobotState.IntakePosition.HOMING) {
    //   if (switchRaw) {
    //     setExtensionOutput(0.0);
    //     zeroEncoders();
    //     robotState.setIntakePosition(RobotState.IntakePosition.RETRACTED);
    //     SmartLogger.logConsole("Intake homing complete", "Intake");
    //   } else {
    //     setExtensionOutput(Constants.Intake.RETRACT_SPEED);
    //   }
    // }

    // Task 19 unlock: if (pos == RobotState.IntakePosition.RETRACTING) {
    //   if (atHome || rotations <= Constants.Intake.EXTENSION_HOME_ROTATIONS) {
    //     setExtensionOutput(0.0);
    //     zeroEncoders();
    //     robotState.setIntakePosition(RobotState.IntakePosition.RETRACTED);
    //   }
    // }

    // Task 19 unlock: if (pos == RobotState.IntakePosition.EXTENDING) {
    //   if (rotations >= Constants.Intake.EXTENSION_TARGET_ROTATIONS) {
    //     setExtensionOutput(0.0);
    //     robotState.setIntakePosition(RobotState.IntakePosition.EXTENDED);
    //   }
    // }

    // Task 19 unlock: if (pos == RobotState.IntakePosition.HOMING || pos == RobotState.IntakePosition.EXTENDING
    //     || pos == RobotState.IntakePosition.RETRACTING) {
    if (currentAmps > Constants.Intake.EXTENSION_STALL_CURRENT_AMPS) {
      stallLoopCount++;
      if (stallLoopCount >= STALL_LOOP_THRESHOLD) {
        setExtensionOutput(0.0);
        extensionStalled = true;
        stallLoopCount = 0;
        // Task 19 unlock: if (pos == RobotState.IntakePosition.HOMING) {
        //   robotState.setIntakePosition(RobotState.IntakePosition.HOMING_FAILED);
        //   SmartLogger.logConsoleError("Intake homing FAILED - stall detected");
        // }
      }
    } else {
      stallLoopCount = 0;
    }
    // Task 19 unlock: }

    SmartLogger.logReplay("Intake/PositionRotations", rotations);
    SmartLogger.logReplay("Intake/CurrentAmps", currentAmps);
    SmartLogger.logReplay("Intake/LimitSwitch", atHome);
    SmartLogger.logReplay("Intake/Stalled", extensionStalled);
  }
}
