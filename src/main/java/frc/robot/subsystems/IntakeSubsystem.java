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
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.IntakePosition;
import frc.robot.RobotState.IntakeRollerState;
import frc.robot.util.SmartLogger;

// Over-bumper intake with two TalonFX arm motors and a SparkMax roller motor.
// Left motor (ID 41) runs forward to extend; right motor (ID 42) is negated (opposite mounting).
// A limit switch at full retract tells us when homing is complete and zeros the encoders.
// Stall detection stops the arm if current stays too high (hard stop or jam).
public class IntakeSubsystem extends SubsystemBase {
  private final RobotState robotState;

  private final TalonFX extensionMotorLeft;
  private final TalonFX extensionMotorRight;
  private final SparkMax rollerMotor;
  private final DigitalInput limitSwitch;

  private final DutyCycleOut extensionOut = new DutyCycleOut(0.0);

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> currentSignal;

  private boolean extensionStalled = false;
  private int stallLoopCount = 0;
  private static final int STALL_LOOP_THRESHOLD = 30; // ~600ms

  public IntakeSubsystem(RobotState robotState) {
    this.robotState = robotState;

    extensionMotorLeft  = new TalonFX(Constants.Intake.INTAKE_EXTENSION_MOTOR_ID_LEFT);
    extensionMotorRight = new TalonFX(Constants.Intake.INTAKE_EXTENSION_MOTOR_ID_RIGHT);

    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs limits = config.CurrentLimits;
    limits.StatorCurrentLimit       = Constants.Intake.EXTENSION_STATOR_LIMIT_AMPS;
    limits.StatorCurrentLimitEnable = true;
    limits.SupplyCurrentLimit       = Constants.Intake.EXTENSION_SUPPLY_LIMIT_AMPS;
    limits.SupplyCurrentLimitEnable = true;
    MotorOutputConfigs output = config.MotorOutput;
    output.Inverted    = InvertedValue.CounterClockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;
    extensionMotorLeft.getConfigurator().apply(config);

    TalonFXConfiguration configRight = new TalonFXConfiguration();
    configRight.CurrentLimits = limits;
    configRight.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
    configRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionMotorRight.getConfigurator().apply(configRight);

    positionSignal = extensionMotorLeft.getPosition();
    currentSignal  = extensionMotorLeft.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50, positionSignal, currentSignal);
    extensionMotorLeft.optimizeBusUtilization();
    extensionMotorRight.optimizeBusUtilization();

    rollerMotor = new SparkMax(Constants.Intake.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.inverted(Constants.Intake.ROLLER_MOTOR_INVERTED);
    rollerConfig.smartCurrentLimit(Constants.Intake.ROLLER_CURRENT_LIMIT_AMPS);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    limitSwitch = new DigitalInput(Constants.Intake.RETRACT_LIMIT_SWITCH_DIO);

    stopAll();
    startHoming();
  }

  // Retract until the limit switch trips, then zero the encoders
  public void startHoming() {
    extensionStalled = false;
    stallLoopCount = 0;
    setExtensionOutput(Constants.Intake.RETRACT_SPEED);
    robotState.setIntakePosition(IntakePosition.HOMING);
    SmartLogger.logConsole("Intake homing started", "Intake");
  }

  // Extend arm  blocked until homing completes
  public void extend() {
    if (robotState.getIntakePosition() == IntakePosition.HOMING
        || robotState.getIntakePosition() == IntakePosition.HOMING_FAILED) return;
    if (robotState.getIntakePosition() == IntakePosition.EXTENDED) return;
    extensionStalled = false;
    setExtensionOutput(Constants.Intake.EXTEND_SPEED);
    robotState.setIntakePosition(IntakePosition.EXTENDING);
  }

  // Extend and automatically start rollers when arm reaches full extension
  public void extendAndSpin() {
    extend();
  }

  // Alias for extend() — arm only, no rollers
  public void extendOnly() {
    extend();
  }

  // Retract arm  blocked until homing completes
  public void retract() {
    if (robotState.getIntakePosition() == IntakePosition.HOMING
        || robotState.getIntakePosition() == IntakePosition.HOMING_FAILED) return;
    if (robotState.getIntakePosition() == IntakePosition.RETRACTED) return;
    extensionStalled = false;
    stopRollers();
    setExtensionOutput(Constants.Intake.RETRACT_SPEED);
    robotState.setIntakePosition(IntakePosition.RETRACTING);
  }

  public void spinIn() {
    rollerMotor.set(Constants.Intake.ROLLER_INTAKE_SPEED);
    robotState.setIntakeRollerState(IntakeRollerState.INTAKING);
  }

  public void spinOut() {
    rollerMotor.set(Constants.Intake.ROLLER_REVERSE_SPEED);
    robotState.setIntakeRollerState(IntakeRollerState.REVERSING);
  }

  public void stopRollers() {
    rollerMotor.set(0.0);
    robotState.setIntakeRollerState(IntakeRollerState.STOPPED);
  }

  public void stopAll() {
    setExtensionOutput(0.0);
    stopRollers();
  }

  public boolean isExtended() {
    return robotState.getIntakePosition() == IntakePosition.EXTENDED;
  }

  public boolean isHomed() {
    IntakePosition pos = robotState.getIntakePosition();
    return pos != IntakePosition.HOMING && pos != IntakePosition.HOMING_FAILED;
  }

  // Left runs at output, right is negated (opposite mounting)
  private void setExtensionOutput(double output) {
    extensionMotorLeft.setControl(extensionOut.withOutput(output));
    extensionMotorRight.setControl(extensionOut.withOutput(-output));
  }

  private void zeroEncoders() {
    extensionMotorLeft.setPosition(Constants.Intake.EXTENSION_HOME_ROTATIONS);
    extensionMotorRight.setPosition(Constants.Intake.EXTENSION_HOME_ROTATIONS_RIGHT);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(positionSignal, currentSignal);
    double rotations   = positionSignal.getValueAsDouble();
    double currentAmps = currentSignal.getValueAsDouble();
    boolean switchRaw  = limitSwitch.get();
    boolean atHome = switchRaw
        && rotations <= Constants.Intake.EXTENSION_HOME_ROTATIONS + Constants.Intake.LIMIT_SWITCH_VALID_WINDOW_ROTATIONS;
    robotState.setIntakeLimitSwitch(atHome);

    IntakePosition pos = robotState.getIntakePosition();

    if (pos == IntakePosition.HOMING) {
      if (switchRaw) {
        setExtensionOutput(0.0);
        zeroEncoders();
        robotState.setIntakePosition(IntakePosition.RETRACTED);
        SmartLogger.logConsole("Intake homing complete", "Intake");
      } else {
        setExtensionOutput(Constants.Intake.RETRACT_SPEED);
      }
    }

    if (pos == IntakePosition.RETRACTING) {
      if (atHome || rotations <= Constants.Intake.EXTENSION_HOME_ROTATIONS) {
        setExtensionOutput(0.0);
        zeroEncoders();
        robotState.setIntakePosition(IntakePosition.RETRACTED);
      }
    }

    if (pos == IntakePosition.EXTENDING) {
      if (rotations >= Constants.Intake.EXTENSION_TARGET_ROTATIONS) {
        setExtensionOutput(0.0);
        robotState.setIntakePosition(IntakePosition.EXTENDED);
      }
    }

    // Stall detection  stop arm if current is too high for too long
    if (pos == IntakePosition.HOMING || pos == IntakePosition.EXTENDING || pos == IntakePosition.RETRACTING) {
      if (currentAmps > Constants.Intake.EXTENSION_STALL_CURRENT_AMPS) {
        stallLoopCount++;
        if (stallLoopCount >= STALL_LOOP_THRESHOLD) {
          setExtensionOutput(0.0);
          extensionStalled = true;
          stallLoopCount = 0;
          if (pos == IntakePosition.HOMING) {
            robotState.setIntakePosition(IntakePosition.HOMING_FAILED);
            SmartLogger.logConsoleError("Intake homing FAILED - stall detected");
          } else {
            SmartLogger.logConsole("Intake stalled - stopping arm", "Intake");
          }
        }
      } else {
        stallLoopCount = 0;
      }
    }

    SmartLogger.logReplay("Intake/PositionRotations", rotations);
    SmartLogger.logReplay("Intake/CurrentAmps", currentAmps);
    SmartLogger.logReplay("Intake/LimitSwitch", atHome);
    SmartLogger.logReplay("Intake/Stalled", extensionStalled);
    SmartLogger.logReplay("Intake/State", pos.toString());
    SmartLogger.logReplay("Intake/RollerCurrentAmps", rollerMotor.getOutputCurrent());
  }
}