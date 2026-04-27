package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

// Cone-shaped spinner that agitates balls and feeds them toward the singulator.
// While spinning forward, if current stays high for too long (ball jam), a short
// reverse pulse fires automatically to relieve pressure, then resumes forward.
//
// === STUDENT EXERCISE ===
// Your job is to implement the jam detection logic inside periodic().
// The motor exposes its current draw via motor.getOutputCurrent().
// When a ball is jammed, current rises and stays high for many loops in a row.
// When enough high-current loops accumulate, fire a short reverse pulse to clear it.
//
// Variables already provided for you:
//   stallLoopCount        - counts consecutive high-current loops
//   agitateLoopsRemaining - counts how many loops the reverse pulse has left
//
// Constants from Constants.Spindexer you will need:
//   LOAD_CURRENT_AMPS       - current threshold that indicates a jam
//   AGITATE_LOOP_THRESHOLD  - how many consecutive high-current loops trigger a pulse
//   AGITATE_PULSE_LOOPS     - how many loops the reverse pulse lasts
//   FORWARD_SPEED           - motor output for forward spin
//   REVERSE_SPEED           - motor output for reverse spin
// ========================
public class SpindexerSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final SparkMax motor;

  @SuppressWarnings("unused") private int stallLoopCount = 0;
  @SuppressWarnings("unused") private int agitateLoopsRemaining = 0;

  public SpindexerSubsystem(RobotState robotState) {
    this.robotState = robotState;

    motor = new SparkMax(Constants.Spindexer.MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(Constants.Spindexer.MOTOR_INVERTED);
    config.smartCurrentLimit(Constants.Spindexer.CURRENT_LIMIT_AMPS);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartLogger.logConsole("Spindexer ready (CAN " + Constants.Spindexer.MOTOR_ID + ")", "Spindexer");
  }

  public void spinForward() {
    if (robotState.getSpindexerState() == RobotState.SpindexerState.FORWARD) return;
    stallLoopCount = 0;
    agitateLoopsRemaining = 0;
    motor.set(Constants.Spindexer.FORWARD_SPEED);
    robotState.setSpindexerState(RobotState.SpindexerState.FORWARD);
  }

  public void spinReverse() {
    agitateLoopsRemaining = 0;
    motor.set(Constants.Spindexer.REVERSE_SPEED);
    robotState.setSpindexerState(RobotState.SpindexerState.REVERSE);
  }

  public void stop() {
    stallLoopCount = 0;
    agitateLoopsRemaining = 0;
    motor.set(0.0);
    robotState.setSpindexerState(RobotState.SpindexerState.STOPPED);
  }

  public void stopAll() { stop(); }

  @Override
  public void periodic() {
    double currentAmps = motor.getOutputCurrent();
    @SuppressWarnings("unused")
    RobotState.SpindexerState state = robotState.getSpindexerState();

    // TODO: Implement jam detection while the spindexer is spinning FORWARD.
    //
    // Part 1 - Detect a jam:
    //   If current is above LOAD_CURRENT_AMPS, increment stallLoopCount.
    //   If current is normal, reset stallLoopCount to 0.
    //   When stallLoopCount reaches AGITATE_LOOP_THRESHOLD:
    //     - Reset stallLoopCount to 0
    //     - Set agitateLoopsRemaining to AGITATE_PULSE_LOOPS
    //     - Set motor output to REVERSE_SPEED
    //     - Update robotState to REVERSE

    // TODO: Part 2 - End the reverse pulse.
    //   While state is REVERSE and agitateLoopsRemaining > 0:
    //     - Decrement agitateLoopsRemaining each loop
    //     - When it reaches 0, set motor back to FORWARD_SPEED
    //     - Update robotState to FORWARD

    SmartLogger.logReplay("Spindexer/VelocityRpm", motor.getEncoder().getVelocity());
    SmartLogger.logReplay("Spindexer/CurrentAmps", currentAmps);
  }
}