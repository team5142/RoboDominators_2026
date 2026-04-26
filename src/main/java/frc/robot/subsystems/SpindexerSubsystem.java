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
public class SpindexerSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final SparkMax motor;

  private int stallLoopCount = 0;
  private int agitateLoopsRemaining = 0;

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
    RobotState.SpindexerState state = robotState.getSpindexerState();

    // While spinning forward, watch for sustained high current (ball jam).
    if (state == RobotState.SpindexerState.FORWARD) {
      if (currentAmps > Constants.Spindexer.LOAD_CURRENT_AMPS) {
        stallLoopCount++;
      } else {
        stallLoopCount = 0;
      }

      if (stallLoopCount >= Constants.Spindexer.AGITATE_LOOP_THRESHOLD) {
        stallLoopCount = 0;
        agitateLoopsRemaining = Constants.Spindexer.AGITATE_PULSE_LOOPS;
        motor.set(Constants.Spindexer.REVERSE_SPEED);
        robotState.setSpindexerState(RobotState.SpindexerState.REVERSE);
      }
    }

    // After the reverse pulse expires, go back to forward
    if (state == RobotState.SpindexerState.REVERSE && agitateLoopsRemaining > 0) {
      agitateLoopsRemaining--;
      if (agitateLoopsRemaining == 0) {
        motor.set(Constants.Spindexer.FORWARD_SPEED);
        robotState.setSpindexerState(RobotState.SpindexerState.FORWARD);
      }
    }

    SmartLogger.logReplay("Spindexer/VelocityRpm", motor.getEncoder().getVelocity());
    SmartLogger.logReplay("Spindexer/CurrentAmps", currentAmps);
  }
}