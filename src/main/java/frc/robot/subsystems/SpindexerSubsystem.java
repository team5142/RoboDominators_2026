// Spindexer subsystem - cone-shaped spinner that agitates balls and feeds them into the singulator.
// The wheel at the bottom directs balls into the singulator groove when spinning forward.
// The cone top acts as an agitator to prevent balls from clumping or sticking.
//
// AGITATOR LOGIC: while spinning forward, if velocity drops below STALL_VELOCITY_RPS for
// AGITATE_LOOP_THRESHOLD loops (~500ms), a short reverse pulse fires automatically to
// jostle stuck balls, then resumes forward. No command input needed.
//
// TODO - COMMISSIONING CHECKLIST (complete in order before enabling in RobotContainer):
// [x] 1. Confirm CAN ID: MOTOR_ID (27) appears in REV Hardware Client and responds.
// [ ] 2. Check motor direction: run spinForward() at low speed, confirm balls move toward
//        the singulator groove. If backwards, set MOTOR_INVERTED = true in Constants.
// [ ] 3. Tune FORWARD_SPEED so balls feed consistently without jamming the singulator.
// [ ] 4. Tune STALL_VELOCITY_RPS to a value that only fires when the cone is truly stuck
//        (not during normal load variation). Watch Spindexer/VelocityRpm in AdvantageScope.
// [ ] 5. Tune AGITATE_LOOP_THRESHOLD and AGITATE_PULSE_LOOPS so the reverse pulse is
//        long enough to free a stuck ball but short enough to not dump balls backward.
// [ ] 6. Verify current limit is not tripping during normal operation — raise
//        CURRENT_LIMIT_AMPS in Constants if the motor cuts out under full ball load.

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

public class SpindexerSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final SparkMax motor;

  // Agitator state: counts loops where velocity is too low while FORWARD
  private int stallLoopCount = 0;
  // Counts down how many loops remain in the active agitator reverse pulse
  private int agitateLoopsRemaining = 0;

  // SparkMax encoder reports RPM; convert threshold from RPS
  private static final double STALL_VELOCITY_RPM = Constants.Spindexer.STALL_VELOCITY_RPS * 60.0;

  public SpindexerSubsystem(RobotState robotState) {
    this.robotState = robotState;

    motor = new SparkMax(Constants.Spindexer.MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(Constants.Spindexer.MOTOR_INVERTED);
    config.smartCurrentLimit(Constants.Spindexer.CURRENT_LIMIT_AMPS);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartLogger.logConsole("Spindexer ready (CAN " + Constants.Spindexer.MOTOR_ID + ")", "Spindexer");
  }

  // Start spinning forward to feed balls into the singulator groove
  public void spinForward() {
    if (robotState.getSpindexerState() == RobotState.SpindexerState.FORWARD) return;
    stallLoopCount = 0;
    agitateLoopsRemaining = 0;
    motor.set(Constants.Spindexer.FORWARD_SPEED);
    robotState.setSpindexerState(RobotState.SpindexerState.FORWARD);
  }

  // Reverse the spindexer briefly to unjam a stuck ball manually
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
    double velocityRpm = motor.getEncoder().getVelocity(); // NEO encoder reports RPM
    RobotState.SpindexerState currentState = robotState.getSpindexerState();

    // Agitator auto-reverse: while FORWARD, watch for sustained low velocity (ball jam)
    if (currentState == RobotState.SpindexerState.FORWARD) {
      if (Math.abs(velocityRpm) < STALL_VELOCITY_RPM) {
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
    if (currentState == RobotState.SpindexerState.REVERSE && agitateLoopsRemaining > 0) {
      agitateLoopsRemaining--;
      if (agitateLoopsRemaining == 0) {
        motor.set(Constants.Spindexer.FORWARD_SPEED);
        robotState.setSpindexerState(RobotState.SpindexerState.FORWARD);
      }
    }

    SmartLogger.logReplay("Spindexer/VelocityRpm", velocityRpm);
    SmartLogger.logReplay("Spindexer/CurrentAmps", motor.getOutputCurrent());
    SmartLogger.logReplay("Spindexer/StallLoopCount", stallLoopCount);
  }
}
