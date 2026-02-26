// Spindexer subsystem - cone-shaped spinner that agitates balls and feeds them into the singulator.
// The wheel at the bottom directs balls into the singulator groove when spinning forward.
// The cone top acts as an agitator to prevent balls from clumping or sticking.
//
// AGITATOR LOGIC: while spinning forward, if velocity drops below STALL_VELOCITY_RPS for
// AGITATE_LOOP_THRESHOLD loops (~500ms), a short reverse pulse fires automatically to
// jostle stuck balls, then resumes forward. No command input needed.
//
// TODO - COMMISSIONING CHECKLIST (complete in order before enabling in RobotContainer):
// [ ] 1. Confirm CAN ID: MOTOR_ID (27) appears in TunerX and responds.
// [ ] 2. Check motor direction: run spinForward() at low speed, confirm balls move toward
//        the singulator groove. If backwards, set MOTOR_INVERTED = true in Constants.
// [ ] 3. Tune FORWARD_SPEED so balls feed consistently without jamming the singulator.
// [ ] 4. Tune STALL_VELOCITY_RPS to a value that only fires when the cone is truly stuck
//        (not during normal load variation). Watch Spindexer/VelocityRps in AdvantageScope.
// [ ] 5. Tune AGITATE_LOOP_THRESHOLD and AGITATE_PULSE_LOOPS so the reverse pulse is
//        long enough to free a stuck ball but short enough to not dump balls backward.
// [ ] 6. Verify supply current limit is not tripping during normal operation — raise
//        SUPPLY_LIMIT_AMPS if the motor cuts out under full ball load.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

public class SpindexerSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final TalonFX motor;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

  // Agitator state: counts loops where velocity is too low while FORWARD
  private int stallLoopCount = 0;
  // Counts down how many loops remain in the active agitator reverse pulse
  private int agitateLoopsRemaining = 0;

  public SpindexerSubsystem(RobotState robotState) {
    this.robotState = robotState;

    motor = new TalonFX(Constants.Spindexer.MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = Constants.Spindexer.STATOR_LIMIT_AMPS;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = Constants.Spindexer.SUPPLY_LIMIT_AMPS;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = Constants.Spindexer.MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput = motorOutput;

    motor.getConfigurator().apply(config);

    SmartLogger.logConsole("Spindexer ready (CAN " + Constants.Spindexer.MOTOR_ID + ")", "Spindexer");
  }

  // Start spinning forward to feed balls into the singulator groove
  public void spinForward() {
    if (robotState.getSpindexerState() == RobotState.SpindexerState.FORWARD) return;
    stallLoopCount = 0;
    agitateLoopsRemaining = 0;
    motor.setControl(dutyCycle.withOutput(Constants.Spindexer.FORWARD_SPEED));
    robotState.setSpindexerState(RobotState.SpindexerState.FORWARD);
  }

  // Reverse the spindexer briefly to unjam a stuck ball manually
  public void spinReverse() {
    agitateLoopsRemaining = 0;
    motor.setControl(dutyCycle.withOutput(Constants.Spindexer.REVERSE_SPEED));
    robotState.setSpindexerState(RobotState.SpindexerState.REVERSE);
  }

  public void stop() {
    stallLoopCount = 0;
    agitateLoopsRemaining = 0;
    motor.setControl(dutyCycle.withOutput(0.0));
    robotState.setSpindexerState(RobotState.SpindexerState.STOPPED);
  }

  public void stopAll() {
    stop();
  }

  @Override
  public void periodic() {
    double velocityRps = motor.getVelocity().getValueAsDouble();
    RobotState.SpindexerState currentState = robotState.getSpindexerState();

    // Agitator auto-reverse: while FORWARD, watch for sustained low velocity (ball jam)
    if (currentState == RobotState.SpindexerState.FORWARD) {
      if (Math.abs(velocityRps) < Constants.Spindexer.STALL_VELOCITY_RPS) {
        stallLoopCount++;
      } else {
        stallLoopCount = 0;
      }

      if (stallLoopCount >= Constants.Spindexer.AGITATE_LOOP_THRESHOLD) {
        // Kick off an agitator reverse pulse
        stallLoopCount = 0;
        agitateLoopsRemaining = Constants.Spindexer.AGITATE_PULSE_LOOPS;
        motor.setControl(dutyCycle.withOutput(Constants.Spindexer.REVERSE_SPEED));
        robotState.setSpindexerState(RobotState.SpindexerState.REVERSE);
      }
    }

    // After the reverse pulse expires, go back to forward
    if (currentState == RobotState.SpindexerState.REVERSE && agitateLoopsRemaining > 0) {
      agitateLoopsRemaining--;
      if (agitateLoopsRemaining == 0) {
        motor.setControl(dutyCycle.withOutput(Constants.Spindexer.FORWARD_SPEED));
        robotState.setSpindexerState(RobotState.SpindexerState.FORWARD);
      }
    }

    SmartLogger.logReplay("Spindexer/VelocityRps", velocityRps);
    SmartLogger.logReplay("Spindexer/CurrentAmps", motor.getStatorCurrent().getValueAsDouble());
    SmartLogger.logReplay("Spindexer/StallLoopCount", stallLoopCount);
  }
}
