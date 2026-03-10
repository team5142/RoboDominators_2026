package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.SmartLogger;

// Runs continuously while auto shoot mode is active (set as singulator default command).
// State machine: feeds balls automatically when turret is on target.
// Handles dead zone recovery: reverse first, push through if reverse fails.
// Feed recovery: alternates spindexer reverse and intake agitate every 4s when no ball
// is seen at staging — gated by ENABLE_AUTO_FEED_RECOVERY in Constants.
//
// Does NOT activate itself — auto shoot mode must be toggled on externally.
public class AutoShootCommand extends Command {

  private enum State {
    IDLE,            // waiting for a ball at the staging beam break
    BALL_STAGED,     // ball present, waiting for on-target
    FIRING,          // singulator feeding, watching for ball to clear staging sensor
    WAIT_GAP,        // brief enforced gap after shot before allowing next ball
    CHECK_DEAD_ZONE, // check if ball got stuck above singulator
    RECOVER_REVERSE, // reversing singulator to pull stuck ball back
    RECOVER_PUSH,    // pushing forward to clear ball near flywheel nip point
    RECOVERY_PULSE,  // active feed recovery pulse (spindexer rev or intake agitate)
    JAM              // all recovery exhausted — pause and log, operator intervention needed
  }

  private final RobotState robotState;
  private final SingulatorSubsystem singulatorSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private State state = State.IDLE;
  private State stateAfterPulse = State.IDLE; // state to return to after a recovery pulse
  private boolean finalPulse = false; // when true, end auto mode after the pulse completes
  private final Timer stateTimer = new Timer();
  private final Timer noBallTimer = new Timer(); // tracks time with no ball at staging in IDLE

  public AutoShootCommand(
      RobotState robotState,
      SingulatorSubsystem singulatorSubsystem,
      SpindexerSubsystem spindexerSubsystem,
      TurretSubsystem turretSubsystem,
      IntakeSubsystem intakeSubsystem) {
    this.robotState = robotState;
    this.singulatorSubsystem = singulatorSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(singulatorSubsystem);
  }

  @Override
  public void initialize() {
    state = State.IDLE;
    stateTimer.restart();
    noBallTimer.restart();
    if (spindexerSubsystem != null) spindexerSubsystem.spinForward();
    SmartLogger.logConsole("->AUTOSHOOT: started", "AutoShoot");
  }

  @Override
  public void execute() {
    if (!robotState.isAutoShootMode()) return;

    boolean ballPresent   = singulatorSubsystem.isBallPresent();
    boolean deadZone      = singulatorSubsystem.isDeadZoneBallPresent();
    boolean onTarget      = turretSubsystem != null && turretSubsystem.isReadyToShoot();
    boolean pauseShooting = robotState.isAutoShootPaused() || robotState.isShotSuppressed();

    switch (state) {
      case IDLE:
        if (ballPresent) {
          noBallTimer.stop();
          noBallTimer.reset();
          if (spindexerSubsystem != null) spindexerSubsystem.stop();
          transitionTo(State.BALL_STAGED);
          break;
        }
        if (deadZone) {
          transitionTo(State.RECOVER_REVERSE);
          break;
        }
        // Feed recovery — only when enabled and no ball has arrived
        if (Constants.Singulator.ENABLE_AUTO_FEED_RECOVERY) {
          double noball = noBallTimer.get();
          if (noball >= Constants.Singulator.FEED_RECOVERY_GIVE_UP_SECS) {
            // Final attempt: intake agitate then stop auto mode
            SmartLogger.logConsole("->AUTOSHOOT: 32s no ball — final intake agitate then stopping", "AutoShoot");
            if (intakeSubsystem != null) intakeSubsystem.agitate();
            finalPulse = true;
            stateAfterPulse = State.IDLE;
            transitionTo(State.RECOVERY_PULSE);
          } else {
            // Fire a recovery action every FEED_RECOVERY_INTERVAL_SECS.
            // Even steps (4s, 12s, 20s, 28s) = spindexer reverse.
            // Odd steps  (8s, 16s, 24s)       = intake agitate.
            int step = (int)(noball / Constants.Singulator.FEED_RECOVERY_INTERVAL_SECS);
            double nextTrigger = (step + 1) * Constants.Singulator.FEED_RECOVERY_INTERVAL_SECS;
            if (noball >= nextTrigger - 0.02) { // within one loop of the trigger
              if (step % 2 == 0) {
                SmartLogger.logConsole("->AUTOSHOOT: feed recovery — spindexer reverse (step " + (step+1) + ")", "AutoShoot");
                if (spindexerSubsystem != null) spindexerSubsystem.spinReverse();
              } else {
                SmartLogger.logConsole("->AUTOSHOOT: feed recovery — intake agitate (step " + (step+1) + ")", "AutoShoot");
                if (intakeSubsystem != null) intakeSubsystem.agitate();
              }
              finalPulse = false;
              stateAfterPulse = State.IDLE;
              transitionTo(State.RECOVERY_PULSE);
            }
          }
        }
        break;

      case RECOVERY_PULSE:
        if (stateTimer.hasElapsed(Constants.Singulator.FEED_RECOVERY_PULSE_SECS)) {
          if (intakeSubsystem != null) intakeSubsystem.stopRollers();
          if (spindexerSubsystem != null) spindexerSubsystem.spinForward();
          if (finalPulse) {
            SmartLogger.logConsole("->AUTOSHOOT: feed recovery exhausted — waiting for ball or operator action", "AutoShoot");
            noBallTimer.stop(); // stop so no more recovery actions fire
          }
          transitionTo(stateAfterPulse);
        }
        break;

      case BALL_STAGED:
        if (!ballPresent) {
          // Ball disappeared without shooting — return to idle and restart no-ball timer
          if (spindexerSubsystem != null) spindexerSubsystem.spinForward();
          noBallTimer.restart();
          transitionTo(State.IDLE);
          break;
        }
        if (!pauseShooting && onTarget) {
          singulatorSubsystem.primeAndFeed();
          transitionTo(State.FIRING);
        }
        break;

      case FIRING:
        // Wait for ball to clear the staging beam break (falling edge = ball exited)
        if (!ballPresent) {
          singulatorSubsystem.pause();
          transitionTo(State.WAIT_GAP);
        }
        break;

      case WAIT_GAP:
        if (stateTimer.hasElapsed(Constants.Singulator.AUTO_SHOT_GAP_SECS)) {
          transitionTo(State.CHECK_DEAD_ZONE);
        }
        break;

      case CHECK_DEAD_ZONE:
        if (deadZone) {
          SmartLogger.logConsole("->AUTOSHOOT: dead zone ball detected, reversing", "AutoShoot");
          singulatorSubsystem.spinReverse();
          transitionTo(State.RECOVER_REVERSE);
        } else {
          if (spindexerSubsystem != null) spindexerSubsystem.spinForward();
          noBallTimer.restart();
          transitionTo(State.IDLE);
        }
        break;

      case RECOVER_REVERSE:
        if (stateTimer.hasElapsed(Constants.Singulator.DEAD_ZONE_REVERSE_SECS)) {
          singulatorSubsystem.pause();
          if (ballPresent) {
            SmartLogger.logConsole("->AUTOSHOOT: ball recovered to staging", "AutoShoot");
            transitionTo(State.BALL_STAGED);
          } else {
            SmartLogger.logConsole("->AUTOSHOOT: reverse failed, pushing through", "AutoShoot");
            singulatorSubsystem.spinFeed();
            transitionTo(State.RECOVER_PUSH);
          }
        }
        break;

      case RECOVER_PUSH:
        if (!deadZone) {
          singulatorSubsystem.pause();
          SmartLogger.logConsole("->AUTOSHOOT: dead zone cleared by push", "AutoShoot");
          if (spindexerSubsystem != null) spindexerSubsystem.spinForward();
          noBallTimer.restart();
          transitionTo(State.IDLE);
        } else if (stateTimer.hasElapsed(Constants.Singulator.DEAD_ZONE_PUSH_TIMEOUT_SECS)) {
          singulatorSubsystem.pause();
          SmartLogger.logConsole("->AUTOSHOOT: JAM — dead zone push failed, operator needed", "AutoShoot");
          transitionTo(State.JAM);
        }
        break;

      case JAM:
        // Hold paused until operator intervenes or auto mode is toggled off
        break;
    }

    SmartLogger.logReplay("AutoShoot/State", state.toString());
    SmartLogger.logReplay("AutoShoot/NoBallTimerSecs", noBallTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    singulatorSubsystem.pause();
    if (spindexerSubsystem != null) spindexerSubsystem.stop();
    if (intakeSubsystem != null) intakeSubsystem.stopRollers();
    SmartLogger.logConsole("->AUTOSHOOT: ended (interrupted=" + interrupted + ")", "AutoShoot");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void transitionTo(State next) {
    state = next;
    stateTimer.restart();
  }
}

