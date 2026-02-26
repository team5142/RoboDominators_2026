package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.MatchPhaseTracker.GamePhase;
import frc.robot.util.SmartLogger;

// Blinkin PWM LED controller. Note: PWM actuators are disabled by RoboRIO safety during robot-disabled
// mode, so LED patterns only take effect when the robot is enabled (teleop/auto).
public class LEDSubsystem extends SubsystemBase {
  private final Spark blinkin;
  private final RobotState robotState;
  private final Timer blinkTimer = new Timer();

  // Seconds per blink half-cycle (on -> off -> on = 2x this)
  private static final double BLINK_PERIOD_SEC = 0.4;

  public static class Pattern {
    public static final double BLUE   =  0.87;
    public static final double RED    =  0.61;
    public static final double ORANGE =  0.65;
    public static final double GOLD   =  0.67;
    public static final double GREEN  =  0.77;
    public static final double WHITE  =  0.93;
    public static final double LAVA   = -0.39;
    public static final double OFF    =  0.99;
  }

  public LEDSubsystem(RobotState robotState) {
    this.robotState = robotState;
    this.blinkin = new Spark(Constants.BLINKIN_PWM_PORT);
    blinkTimer.start();
    SmartLogger.logConsole("Blinkin LED initialized on PWM port " + Constants.BLINKIN_PWM_PORT, "LED");
  }

  // Manual override - use for testing only
  public void setPattern(double sparkValue) {
    blinkin.set(sparkValue);
  }

  @Override
  public void periodic() {
    blinkin.set(choosePattern());
  }

  // Returns the Blinkin value for the current game state.
  // Priority: disabled -> auto -> endgame -> turret-ready -> hub-last-5s -> hub-active -> last-5s -> teleop default
  private double choosePattern() {
    if (!DriverStation.isEnabled()) {
      return Pattern.LAVA; // PWM is off during disabled; this is a no-op but keeps state clean
    }

    if (DriverStation.isAutonomousEnabled()) {
      // Blink red/off during auto so drive team knows auto is running
      return blinkOn() ? Pattern.RED : Pattern.OFF;
    }

    // Teleop states
    GamePhase phase = robotState.getGamePhase();
    boolean hubActive = robotState.isHubActive();
    double secsLeft = robotState.getSecondsUntilPhaseEnd();

    if (phase == GamePhase.END_GAME) {
      return Pattern.GOLD;
    }

    if (hubActive) {
      if (secsLeft <= 5.0) {
        // Hub window closing - urgent blink blue/white
        return blinkOn() ? Pattern.BLUE : Pattern.WHITE;
      }
      // Hub is active and flywheel is commanded - ready to shoot
      if (robotState.getTurretFlywheelPercent() > 0.05) {
        return blinkOn() ? Pattern.GREEN : Pattern.BLUE;
      }
      return Pattern.BLUE;
    }

    if (secsLeft <= 5.0) {
      // Shift ending soon, hub not yet active - warn driver
      return blinkOn() ? Pattern.RED : Pattern.ORANGE;
    }

    // Default teleop: solid red (opponent hub active or neutral zone)
    return Pattern.RED;
  }

  // Returns true during the "on" half of the blink cycle
  private boolean blinkOn() {
    return (blinkTimer.get() % (BLINK_PERIOD_SEC * 2)) < BLINK_PERIOD_SEC;
  }
}