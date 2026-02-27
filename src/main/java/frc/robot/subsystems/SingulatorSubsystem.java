// Singulator subsystem - feeds balls one at a time from the spindexer up through the turret
// into the flywheels. A beam break at the staging point tells us when a ball is present.
// Tracks total balls fed since boot via a falling-edge counter on the beam break.
//
// BALL COUNTER: counts on the falling edge (beam break clears after being blocked), meaning
// a ball has just exited the staging point heading toward the flywheels. The count never
// resets during a match — visible in AdvantageScope at RobotState/BallsFedCount.
//
// TODO - COMMISSIONING CHECKLIST (complete in order before enabling in RobotContainer):
// [ ] 1. Confirm CAN ID: MOTOR_ID (24) appears in REV Hardware Client and responds.
// [ ] 2. Confirm DIO port: BEAM_BREAK_DIO (28). In Test mode, block the sensor with your
//        hand and verify Singulator/BallPresent toggles true in AdvantageScope.
//        Note: beam break is normally-open — blocked = false from sensor = true in code.
// [ ] 3. Check motor direction: run spinFeed() at low speed, confirm balls move toward
//        the flywheels. If backwards, set MOTOR_INVERTED = true in Constants.
// [ ] 4. Tune FEED_SPEED to match the flywheel acceptance rate — too fast risks double-
//        feeding; too slow starves the shooter.
// [ ] 5. Run several balls through end-to-end and verify the counter increments once per
//        ball. Watch RobotState/BallsFedCount in AdvantageScope.
// [ ] 6. Test spinReverse() with a stuck ball to confirm it clears cleanly.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

public class SingulatorSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final SparkMax motor;
  private final DigitalInput beamBreak;

  // Track previous beam break state to detect falling edge (ball just passed)
  private boolean lastBeamBreakBlocked = false;

  // Rolling shot rate: track timestamps of the last few shots to compute balls/sec
  private static final int RATE_WINDOW = 5;
  private final double[] shotTimestamps = new double[RATE_WINDOW];
  private int shotTimestampIndex = 0;
  private int shotsSeen = 0;

  public SingulatorSubsystem(RobotState robotState) {
    this.robotState = robotState;

    motor = new SparkMax(Constants.Singulator.MOTOR_ID, MotorType.kBrushless);
    beamBreak = new DigitalInput(Constants.Singulator.BEAM_BREAK_DIO);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(Constants.Singulator.MOTOR_INVERTED);
    config.smartCurrentLimit(Constants.Singulator.CURRENT_LIMIT_AMPS);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartLogger.logConsole("Singulator ready (CAN " + Constants.Singulator.MOTOR_ID + ")", "Singulator");
  }

  // Start feeding balls toward the flywheels
  public void spinFeed() {
    if (robotState.getSingulatorState() == RobotState.SingulatorState.FEEDING) return;
    motor.set(Constants.Singulator.FEED_SPEED);
    robotState.setSingulatorState(RobotState.SingulatorState.FEEDING);
  }

  // Pause feeding — hold position, don't run motor
  public void pause() {
    if (robotState.getSingulatorState() == RobotState.SingulatorState.PAUSED) return;
    motor.set(0.0);
    robotState.setSingulatorState(RobotState.SingulatorState.PAUSED);
  }

  // Reverse to clear a jam
  public void spinReverse() {
    motor.set(Constants.Singulator.REVERSE_SPEED);
    robotState.setSingulatorState(RobotState.SingulatorState.REVERSING);
  }

  public void stopAll() { pause(); }

  // True when a ball is blocking the beam break sensor
  public boolean isBallPresent() {
    return !beamBreak.get(); // beam break is normally-open: false = blocked = ball present
  }

  @Override
  public void periodic() {
    boolean ballBlocked = isBallPresent();

    // Falling edge: beam clears after being blocked — ball has just exited toward flywheels
    if (!ballBlocked && lastBeamBreakBlocked) {
      robotState.incrementBallsFed();
      recordShot();
    }
    lastBeamBreakBlocked = ballBlocked;

    robotState.setSingulatorBeamBreak(ballBlocked);

    SmartLogger.logReplay("Singulator/BallPresent", ballBlocked);
    SmartLogger.logReplay("Singulator/CurrentAmps", motor.getOutputCurrent());
    SmartLogger.logReplay("Singulator/BallsFedCount", robotState.getBallsFedCount());

    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Singulator/BallsFed", robotState.getBallsFedCount());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Singulator/ShotRatePerSec", computeShotRate());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Singulator/BallPresent", ballBlocked);
  }

  private void recordShot() {
    shotTimestamps[shotTimestampIndex] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    shotTimestampIndex = (shotTimestampIndex + 1) % RATE_WINDOW;
    shotsSeen++;
  }

  // Returns rolling average shots/sec over the last RATE_WINDOW shots.
  private double computeShotRate() {
    if (shotsSeen < 2) return 0.0;
    int filled = Math.min(shotsSeen, RATE_WINDOW);
    int oldestIndex = (shotTimestampIndex - filled + RATE_WINDOW) % RATE_WINDOW;
    int newestIndex = (shotTimestampIndex - 1 + RATE_WINDOW) % RATE_WINDOW;
    double span = shotTimestamps[newestIndex] - shotTimestamps[oldestIndex];
    if (span <= 0.0) return 0.0;
    return (filled - 1) / span;
  }
}
