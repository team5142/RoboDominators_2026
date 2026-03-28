// Singulator subsystem - feeds balls one at a time from the spindexer up through the turret
// into the flywheels. Two LaserCANs tell us ball position:
//   CAN 28 — staging beam break: ball present and ready to feed
//   CAN 29 — dead zone beam break: ball stuck above singulator, below flywheels
// Tracks total balls fed since boot via direction-aware edge detection on the sensor.
//
// BALL COUNTER: increments on the falling edge only while FEEDING (ball exits toward flywheels).
// Decrements on the rising edge only while REVERSING (ball pulled back past sensor).
// This keeps the count accurate even when reverse-feeding to clear a jam.
// Count is visible in AdvantageScope at RobotState/BallsFedCount.
//
// TODO - COMMISSIONING CHECKLIST (complete in order before enabling in RobotContainer):
// [x] 1. Confirm CAN ID: MOTOR_ID (24) appears in REV Hardware Client and responds.
// [ ] 2. Confirm LaserCAN (CAN 28) appears on CAN bus and reports valid measurements.
//        In Test mode, block the sensor with your hand and verify Singulator/BallPresent
//        toggles true in AdvantageScope. Threshold is LASERCAN_THRESHOLD_MM in Constants.
// [ ] 3. Confirm dead zone LaserCAN (CAN 29) appears on CAN bus and reports valid measurements.
//        Block with hand and verify Singulator/DeadZoneBallPresent toggles true.
// [x] 4. Check motor direction: run spinFeed() at low speed, confirm balls move toward
//        the flywheels. If backwards, set MOTOR_INVERTED = true in Constants.
// [ ] 5. Tune FEED_SPEED to match the flywheel acceptance rate — too fast risks double-
//        feeding; too slow starves the shooter.
// [ ] 6. Run several balls through end-to-end and verify the counter increments once per
//        ball. Watch RobotState/BallsFedCount in AdvantageScope.
// [x] 7. Test spinReverse() with a stuck ball to confirm it clears cleanly.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

public class SingulatorSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final SparkMax motor;
  private final LaserCan laserCan;
  private final LaserCan deadZoneLaserCan;

  // Track previous beam break state to detect falling edge (ball just passed)
  private boolean lastBeamBreakBlocked = false;

  // Timer used by primeAndFeed() to hold the reverse pulse before switching to feed
  private final Timer primeTimer = new Timer();
  private boolean priming = false;

  // Dead zone recovery: if a ball is stuck in the dead zone while feeding, reverse once to kick it loose.
  // Only one attempt per continuous blockage — if it fails, resume feeding and wait for operator to retry.
  private final Timer deadZoneTimer = new Timer();
  private boolean deadZoneTimerRunning = false;
  private boolean deadZoneRecoveryActive = false;
  private boolean deadZoneRecoveryAttempted = false; // one attempt per blockage event

  // Rolling shot rate: track timestamps of the last few shots to compute balls/sec
  private static final int RATE_WINDOW = 5;
  private final double[] shotTimestamps = new double[RATE_WINDOW];
  private int shotTimestampIndex = 0;
  private int shotsSeen = 0;

  public SingulatorSubsystem(RobotState robotState) {
    this.robotState = robotState;

    motor = new SparkMax(Constants.Singulator.MOTOR_ID, MotorType.kBrushless);

    laserCan = new LaserCan(Constants.Singulator.LASERCAN_ID);
    deadZoneLaserCan = new LaserCan(Constants.Singulator.DEAD_ZONE_LASERCAN_ID);
    try {
      // SHORT mode: better ambient light rejection at close range (<1.3m).
      // 33ms budget: updates ~every 2 robot loops, fast enough to catch any ball passing through.
      // 4x4 ROI centered at 8,8: narrows the beam to ~18mm at 150mm — sees ball, ignores channel walls.
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 8, 8)); // wider ROI — catches fast balls
      deadZoneLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      deadZoneLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      deadZoneLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
    } catch (au.grapplerobotics.ConfigurationFailedException e) {
      SmartLogger.logConsole("LaserCAN config failed: " + e.getMessage(), "Singulator");
    }

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(Constants.Singulator.MOTOR_INVERTED);
    config.smartCurrentLimit(Constants.Singulator.CURRENT_LIMIT_AMPS);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartLogger.logConsole("Singulator ready (CAN " + Constants.Singulator.MOTOR_ID + ")", "Singulator");
  }

  // Start feeding balls toward the flywheels
  public void spinFeed() {
    if (robotState.getSingulatorState() == RobotState.SingulatorState.FEEDING) return;
    priming = false;
    motor.set(Constants.Singulator.FEED_SPEED);
    robotState.setSingulatorState(RobotState.SingulatorState.FEEDING);
  }

  // Reverse briefly to pull the ball back into compression, then transition to feed.
  // Call this instead of spinFeed() for normal shooting — the deadzone at the turret
  // plane means a ball sitting there won't have enough traction on a cold forward start.
  // periodic() watches the primeTimer and calls spinFeed() once the pulse expires.
  public void primeAndFeed() {
    if (priming || robotState.getSingulatorState() == RobotState.SingulatorState.FEEDING) return;
    priming = true;
    primeTimer.restart();
    motor.set(Constants.Singulator.REVERSE_SPEED);
    robotState.setSingulatorState(RobotState.SingulatorState.REVERSING);
  }

  // Pause feeding — hold position, don't run motor
  public void pause() {
    if (robotState.getSingulatorState() == RobotState.SingulatorState.PAUSED) return;
    priming = false;
    // Reset dead zone recovery so the next RT press starts clean
    deadZoneRecoveryActive = false;
    deadZoneRecoveryAttempted = false;
    deadZoneTimerRunning = false;
    deadZoneTimer.stop();
    motor.set(0.0);
    robotState.setSingulatorState(RobotState.SingulatorState.PAUSED);
  }

  // Reverse to clear a jam
  public void spinReverse() {
    motor.set(Constants.Singulator.REVERSE_SPEED);
    robotState.setSingulatorState(RobotState.SingulatorState.REVERSING);
  }

  public void stopAll() { pause(); }

  // True when a ball is close enough to block the staging LaserCAN beam
  public boolean isBallPresent() {
    LaserCan.Measurement m = laserCan.getMeasurement();
    return m != null
        && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && m.distance_mm <= Constants.Singulator.LASERCAN_THRESHOLD_MM;
  }

  // True when a ball is stuck in the dead zone above the singulator, below the flywheels
  public boolean isDeadZoneBallPresent() {
    LaserCan.Measurement m = deadZoneLaserCan.getMeasurement();
    return m != null
        && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && m.distance_mm <= Constants.Singulator.DEAD_ZONE_LASERCAN_THRESHOLD_MM;
  }

  @Override
  public void periodic() {
    // If we're in the prime pulse, check if time has elapsed and transition to feed
    if (priming && primeTimer.hasElapsed(Constants.Singulator.PRIME_REVERSE_SECS)) {
      priming = false;
      motor.set(Constants.Singulator.FEED_SPEED);
      robotState.setSingulatorState(RobotState.SingulatorState.FEEDING);
    }

    boolean ballBlocked = isBallPresent();
    RobotState.SingulatorState singState = robotState.getSingulatorState();

    // Dead zone recovery: only while actively feeding, one attempt per continuous blockage.
    // If the ball is still stuck after the reverse pulse completes, give up and resume feeding.
    // Operator must release and re-press RT to trigger another attempt.
    boolean deadZoneBlocked = isDeadZoneBallPresent();
    if (singState == RobotState.SingulatorState.FEEDING && !deadZoneRecoveryActive) {
      if (deadZoneBlocked && !deadZoneRecoveryAttempted) {
        if (!deadZoneTimerRunning) {
          deadZoneTimer.restart();
          deadZoneTimerRunning = true;
        } else if (deadZoneTimer.hasElapsed(Constants.Singulator.DEAD_ZONE_DETECT_SECS)) {
          // Ball stuck for 1+ seconds — fire one reverse pulse
          deadZoneRecoveryActive = true;
          deadZoneRecoveryAttempted = true;
          deadZoneTimerRunning = false;
          priming = false;
          motor.set(Constants.Singulator.REVERSE_SPEED);
          robotState.setSingulatorState(RobotState.SingulatorState.REVERSING);
          SmartLogger.logConsole("Dead zone ball detected — reverse pulse", "Singulator");
        }
      } else if (!deadZoneBlocked) {
        // Ball cleared — reset so a future new blockage can trigger recovery again
        deadZoneTimerRunning = false;
        deadZoneRecoveryAttempted = false;
        deadZoneTimer.stop();
      }
    }

    // Recovery pulse complete — resume feeding regardless of whether ball cleared
    if (deadZoneRecoveryActive && deadZoneTimer.hasElapsed(Constants.Singulator.DEAD_ZONE_REVERSE_SECS)) {
      deadZoneRecoveryActive = false;
      deadZoneTimerRunning = false;
      motor.set(Constants.Singulator.FEED_SPEED);
      robotState.setSingulatorState(RobotState.SingulatorState.FEEDING);
      SmartLogger.logConsole("Dead zone recovery complete — resuming feed", "Singulator");
    }

    SmartLogger.logReplay("Singulator/DeadZoneRecoveryActive", deadZoneRecoveryActive);
    if (!ballBlocked && lastBeamBreakBlocked
        && singState == RobotState.SingulatorState.FEEDING) {
      robotState.incrementBallsFed();
      recordShot();
    }
    // Rising edge while reversing: ball pulled back past sensor, undo the count
    if (ballBlocked && !lastBeamBreakBlocked
        && singState == RobotState.SingulatorState.REVERSING) {
      robotState.decrementBallsFed();
    }
    lastBeamBreakBlocked = ballBlocked;

    robotState.setSingulatorBeamBreak(ballBlocked);

    robotState.setDeadZoneBeamBreak(deadZoneBlocked);

    SmartLogger.logReplay("Singulator/BallPresent", ballBlocked);
    SmartLogger.logReplay("Singulator/DeadZoneBallPresent", deadZoneBlocked);
    SmartLogger.logReplay("Singulator/CurrentAmps", motor.getOutputCurrent());
    SmartLogger.logReplay("Singulator/BallsFedCount", robotState.getBallsFedCount());

    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Singulator/BallsFed", robotState.getBallsFedCount());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Singulator/ShotRatePerSec", computeShotRate());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Singulator/BallPresent", ballBlocked);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Singulator/DeadZoneBallPresent", deadZoneBlocked);
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
