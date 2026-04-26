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

// Feeds balls one at a time from the spindexer up to the flywheels.
// A LaserCAN sensor detects when a ball is present and staged for feeding.
// primeAndFeed() does a brief reverse pulse first to seat the ball before feeding forward.
public class SingulatorSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final SparkMax motor;
  private final LaserCan laserCan;

  private final Timer primeTimer = new Timer();
  private boolean priming = false;

  public SingulatorSubsystem(RobotState robotState) {
    this.robotState = robotState;

    motor = new SparkMax(Constants.Singulator.MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(Constants.Singulator.MOTOR_INVERTED);
    config.smartCurrentLimit(Constants.Singulator.CURRENT_LIMIT_AMPS);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    laserCan = new LaserCan(Constants.Singulator.LASERCAN_ID);
    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 8, 8));
    } catch (au.grapplerobotics.ConfigurationFailedException e) {
      SmartLogger.logConsole("LaserCAN config failed: " + e.getMessage(), "Singulator");
    }

    SmartLogger.logConsole("Singulator ready (CAN " + Constants.Singulator.MOTOR_ID + ")", "Singulator");
  }

  // Reverse briefly to seat the ball, then switch to feeding forward.
  // Use this instead of spinFeed() for normal shooting.
  public void primeAndFeed() {
    if (priming || robotState.getSingulatorState() == RobotState.SingulatorState.FEEDING) return;
    priming = true;
    primeTimer.restart();
    motor.set(Constants.Singulator.REVERSE_SPEED);
    robotState.setSingulatorState(RobotState.SingulatorState.REVERSING);
  }

  // Feed forward directly without the prime pulse
  public void spinFeed() {
    priming = false;
    motor.set(Constants.Singulator.FEED_SPEED);
    robotState.setSingulatorState(RobotState.SingulatorState.FEEDING);
  }

  // Stop the motor
  public void pause() {
    priming = false;
    motor.set(0.0);
    robotState.setSingulatorState(RobotState.SingulatorState.PAUSED);
  }

  // Reverse to clear a jam
  public void spinReverse() {
    priming = false;
    motor.set(Constants.Singulator.REVERSE_SPEED);
    robotState.setSingulatorState(RobotState.SingulatorState.REVERSING);
  }

  public void stopAll() { pause(); }

  // True when a ball is close enough to block the LaserCAN beam
  public boolean isBallPresent() {
    LaserCan.Measurement m = laserCan.getMeasurement();
    return m != null
        && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && m.distance_mm <= Constants.Singulator.LASERCAN_THRESHOLD_MM;
  }

  @Override
  public void periodic() {
    // After the prime pulse expires, switch to feed
    if (priming && primeTimer.hasElapsed(Constants.Singulator.PRIME_REVERSE_SECS)) {
      priming = false;
      motor.set(Constants.Singulator.FEED_SPEED);
      robotState.setSingulatorState(RobotState.SingulatorState.FEEDING);
    }

    boolean ballPresent = isBallPresent();
    robotState.setSingulatorBeamBreak(ballPresent);
    SmartLogger.logReplay("Singulator/BallPresent", ballPresent);
    SmartLogger.logReplay("Singulator/CurrentAmps", motor.getOutputCurrent());
    SmartLogger.logReplay("Singulator/State", robotState.getSingulatorState().toString());
  }
}