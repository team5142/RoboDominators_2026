package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.RobotState;
import frc.robot.RobotState.SingulatorState;
import frc.robot.util.SmartLogger;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
 import com.revrobotics.ResetMode;
// singulator class is completed
=======
/*
 * TASK 14 - Declare and Create the Singulator Motor
 * -----------------------------------------------------------------------
 * The Singulator feeds balls one at a time from the Spindexer up to the
 * flywheels. It has one NEO motor on a SparkMax, and a LaserCAN sensor
 * that detects when a ball is staged and ready.
 *
 * The LaserCAN is already wired up for you in isBallPresent() below.
 * Your job is to declare and configure the motor - same pattern as the
 * Spindexer, which you have already done.
 *
 * CAN ID: Constants.Singulator.MOTOR_ID
 *
 * Steps:
 *   1. Add imports for SparkMax, MotorType, SparkMaxConfig, ResetMode, PersistMode,
 *      RobotState, SmartLogger, and frc.robot.Constants (if not already imported).
 *   2. Declare private final fields for the motor and for RobotState.
 *   3. Update the constructor to accept RobotState.
 *   4. In the constructor, create the SparkMax.
 *   5. Create a SparkMaxConfig, set inverted to Constants.Singulator.MOTOR_INVERTED,
 *      set smartCurrentLimit to Constants.Singulator.CURRENT_LIMIT_AMPS,
 *      and call motor.configure().
 *   6. Add a SmartLogger.logConsole line so you can see it boot in the console.
 *      Something like: "Singulator ready (CAN " + Constants.Singulator.MOTOR_ID + ")"
 *
 * When done: compile and move to Task 15.
 * -----------------------------------------------------------------------
 */

/*
 * TASK 15 - Add Feed, Pause, Reverse, and StopAll Methods
 * -----------------------------------------------------------------------
 * The Singulator needs four methods. The Spindexer had similar methods -
 * think about what each one should do before writing it.
 *
 *   spinFeed()    - run motor at Constants.Singulator.FEED_SPEED, set state to FEEDING
 *   pause()       - stop motor, set state to PAUSED
 *   spinReverse() - run motor at Constants.Singulator.REVERSE_SPEED, set state to REVERSING
 *   stopAll()     - same as pause() - called as a safety stop on disable
 *
 * Notice that FEED_SPEED and REVERSE_SPEED need to exist in Constants.Singulator.
 * Check if they are already there. If not, Task 13a asked you to add them - do that first.
 *
 * When done: compile and move to Task 16 in RobotContainer.java.
 * -----------------------------------------------------------------------
 */

/*
 * TASK 17a - Add periodic() Logging and Ball Detection
 * -----------------------------------------------------------------------
 * isBallPresent() is already written below - it reads the LaserCAN sensor.
 * Your job is to call it in periodic() and log the result.
 *
 * Steps:
 *   1. In periodic(), call isBallPresent() and store the result in a boolean.
 *   2. Call robotState.setSingulatorBeamBreak() with that value.
 *      (You wrote setSingulatorBeamBreak in Task 21a of RobotState.java)
 *   3. Log it: SmartLogger.logReplay("Singulator/BallPresent", ballPresent)
 *   4. Also log motor current: SmartLogger.logReplay("Singulator/CurrentAmps", ...)
 *
 * [ROBOT OPTIONAL] Deploy and watch Singulator/BallPresent toggle in AdvantageScope
 * when you place a ball at the singulator staging point.
 *
 * When done: move to Task 17b in RobotContainer.java.
 * -----------------------------------------------------------------------
 */

// The Singulator feeds balls one at a time from the Spindexer up to the flywheels.
// A LaserCAN sensor detects when a ball is staged and ready.
>>>>>>> 6b293bbba60886be576841eeea49c1b5596d8bd1
public class SingulatorSubsystem extends SubsystemBase {

  private final LaserCan laserCan;
  private final SparkMax singulatorMotor;
  private final int smartCurrentLimit;

  public boolean ballPresent;
  private RobotState.SingulatorState singulatorState;
  public SingulatorSubsystem(RobotState robotState) {

    singulatorState=RobotState.SingulatorState.PAUSED;
    smartCurrentLimit=Constants.Singulator.CURRENT_LIMIT_AMPS;
    singulatorMotor=new SparkMax(Constants.Singulator.MOTOR_ID, null);
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(Constants.Singulator.MOTOR_INVERTED);
    config.smartCurrentLimit(Constants.Singulator.CURRENT_LIMIT_AMPS);
    singulatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartLogger.logConsole( "Singulator ready (CAN " + Constants.Singulator.MOTOR_ID + ")");
    laserCan = new LaserCan(Constants.Singulator.LASERCAN_ID);
    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 8, 8));
    } catch (au.grapplerobotics.ConfigurationFailedException e) {
      System.out.println("LaserCAN config failed: " + e.getMessage());
    }
  }
  public void spinFeed() {
    singulatorMotor.set(Constants.Singulator.FORWARD_SINGULATOR_SPEED);
    SmartLogger.logConsole("Singulator moving forward");
    singulatorState=RobotState.SingulatorState.FEEDING;
  }
  public void pause() {
    singulatorMotor.set(0.0);
    SmartLogger.logConsole(" Singulator Paused ");
    singulatorState=RobotState.SingulatorState.PAUSED;
  }
  private void spinReverse(){
    singulatorMotor.set(Constants.Singulator.REVERSE_SINGULATOR_SPEED);
    SmartLogger.logConsole("Singulator in reverse");
    singulatorState=RobotState.SingulatorState.REVERSING;
  }
  private void StopAll() {
    singulatorMotor.set(0.0);
    SmartLogger.logConsole(" Singulator Paused ");
    singulatorState=RobotState.SingulatorState.PAUSED;
  }
  // Returns true when a ball is close enough to block the LaserCAN beam.
  // This is pre-built - you do not need to modify it.
  public boolean isBallPresent() {
    LaserCan.Measurement m = laserCan.getMeasurement();
    return m != null
        && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && m.distance_mm <= Constants.Singulator.LASERCAN_THRESHOLD_MM;
  }

  @Override
  public void periodic() {
    ballPresent=isBallPresent();
    RobotState.setSingulatorBeamBreak(ballPresent);
    SmartLogger.logReplay("Singulator/BallPresent", ballPresent);
    SmartLogger.logReplay("Singulator/CurrentAmps", singulatorMotor.getOutputCurrent());  
  }
}
