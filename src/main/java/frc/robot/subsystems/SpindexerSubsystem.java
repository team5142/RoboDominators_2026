package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants; 
import frc.robot.RobotState;  
import frc.robot.util.SmartLogger;
import frc.robot.RobotState;
/*
 * TASK 2 - Declare a Motor
 * -----------------------------------------------------------------------
 * The Spindexer is a cone-shaped spinning disk that feeds balls toward
 * the Singulator. It is driven by a single NEO brushless motor controlled
 * by a SparkMax motor controller.
 *
 * A SparkMax is created with two arguments:
 *   - CAN ID : a unique number identifying this motor on the robot bus
 *   - MotorType : NEO motors use MotorType.kBrushless
 *
 * Example:
 *   private final SparkMax motor = new SparkMax(10, MotorType.kBrushless);
 *
 * The CAN ID for the spindexer motor is: Constants.Spindexer.MOTOR_ID
 *
 * Steps:
 *   1. Add these imports at the top of the file:
 *        import com.revrobotics.spark.SparkMax;
 *        import com.revrobotics.spark.SparkLowLevel.MotorType;
 *        import frc.robot.Constants; *     

 *   2. Declare a private final SparkMax field called "motor" inside the class.
 *   3. In the constructor, create the SparkMax using Constants.Spindexer.MOTOR_ID.
 *
 * When done: run .\gradlew.bat compileJava
 * If it says BUILD SUCCESSFUL, move on to Task 3.
 * -----------------------------------------------------------------------
 */

/*
 * TASK 3 - Spin the Motor Forward and Stop It
 * -----------------------------------------------------------------------
 * motor.set(speed) tells the motor how fast to spin.
 *   +1.0 = full speed forward
 *    0.0 = stopped
 *   -1.0 = full speed reverse
 *
 * Use constants instead of hardcoded numbers so speeds are easy to tune.
 * Constants.Spindexer.FORWARD_SPEED is already defined for you.
 *
 * Example method:
 *   public void spinForward() {
 *     motor.set(Constants.Spindexer.FORWARD_SPEED);
 *   }
 *
 * Steps:
 *   1. Add a public void spinForward() method that calls motor.set() with FORWARD_SPEED.
 *   2. Add a public void stop() method that calls motor.set(0.0).
 *
 * When done: compile, then go to Task 4 in RobotContainer.java.
 * -----------------------------------------------------------------------
 */

/*
 * TASK 4 - Wire Buttons in RobotContainer  (go to RobotContainer.java)
 * -----------------------------------------------------------------------
 * Now that spinForward() and stop() exist, go to RobotContainer.java
 * and find Task 4. You will uncomment the button bindings that call them.
 * Come back here for Task 5 when done.
 * -----------------------------------------------------------------------
 */ 

/*
 * TASK 5 - Add Reverse and Safety Stop
 * -----------------------------------------------------------------------
 * Steps:
 *   1. Add spinReverse() - runs the motor at Constants.Spindexer.REVERSE_SPEED
 *
 *      Example:
 *        public void spinReverse() {
 *          motor.set(Constants.Spindexer.REVERSE_SPEED);
 *        }
 *
 *   2. Add stopAll() - this is a safety method called when the robot disables.
 *      It should do the same thing as stop().
 *
 *      Example:
 *        public void stopAll() { stop(); }
 *
 * When done: go to Task 6 in RobotContainer.java, then Task 7 in Robot.java.
 * -----------------------------------------------------------------------
 */

/*
 * TASK 8 - Track State with RobotState
 * -----------------------------------------------------------------------
 * RobotState is a shared record of what every mechanism is doing right now.
 * Other parts of the code read it to make decisions without needing a direct
 * reference to this subsystem.
 *
 * RobotState already has a SpindexerState enum with values:
 *   STOPPED, FORWARD, REVERSE
 *
 * To use it, the constructor needs to accept a RobotState and store it:
 *
 *   Example field:
 *     private final RobotState robotState;
 *
 *   Example constructor change:
 *     public SpindexerSubsystem(RobotState robotState) {
 *       this.robotState = robotState;
 *     }
 *
 *   Example state update inside spinForward():
 *     robotState.setSpindexerState(RobotState.SpindexerState.FORWARD);
 *
 * Steps:
 *   1. Add: import frc.robot.RobotState;
 *   2. Add a private final RobotState field.
 *   3. Update the constructor to accept and store RobotState.
 *   4. In spinForward()  set state to FORWARD.
 *   5. In spinReverse()  set state to REVERSE.
 *   6. In stop() and stopAll()  set state to STOPPED.
 *
 * When done: go to Task 9 in RobotContainer.java to update the constructor call.
 * -----------------------------------------------------------------------
 */

/*
 * TASK 10 - Log Data in periodic()
 * -----------------------------------------------------------------------
 * periodic() runs every 20ms while the robot is on.
 * It is the right place to read sensors and record data for debugging.
 *
 * SmartLogger.logReplay(key, value) saves a value to the log and streams
 * it live to AdvantageScope so you can watch it change in real time.
 *
 * Example:
 *   SmartLogger.logReplay("Spindexer/CurrentAmps", motor.getOutputCurrent());
 *   SmartLogger.logReplay("Spindexer/VelocityRpm", motor.getEncoder().getVelocity());
 *
 * Steps:
 *   1. Add: import frc.robot.util.SmartLogger;
 *   2. Add those two logReplay lines inside periodic().
 *
 * [ROBOT OPTIONAL] If the robot is free, deploy and open AdvantageScope.
 * Hold the Right Bumper and watch CurrentAmps rise as the motor spins.
 * If the robot is not available, keep going and verify this later.
 * See docs/ADVANTAGESCOPE_GUIDE.md when you are ready to try it.
 *
 * When done: go to Task 11 in AutoCommands.java.
 * -----------------------------------------------------------------------
 */

/*
 * TASK 12b - Configure the Motor (Advanced)
 * -----------------------------------------------------------------------
 * Motors need configuration before use:
 *   - Invert: flips which direction counts as forward based on how it is mounted
 *   - Current limit: caps power draw to protect the motor from burning out
 *
 * SparkMaxConfig is how you set both of those on a SparkMax.
 *
 * Example - add this in the constructor after creating the motor:
 *   SparkMaxConfig config = new SparkMaxConfig();
 *   config.inverted(Constants.Spindexer.MOTOR_INVERTED);
 *   config.smartCurrentLimit(Constants.Spindexer.CURRENT_LIMIT_AMPS);
 *   motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 *
 * Steps:
 *   1. Add these imports:
 *        import com.revrobotics.spark.config.SparkMaxConfig;
 *        import com.revrobotics.PersistMode;
 *        import com.revrobotics.ResetMode;
 *   2. Add the configuration block to the constructor.
 *
 * When done: compile, then move to Task 13a in Constants.java.
 * -----------------------------------------------------------------------
 */

/*
 * TASK 13b - Jam Detection in periodic() (Advanced)
 * -----------------------------------------------------------------------
 * When a ball jams the spindexer, the motor strains against it and draws
 * extra current. If current stays high long enough, we know something is stuck.
 * The fix: fire a short reverse pulse to relieve pressure, then resume forward.
 *
 * You will need two counter fields:
 *   private int stallLoopCount = 0;        // counts consecutive high-current loops
 *   private int agitateLoopsRemaining = 0; // counts how many loops the reverse pulse has left
 *
 * Constants you will use (already defined in Constants.Spindexer):
 *   LOAD_CURRENT_AMPS      - current level that indicates a jam
 *   AGITATE_LOOP_THRESHOLD - consecutive high-current loops before triggering a pulse
 *   AGITATE_PULSE_LOOPS    - how many loops the reverse pulse lasts
 *
 * Logic to add inside periodic():
 *
 *   Part 1 - while state is FORWARD:
 *     if (currentAmps > Constants.Spindexer.LOAD_CURRENT_AMPS) {
 *       stallLoopCount++;
 *     } else {
 *       stallLoopCount = 0;
 *     }
 *     if (stallLoopCount >= Constants.Spindexer.AGITATE_LOOP_THRESHOLD) {
 *       stallLoopCount = 0;
 *       agitateLoopsRemaining = Constants.Spindexer.AGITATE_PULSE_LOOPS;
 *       motor.set(Constants.Spindexer.REVERSE_SPEED);
 *       robotState.setSpindexerState(RobotState.SpindexerState.REVERSE);
 *     }
 *
 *   Part 2 - while state is REVERSE and agitateLoopsRemaining > 0:
 *     agitateLoopsRemaining--;
 *     if (agitateLoopsRemaining == 0) { 
 *       motor.set(Constants.Spindexer.FORWARD_SPEED);
 *       robotState.setSpindexerState(RobotState.SpindexerState.FORWARD);
 *     }
 *
 * [ROBOT OPTIONAL] While the spindexer is spinning, block it by hand briefly.
 * Watch Spindexer/CurrentAmps spike in AdvantageScope, then see the pulse fire.
 * -----------------------------------------------------------------------
 */

// The Spindexer is a cone-shaped spinning disk that feeds balls toward the Singulator.
// It is driven by a single NEO motor controlled by a SparkMax motor controller.
public class SpindexerSubsystem extends SubsystemBase {
  private final SparkMax special554;
  public SpindexerSubsystem() {
    special554 = new SparkMax(Constants.Spindexer.MOTOR_ID, MotorType.kBrushless);
  
  }
  
  
  public void spinForward() {
    special554.set(Constants.Spindexer.FORWARD_SPEED);
  }
  public void spinReverse() {
    special554.set(Constants.Spindexer.REVERSE_SPEED);
  }
  public void stop() {
    special554.set(0.0);
  }
  public void stopAll() { stop(); }
  //private final RobotState robotState;
  
  
  @Override
  public void periodic() {
  }
}
