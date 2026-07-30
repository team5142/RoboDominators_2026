package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.SpindexerState;
import frc.robot.util.SmartLogger;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
 import com.revrobotics.ResetMode;
// spindexer Class is completed
public class SpindexerSubsystem extends SubsystemBase {
  private final SparkMax spinMotor;
  private RobotState.SpindexerState robotState;
  private double currentAmps;
  private double currentVelocity;
  private int stallLoopCount = 0;        // counts consecutive high-current loops
  private int agitateLoopsRemaining = 0; // counts how many loops the reverse pulse has left
    public SpindexerSubsystem(RobotState robotState) {
      spinMotor = new SparkMax(Constants.Spindexer.MOTOR_ID , MotorType.kBrushless);
      this.robotState = RobotState.SpindexerState.STOPPED;
      SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(Constants.Spindexer.MOTOR_INVERTED);
    config.smartCurrentLimit(Constants.Spindexer.CURRENT_LIMIT_AMPS);
    spinMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void motorForward() { 
      spinMotor.set(Constants.Spindexer.FORWARD_SPINDEXER_SPEED);
      System.out.println("Spindexer spinning forward");
      robotState = RobotState.SpindexerState.FORWARD;
  }
  public void motorStop() {
    spinMotor.set(0.0);
    System.out.println("Spindexer stopped");
    robotState = RobotState.SpindexerState.STOPPED;
  }
  public void motorBackward() {
    spinMotor.set(Constants.Spindexer.REVERSE_SPINDEXER_SPEED);
    System.out.println("Spinder spinning backward");
    robotState = RobotState.SpindexerState.REVERSE;
  }
  public void stopAll() { 
    spinMotor.set(0.0);
    System.out.println("Spindexer stopped");
    robotState = RobotState.SpindexerState.STOPPED;
  }
  
  @Override
  public void periodic() {
    currentAmps=spinMotor.getOutputCurrent();
    currentVelocity=spinMotor.getEncoder().getVelocity();
    SmartLogger.logReplay("Spindexer/CurrentAmps", spinMotor.getOutputCurrent());
    SmartLogger.logReplay("Spindexer/VelocityRpm", spinMotor.getEncoder().getVelocity());
     if (currentAmps>Constants.Spindexer.LOAD_CURRENT_AMPS) {
        stallLoopCount++;
        System.out.println("Spindexer stalling");
      } 
      if (stallLoopCount >= Constants.Spindexer.AGITATE_LOOP_THRESHOLD) {
        stallLoopCount = 0;
        agitateLoopsRemaining = Constants.Spindexer.AGITATE_PULSE_LOOPS;
        spinMotor.set(Constants.Spindexer.REVERSE_SPINDEXER_SPEED);
        robotState=RobotState.SpindexerState.REVERSE;
        System.out.println("Agitation initiated ");
      }
    if (robotState==RobotState.SpindexerState.REVERSE&&agitateLoopsRemaining>0) {
      agitateLoopsRemaining--;
      if (agitateLoopsRemaining==0) {
        spinMotor.set(Constants.Spindexer.FORWARD_SPINDEXER_SPEED);
        robotState=RobotState.SpindexerState.FORWARD;
      }
     }
   
  }
}
