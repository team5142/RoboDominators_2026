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
