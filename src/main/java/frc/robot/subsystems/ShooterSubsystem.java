package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartLogger;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

// Controls two Kraken X60 flywheel motors that pinch-shoot game pieces at 45 degrees
// Fixed angle shooter - adjust RPM for different distances (3000-5000 RPM range)
// Shooting zone: 2.1 feet (closest legal, just past zone boundary) to 13.0 feet (far back at alliance wall)
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX leftFlywheel;  // Left side motor (spins one direction)
  private final TalonFX rightFlywheel; // Right side motor (spins opposite to pinch ball)
  
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0); // Control flywheel speed
  private double targetRPM = 0.0; // Speed we're trying to reach
  
  private PoseEstimatorSubsystem poseEstimator; // Gets robot position on field
  
  public ShooterSubsystem() {
    // Create motor objects with CAN IDs and bus name from Constants
    leftFlywheel = new TalonFX(Constants.Shooter.LEFT_FLYWHEEL_ID, Constants.Shooter.SHOOTER_CAN_BUS);
    rightFlywheel = new TalonFX(Constants.Shooter.RIGHT_FLYWHEEL_ID, Constants.Shooter.SHOOTER_CAN_BUS);
    
    configureFlywheels(); // Set up PID, current limits, and inversions
    
    SmartLogger.logConsole("ShooterSubsystem initialized - fixed 45deg angle");
  }
  
  private void configureFlywheels() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // PID values control how motors reach target speed (kP = proportional gain)
    config.Slot0.kP = Constants.Shooter.FLYWHEEL_KP;
    config.Slot0.kI = Constants.Shooter.FLYWHEEL_KI;
    config.Slot0.kD = Constants.Shooter.FLYWHEEL_KD;
    config.Slot0.kS = Constants.Shooter.FLYWHEEL_KS; // Voltage to overcome friction
    config.Slot0.kV = Constants.Shooter.FLYWHEEL_KV; // Voltage per unit velocity
    
    // Coast mode = flywheels spin freely when not powered (faster spin-down)
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    // Limit current to 80 amps to prevent motor damage
    config.CurrentLimits.StatorCurrentLimit = Constants.Shooter.FLYWHEEL_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Gear ratio: 1.5:1 means flywheels spin 1.5x faster than motors (overdrive)
    config.Feedback.SensorToMechanismRatio = Constants.Shooter.FLYWHEEL_GEAR_RATIO;
    
    // Left flywheel spins counter-clockwise (normal direction)
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftFlywheel.getConfigurator().apply(config);
    
    // Right flywheel spins clockwise (opposite) to create pinch effect
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightFlywheel.getConfigurator().apply(config);
  }
  
  // Set how fast flywheels should spin (input in RPM - rotations per minute)
  public void setTargetRPM(double rpm) {
    double clampedRPM = Math.min(rpm, Constants.Shooter.MAX_RPM); // Don't exceed 6000 RPM max
    targetRPM = clampedRPM;
    
    // Motors use RPS (rotations per second) so divide by 60
    double rps = clampedRPM / 60.0;
    leftFlywheel.setControl(velocityControl.withVelocity(rps));
    rightFlywheel.setControl(velocityControl.withVelocity(rps));
  }
  
  // Spin up to shooting speed (3500 RPM default)
  public void spinUp() {
    setTargetRPM(Constants.Shooter.SHOOTING_RPM);
  }
  
  // Return to idle speed (500 RPM) - keeps wheels warm but not dangerous
  public Command idle() {
    return Commands.runOnce(() -> setTargetRPM(Constants.Shooter.IDLE_RPM), this);
  }
  
  // Completely stop flywheels
  public void stop() {
    setTargetRPM(0.0);
  }
  
  // Check if flywheels are within 50 RPM of target speed (ready to shoot)
  public boolean atTargetSpeed() {
    double leftRPM = leftFlywheel.getVelocity().getValueAsDouble() * 60.0;  // Convert RPS to RPM
    double rightRPM = rightFlywheel.getVelocity().getValueAsDouble() * 60.0;
    double avgRPM = (leftRPM + rightRPM) / 2.0; // Average both sides
    
    return Math.abs(avgRPM - targetRPM) < Constants.Shooter.RPM_TOLERANCE; // Within 50 RPM?
  }
  
  // Get current flywheel speed in RPM (average of both sides)
  public double getCurrentRPM() {
    double leftRPM = leftFlywheel.getVelocity().getValueAsDouble() * 60.0;
    double rightRPM = rightFlywheel.getVelocity().getValueAsDouble() * 60.0;
    return (leftRPM + rightRPM) / 2.0;
  }
  
  // Get hub position for our alliance (blue hub or red hub)
  private Pose2d getAllianceHub() {
    var alliance = DriverStation.getAlliance();
    
    // Red alliance hub is mirrored only along field length (X-axis), Y stays same
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      Pose2d blueHub = Constants.StartingPositions.BLUE_HUB_CENTER;
      double fieldLength = Units.inchesToMeters(650.12); // 2026 field is 650.12 inches long
      double redX = fieldLength - blueHub.getX(); // Mirror X coordinate
      double redY = blueHub.getY(); // Y coordinate stays same (field width doesn't flip)
      return new Pose2d(redX, redY, blueHub.getRotation());
    }
    
    return Constants.StartingPositions.BLUE_HUB_CENTER; // Default to blue hub
  }
  
  // Public accessor for TurretSubsystem auto-tracking
  public Pose2d getAllianceHubPublic() {
    return getAllianceHub();
  }
  
  // Check if robot center is in legal shooting zone (accounts for robot extending past line)
  public boolean isInShootingZone(Pose2d robotPose) {
    var alliance = DriverStation.getAlliance();
    
    // Red alliance: Must stay on far side of field (high X values)
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      double fieldLength = Units.inchesToMeters(650.12);
      // Use effective boundary (accounts for robot length extending past line)
      double redZoneMin = fieldLength - Constants.StartingPositions.EFFECTIVE_SHOOTING_ZONE_MAX_X_METERS;
      return robotPose.getX() >= redZoneMin; // Robot center X must be greater than effective boundary
    }
    
    // Blue alliance: Must stay on near side of field (low X values)
    // Use effective boundary to allow front bumper at line while center is back
    return robotPose.getX() <= Constants.StartingPositions.EFFECTIVE_SHOOTING_ZONE_MAX_X_METERS;
  }
  
  // Check if robot can legally shoot at hub from current position
  public boolean canShootAtHub(Pose2d robotPose) {
    // First check: Are we in the legal shooting zone?
    if (!isInShootingZone(robotPose)) {
      Logger.recordOutput("Shooter/IllegalPosition", true);
      return false;
    }
    
    Pose2d hubCenter = getAllianceHub();
    
    // Second check: Are we on the correct side of hub? (must be closer to driver station)
    var alliance = DriverStation.getAlliance();
    boolean validSide;
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      validSide = robotPose.getX() >= hubCenter.getX(); // Red shoots downfield (high X to low X)
    } else {
      validSide = robotPose.getX() <= hubCenter.getX(); // Blue shoots upfield (low X to high X)
    }
    
    if (!validSide) {
      Logger.recordOutput("Shooter/WrongSideOfHub", true);
      return false;
    }
    
    Logger.recordOutput("Shooter/CanShoot", true);
    return true; // Legal position and correct side - ready to shoot!
  }
  
  // Calculate straight-line distance from robot to hub center (in meters)
  public double getDistanceToHub(Pose2d robotPose) {
    return robotPose.getTranslation().getDistance(getAllianceHub().getTranslation());
  }
  
  // Called from RobotContainer to give shooter access to robot position
  public void setPoseEstimator(PoseEstimatorSubsystem poseEstimator) {
    this.poseEstimator = poseEstimator;
  }
  
  // Runs every 20ms (50 times per second) - logs data to AdvantageScope
  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/TargetRPM", targetRPM);
    Logger.recordOutput("Shooter/CurrentRPM", getCurrentRPM());
    Logger.recordOutput("Shooter/AtTargetSpeed", atTargetSpeed());
    Logger.recordOutput("Shooter/LeftFlywheelAmps", leftFlywheel.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/RightFlywheelAmps", rightFlywheel.getStatorCurrent().getValueAsDouble());
    
    // Only log position data if we have a pose estimator
    if (poseEstimator != null) {
      Pose2d robotPose = poseEstimator.getEstimatedPose(); // Get current robot position
      Logger.recordOutput("Shooter/AllianceHub", getAllianceHub()); // Where we're aiming
      Logger.recordOutput("Shooter/DistanceToHubMeters", getDistanceToHub(robotPose)); // How far away
    }
  }
}
