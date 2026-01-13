package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartLogger;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;

// Controls turret rotation using Kraken X44 motor with 180:1 gear reduction
// Turret can spin 360 degrees (-180 to +180) with hard stops at both ends
public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor;  // Motor that spins the turret
  private final CANcoder cancoder;    // Absolute encoder that tracks turret angle
  
  private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(0); // Control turret position
  private double targetAngleDeg = 0.0; // Angle we're trying to reach (in degrees)
  
  private PoseEstimatorSubsystem poseEstimator; // Gets robot position on field
  private ShooterSubsystem shooter; // Gets hub position
  
  public TurretSubsystem() {
    // Create motor and encoder objects with CAN IDs from Constants
    turretMotor = new TalonFX(Constants.Shooter.TURRET_MOTOR_ID, Constants.Shooter.SHOOTER_CAN_BUS);
    cancoder = new CANcoder(Constants.Shooter.TURRET_CANCODER_ID, Constants.Shooter.SHOOTER_CAN_BUS);
    
    configureTurretMotor(); // Set up PID, limits, and encoder fusion
    configureCANcoder();    // Set up absolute encoder with offset
    
    SmartLogger.logConsole("TurretSubsystem initialized");
  }
  
  private void configureTurretMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // PID values control how motor reaches target angle
    config.Slot0.kP = Constants.Shooter.TURRET_KP; // Proportional gain (24.0 = aggressive tracking)
    config.Slot0.kI = Constants.Shooter.TURRET_KI; // Integral gain (0 = no accumulation)
    config.Slot0.kD = Constants.Shooter.TURRET_KD; // Derivative gain (1.5 = dampens oscillation)
    config.Slot0.kS = Constants.Shooter.TURRET_KS; // Voltage to overcome friction
    config.Slot0.kV = Constants.Shooter.TURRET_KV; // Voltage per unit velocity
    
    // Motion Magic creates smooth acceleration curves (prevents jerky movement)
    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Shooter.TURRET_MAX_VELOCITY_DEG_PER_SEC; // Max 180 deg/s
    config.MotionMagic.MotionMagicAcceleration = Constants.Shooter.TURRET_MAX_ACCEL_DEG_PER_SEC2;     // Max 360 deg/s^2
    
    // Brake mode = motor actively resists movement when stopped (holds position)
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    // Limit current to 40 amps to prevent motor damage
    config.CurrentLimits.StatorCurrentLimit = Constants.Shooter.TURRET_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Use CANcoder as feedback source (more accurate than motor encoder)
    config.Feedback.FeedbackRemoteSensorID = Constants.Shooter.TURRET_CANCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; // Fuse motor + CANcoder
    config.Feedback.SensorToMechanismRatio = Constants.Shooter.TURRET_GEAR_RATIO;   // 180:1 gearing
    config.Feedback.RotorToSensorRatio = 1.0; // CANcoder directly on turret shaft
    
    // Software limits prevent turret from hitting hard stops
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Shooter.TURRET_MAX_ANGLE_DEG;  // +180 degrees
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Shooter.TURRET_MIN_ANGLE_DEG;  // -180 degrees
    
    turretMotor.getConfigurator().apply(config); // Send config to motor
    
    turretMotor.setPosition(0.0); // Reset motor encoder to match CANcoder at startup
  }
  
  private void configureCANcoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    
    // Counter-clockwise rotation increases angle (standard math convention)
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    
    // Offset makes CANcoder read 0 degrees when turret points forward
    config.MagnetSensor.MagnetOffset = Constants.Shooter.TURRET_CANCODER_OFFSET; // Calibrated offset (0.102 rotations)
    
    cancoder.getConfigurator().apply(config);
  }
  
  // Command turret to rotate to specific angle
  public void setTargetAngle(Rotation2d angle) {
    // Clamp angle to physical limits (-180 to +180 degrees)
    double clampedAngle = Math.max(Constants.Shooter.TURRET_MIN_ANGLE_DEG,
                                    Math.min(Constants.Shooter.TURRET_MAX_ANGLE_DEG, angle.getDegrees()));
    targetAngleDeg = clampedAngle;
    
    // Convert degrees to rotations (motor uses rotations internally)
    turretMotor.setControl(positionControl.withPosition(clampedAngle / 360.0));
  }
  
  // Get turret's current angle in degrees
  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromDegrees(turretMotor.getPosition().getValueAsDouble() * 360.0); // Convert rotations to degrees
  }
  
  // Check if turret is within 2 degrees of target (close enough to shoot)
  public boolean atTarget() {
    double error = Math.abs(getCurrentAngle().getDegrees() - targetAngleDeg);
    return error < 2.0; // Tolerance: 2 degrees
  }
  
  // Stop turret motor (no active control)
  public void stop() {
    turretMotor.setControl(new NeutralOut());
  }
  
  // Called from RobotContainer to enable auto-tracking
  public void setPoseEstimator(PoseEstimatorSubsystem poseEstimator) {
    this.poseEstimator = poseEstimator;
  }
  
  public void setShooter(ShooterSubsystem shooter) {
    this.shooter = shooter;
  }
  
  // Create default command that tracks hub when in shooting zone, pass target when in neutral zone
  public Command createAutoTrackCommand() {
    return Commands.run(() -> {
      // Only track if we have pose estimator and shooter configured
      if (poseEstimator == null || shooter == null) {
        return;
      }
      
      Pose2d robotPose = poseEstimator.getEstimatedPose();
      Pose2d targetPose;
      String targetMode;
      
      // Determine target based on robot location
      if (shooter.isInShootingZone(robotPose)) {
        // IN ALLIANCE AREA: Aim at hub for scoring
        targetPose = shooter.getAllianceHubPublic();
        targetMode = "HUB_SCORING";
        Logger.recordOutput("Turret/AutoTrackActive", true);
      } else {
        // IN NEUTRAL ZONE: Aim at safe passing target (avoid hub penalty)
        targetPose = getPassingTarget(robotPose);
        targetMode = "PASSING_MODE";
        Logger.recordOutput("Turret/AutoTrackActive", false); // Not scoring, just passing
      }
      
      // Calculate angle from robot to target
      Translation2d robotToTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
      double angleToTargetRad = Math.atan2(robotToTarget.getY(), robotToTarget.getX());
      
      // Convert to turret-relative angle (subtract robot heading)
      double robotHeadingRad = robotPose.getRotation().getRadians();
      double turretAngleRad = angleToTargetRad - robotHeadingRad;
      
      // Command turret to aim at target
      setTargetAngle(new Rotation2d(turretAngleRad));
      
      Logger.recordOutput("Turret/TargetMode", targetMode);
      Logger.recordOutput("Turret/TargetPose", targetPose);
      Logger.recordOutput("Turret/DistanceToTargetMeters", robotToTarget.getNorm());
    }, this).withName("AutoTrackHub");
  }
  
  // Determine which passing target to use based on robot Y position
  private Pose2d getPassingTarget(Pose2d robotPose) {
    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    
    // Blue alliance: Use Y threshold to pick left/right target
    if (!alliance.isPresent() || alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
      boolean onRightSide = robotPose.getY() < Constants.StartingPositions.PASS_TARGET_Y_THRESHOLD;
      Pose2d blueTarget = onRightSide 
          ? Constants.StartingPositions.BLUE_PASS_TARGET_RIGHT_SIDE
          : Constants.StartingPositions.BLUE_PASS_TARGET_LEFT_SIDE;
      
      Logger.recordOutput("Turret/PassingSide", onRightSide ? "RIGHT" : "LEFT");
      return blueTarget;
    }
    
    // Red alliance: Mirror blue targets across field
    boolean onRightSide = robotPose.getY() < Constants.StartingPositions.PASS_TARGET_Y_THRESHOLD;
    Pose2d blueTarget = onRightSide
        ? Constants.StartingPositions.BLUE_PASS_TARGET_RIGHT_SIDE
        : Constants.StartingPositions.BLUE_PASS_TARGET_LEFT_SIDE;
    
    // Mirror X coordinate for red alliance (Y stays same)
    double fieldLength = edu.wpi.first.math.util.Units.inchesToMeters(650.12);
    double redX = fieldLength - blueTarget.getX();
    Pose2d redTarget = new Pose2d(redX, blueTarget.getY(), blueTarget.getRotation());
    
    Logger.recordOutput("Turret/PassingSide", onRightSide ? "RIGHT" : "LEFT");
    return redTarget;
  }
  
  // Runs every 20ms (50 times per second) - logs data to AdvantageScope
  @Override
  public void periodic() {
    Logger.recordOutput("Turret/CurrentAngleDeg", getCurrentAngle().getDegrees());
    Logger.recordOutput("Turret/TargetAngleDeg", targetAngleDeg);
    Logger.recordOutput("Turret/AtTarget", atTarget());
    Logger.recordOutput("Turret/CurrentAmps", turretMotor.getStatorCurrent().getValueAsDouble());
  }
}
