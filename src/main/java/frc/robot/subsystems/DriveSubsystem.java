package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.TunableCTREGains; // ADD THIS
import org.littletonrobotics.junction.Logger;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;

import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;

/**
 * Swerve drivetrain extending CTRE's CommandSwerveDrivetrain.
 * Integrates with custom GyroSubsystem for QuestNav/Pigeon failover.
 */
public class DriveSubsystem extends CommandSwerveDrivetrain {
  private final GyroSubsystem gyro;
  private final RobotState robotState;
  
  // NEW: Track if we've manually set operator perspective from QuestNav pose
  private boolean operatorPerspectiveSetFromPose = false;
  
  // Swerve requests for different drive modes
  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

  // SysId requests (directly from parent class)
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  // NEW: AutoPilot integration
  private Autopilot autopilot;
  private APTarget currentTarget = null;
  private boolean autopilotActive = false;

  // AutoPilot error tolerances - AGGRESSIVE for <3cm precision
  private static final double AUTOPILOT_ERROR_XY_METERS = 0.03; // 3cm translation tolerance (was 5cm)
  private static final double AUTOPILOT_ERROR_THETA_DEGREES = 1.0; // 1° rotation tolerance (was 2°)
  private static final double AUTOPILOT_BEELINE_RADIUS_METERS = 0.3; // 30cm beeline radius
  
  // Rotation PID gains - Added damping to prevent oscillation
  private static final double AUTOPILOT_ROTATION_KP = 3.0; // Proportional gain
  private static final double AUTOPILOT_ROTATION_KD = 0.5; // Derivative gain (damping)
  
  // Track previous angle error for derivative calculation
  private double lastAngleError = 0.0;

  /**
   * Constructs the drivetrain using Tuner X generated constants
   */
  public DriveSubsystem(RobotState robotState, GyroSubsystem gyro) {
    // Call parent constructor with Tuner X constants
    super(
        frc.robot.generated.TunerConstants.DrivetrainConstants,
        frc.robot.generated.TunerConstants.FrontLeft,
        frc.robot.generated.TunerConstants.FrontRight,
        frc.robot.generated.TunerConstants.BackLeft,
        frc.robot.generated.TunerConstants.BackRight
    );
    
    this.robotState = robotState;
    this.gyro = gyro;

    // AutoPilot will be initialized when we start navigation
    this.autopilot = null;
  }

  @Override
  public void periodic() {
    // DON'T call super.periodic() if we've manually set operator perspective from QuestNav
    // This prevents CTRE from overwriting our calculated perspective
    if (!operatorPerspectiveSetFromPose) {
      super.periodic(); // Let CTRE set alliance-based perspective
    }
    
    // Get current chassis speeds
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    
    // Log chassis speeds
    Logger.recordOutput("Drive/ChassisSpeed/VX", speeds.vxMetersPerSecond);
    Logger.recordOutput("Drive/ChassisSpeed/VY", speeds.vyMetersPerSecond);
    Logger.recordOutput("Drive/ChassisSpeed/Omega", speeds.omegaRadiansPerSecond);
    
    // Also log combined speed magnitude for easy debugging
    double translationSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    Logger.recordOutput("Drive/ChassisSpeed/TranslationMagnitude", translationSpeed);
    
    // Existing logging
    Logger.recordOutput("Drive/GyroYawDeg", getGyroRotation().getDegrees());
    Logger.recordOutput("Drive/Pose", getState().Pose);
    
    // NEW: Log field-relative "forward" direction for driver reference
    // This is the direction that joystick-forward will move the robot
    Rotation2d operatorForward = getOperatorPerspectiveForward();
    
    // Log as a pose arrow pointing downfield from robot's current position
    Pose2d currentPose = robotState.getRobotPose();
    Pose2d fieldForwardPose = new Pose2d(currentPose.getTranslation(), operatorForward);
    
    Logger.recordOutput("Drive/FieldForwardDirection", fieldForwardPose);
    Logger.recordOutput("Drive/FieldForwardDegrees", operatorForward.getDegrees());
    
    SmartDashboard.putNumber("Drive/FieldForward", operatorForward.getDegrees());
    
    // NEW: Log individual module states for PID tuning
    SwerveModuleState[] moduleStates = getState().ModuleStates;
    Logger.recordOutput("Drive/ModuleStates/FrontLeft/Angle", normalizeAngle(moduleStates[0].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleStates/FrontLeft/Speed", moduleStates[0].speedMetersPerSecond);
    Logger.recordOutput("Drive/ModuleStates/FrontRight/Angle", normalizeAngle(moduleStates[1].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleStates/FrontRight/Speed", moduleStates[1].speedMetersPerSecond);
    Logger.recordOutput("Drive/ModuleStates/BackLeft/Angle", normalizeAngle(moduleStates[2].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleStates/BackLeft/Speed", moduleStates[2].speedMetersPerSecond);
    Logger.recordOutput("Drive/ModuleStates/BackRight/Angle", normalizeAngle(moduleStates[3].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleStates/BackRight/Speed", moduleStates[3].speedMetersPerSecond);
    
    // NEW: Calculate and log desired module states from current chassis speeds
    // This shows what the modules SHOULD be doing based on commanded robot motion
    SwerveModuleState[] desiredStates = getKinematics().toSwerveModuleStates(speeds);

    // Optimize each state relative to current module angle (this is what CTRE does internally)
    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      optimizedStates[i] = SwerveModuleState.optimize(desiredStates[i], moduleStates[i].angle);
    }

    // Log optimized setpoints (what PID is actually targeting)
    Logger.recordOutput("Drive/ModuleSetpoints/FrontLeft/Angle", normalizeAngle(optimizedStates[0].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleSetpoints/FrontLeft/Speed", optimizedStates[0].speedMetersPerSecond);
    Logger.recordOutput("Drive/ModuleSetpoints/FrontRight/Angle", normalizeAngle(optimizedStates[1].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleSetpoints/FrontRight/Speed", optimizedStates[1].speedMetersPerSecond);
    Logger.recordOutput("Drive/ModuleSetpoints/BackLeft/Angle", normalizeAngle(optimizedStates[2].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleSetpoints/BackLeft/Speed", optimizedStates[2].speedMetersPerSecond);
    Logger.recordOutput("Drive/ModuleSetpoints/BackRight/Angle", normalizeAngle(optimizedStates[3].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleSetpoints/BackRight/Speed", optimizedStates[3].speedMetersPerSecond);
    
    // NEW: Log module positions (distance traveled by each wheel)
    edu.wpi.first.math.kinematics.SwerveModulePosition[] modulePositions = getModulePositions();
    Logger.recordOutput("Drive/ModulePositions/FrontLeft", modulePositions[0].distanceMeters);
    Logger.recordOutput("Drive/ModulePositions/FrontRight", modulePositions[1].distanceMeters);
    Logger.recordOutput("Drive/ModulePositions/BackLeft", modulePositions[2].distanceMeters);
    Logger.recordOutput("Drive/ModulePositions/BackRight", modulePositions[3].distanceMeters);

    // NEW: Live-update CTRE gains when changed in AdvantageScope
    if (TunableCTREGains.hasChanged()) {
      System.out.println("CTRE gains updated from AdvantageScope:");
      System.out.println("  Steer: kP=" + TunableCTREGains.STEER_KP.get() + 
                         " kD=" + TunableCTREGains.STEER_KD.get());
      System.out.println("  Drive: kP=" + TunableCTREGains.DRIVE_KP.get() + 
                         " kV=" + TunableCTREGains.DRIVE_KV.get());
      System.out.println("  → Apply new gains manually or restart robot to apply");
      
      // TODO: Optionally re-apply configs here
      // This requires access to TalonFX objects which CommandSwerveDrivetrain hides
      // For now, gains update on next robot reboot
    }

    // NEW: Run AutoPilot if active
    if (autopilotActive && currentTarget != null && autopilot != null) {
      Pose2d currentRobotPose = getState().Pose;
      ChassisSpeeds robotRelativeSpeeds = getRobotRelativeSpeeds();
      
      // Calculate next velocity using AutoPilot
      Autopilot.APResult result = autopilot.calculate(currentRobotPose, robotRelativeSpeeds, currentTarget);
      
      // Calculate rotational velocity to reach target angle with damping
      double currentAngle = currentRobotPose.getRotation().getRadians();
      double targetAngle = result.targetAngle().getRadians();
      double angleError = targetAngle - currentAngle;
      
      // Normalize angle error to [-π, π]
      while (angleError > Math.PI) angleError -= 2 * Math.PI;
      while (angleError < -Math.PI) angleError += 2 * Math.PI;
      
      // Calculate derivative (rate of change of error)
      double angleErrorDerivative = (angleError - lastAngleError) / 0.02; // 20ms loop time
      lastAngleError = angleError;
      
      // PD control for rotation (proportional + derivative damping)
      double omega = (angleError * AUTOPILOT_ROTATION_KP) - (angleErrorDerivative * AUTOPILOT_ROTATION_KD);
      
      // Convert result to ChassisSpeeds (field-relative velocities + rotation)
      ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          result.vx().in(edu.wpi.first.units.Units.MetersPerSecond),
          result.vy().in(edu.wpi.first.units.Units.MetersPerSecond),
          omega, // Use PD-controlled rotation velocity
          currentRobotPose.getRotation()
      );
      
      // Apply the calculated speeds
      driveRobotRelative(targetSpeeds);
      
      // Log AutoPilot state
      Logger.recordOutput("AutoPilot/Active", true);
      Logger.recordOutput("AutoPilot/CurrentPose", currentRobotPose);
      Logger.recordOutput("AutoPilot/TargetPose", currentTarget.getReference());
      Logger.recordOutput("AutoPilot/TargetRotation", result.targetAngle().getDegrees());
      Logger.recordOutput("AutoPilot/AngleError", Math.toDegrees(angleError));
      Logger.recordOutput("AutoPilot/AngleErrorDerivative", angleErrorDerivative);
      Logger.recordOutput("AutoPilot/Omega", omega);
      Logger.recordOutput("AutoPilot/VX", result.vx().in(edu.wpi.first.units.Units.MetersPerSecond));
      Logger.recordOutput("AutoPilot/VY", result.vy().in(edu.wpi.first.units.Units.MetersPerSecond));
      Logger.recordOutput("AutoPilot/AtTarget", autopilot.atTarget(currentRobotPose, currentTarget));
      
      // Check if we've reached the target
      if (autopilot.atTarget(currentRobotPose, currentTarget)) {
        stopAutoPilot();
      }
    } else {
      Logger.recordOutput("AutoPilot/Active", false);
      lastAngleError = 0.0; // Reset derivative tracking when inactive
    }
  }
  
  // NEW: Override setOperatorPerspectiveForward to track when we manually set it
  @Override
  public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
    super.setOperatorPerspectiveForward(fieldDirection);
    operatorPerspectiveSetFromPose = true; // Lock CTRE's auto-setting
    
    Logger.recordOutput("Drive/OperatorPerspectiveLocked", true);
    Logger.recordOutput("Drive/OperatorPerspectiveForward", fieldDirection.getDegrees());
    
    System.out.println("[DriveSubsystem] Operator perspective manually set and locked to: " + fieldDirection.getDegrees() + " deg");
  }
  
  /**
   * Get the current "forward" direction for field-relative driving
   * Blue alliance: 0° (toward red wall)
   * Red alliance: 180° (toward blue wall)
   */
  private Rotation2d getOperatorPerspectiveForward() {
    // CTRE's internal state tracks this, but we need to expose it
    // For now, infer from alliance
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return Rotation2d.fromDegrees(180.0); // Red faces blue wall
    } else {
      return Rotation2d.fromDegrees(0.0); // Blue faces red wall (default)
    }
  }

  /**
   * Start AutoPilot navigation to a target pose
   * @param targetPose The pose to navigate to
   * @param constraints Motion constraints (max velocity, acceleration)
   */
  public void startAutoPilot(Pose2d targetPose, APConstraints constraints) {
    // Create profile with constraints and error tolerances
    APProfile profile = new APProfile(constraints)
        .withErrorXY(Meters.of(AUTOPILOT_ERROR_XY_METERS))
        .withErrorTheta(Degrees.of(AUTOPILOT_ERROR_THETA_DEGREES))
        .withBeelineRadius(Meters.of(AUTOPILOT_BEELINE_RADIUS_METERS));
    
    // Create AutoPilot instance with profile
    autopilot = new Autopilot(profile);
    
    // Create target from pose (using constructor that takes Pose2d)
    currentTarget = new APTarget(targetPose);
    
    // Reset derivative tracking
    lastAngleError = 0.0;
    
    autopilotActive = true;
    
    System.out.println("[AutoPilot] Started navigation to " + formatPose(targetPose));
    System.out.println("  Error tolerances: XY=" + AUTOPILOT_ERROR_XY_METERS + "m (" + 
                       (AUTOPILOT_ERROR_XY_METERS * 100) + "cm), Theta=" + 
                       AUTOPILOT_ERROR_THETA_DEGREES + "°");
    System.out.println("  Rotation PID: kP=" + AUTOPILOT_ROTATION_KP + ", kD=" + AUTOPILOT_ROTATION_KD);
    Logger.recordOutput("AutoPilot/Started", true);
  }

  /**
   * Stop AutoPilot navigation
   */
  public void stopAutoPilot() {
    autopilotActive = false;
    currentTarget = null;
    autopilot = null;
    lastAngleError = 0.0; // Reset derivative tracking
    
    // Stop the robot
    driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    
    System.out.println("[AutoPilot] Stopped");
    Logger.recordOutput("AutoPilot/Stopped", true);
  }

  /**
   * Check if AutoPilot is currently active
   */
  public boolean isAutoPilotActive() {
    return autopilotActive;
  }

  /**
   * Check if AutoPilot has reached its target
   */
  public boolean isAutoPilotAtTarget() {
    if (!autopilotActive || autopilot == null || currentTarget == null) {
      return false;
    }
    Pose2d currentPose = getState().Pose;
    return autopilot.atTarget(currentPose, currentTarget);
  }

  /**
   * Format pose for logging
   */
  private String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  /**
   * Drive using field-relative or robot-relative control
   */
  public void drive(
      double xVelocity,           // LeftY from RobotContainer
      double yVelocity,           // LeftX from RobotContainer
      double rotationalVelocity,  // RightX from RobotContainer
      boolean fieldRelative) {
    
    if (fieldRelative) {
      setControl(fieldCentricDrive
          .withVelocityX(xVelocity)              // ORIGINAL - don't swap!
          .withVelocityY(yVelocity)              // ORIGINAL - don't swap!
          .withRotationalRate(rotationalVelocity)); // ORIGINAL - don't swap!
    } else {
      setControl(robotCentricDrive
          .withVelocityX(xVelocity)              // ORIGINAL - don't swap!
          .withVelocityY(yVelocity)              // ORIGINAL - don't swap!
          .withRotationalRate(rotationalVelocity)); // ORIGINAL - don't swap!
    }
  }

  /**
   * Drive using robot-relative chassis speeds (for PathPlanner)
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Removed target state logging - now done in periodic()
    setControl(robotCentricDrive
        .withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  /**
   * Get robot-relative chassis speeds (for PathPlanner)
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return super.getKinematics().toChassisSpeeds(getState().ModuleStates);
  }

  /**
   * Get swerve module positions (for pose estimator)
   */
  public edu.wpi.first.math.kinematics.SwerveModulePosition[] getModulePositions() {
    return getState().ModulePositions;
  }

  // DON'T add getKinematics() - it's already inherited from CommandSwerveDrivetrain
  // The parent class already has it as public, so we can use it directly

  /**
   * Get gyro rotation (uses our hybrid QuestNav/Pigeon system)
   */
  public Rotation2d getGyroRotation() {
    return gyro.getRotation();
  }

  /**
   * Reset heading to zero
   */
  public void zeroHeading() {
    gyro.resetHeading();
  }

  /**
   * Create command to orient robot to field
   * This sets the CURRENT direction as "forward" (0°) for field-relative driving
   */
  public Command createOrientToFieldCommand(RobotState robotState) {
    return runOnce(() -> {
      // Reset gyro to 0°
      gyro.resetHeading();
      
      // Tell CTRE framework that current direction is now "operator forward"
      // This makes field-relative driving use this as the reference
      setOperatorPerspectiveForward(Rotation2d.fromDegrees(0.0));
      
      Logger.recordOutput("Drive/OrientToFieldButton", true);
      System.out.println("Field orientation set - current direction is now 0° (downfield)");
    });
  }

  // Add deadband to prevent wheel twitching at slow speeds
  private SwerveModuleState applySteeringDeadband(SwerveModuleState state) {
    // If speed is very slow, don't bother rotating wheels
    if (Math.abs(state.speedMetersPerSecond) < 0.05) { // Less than 5cm/s
      return new SwerveModuleState(0.0, state.angle); // Stop moving but keep current angle
    }
    return state;
  }

  /**
   * Apply deadband to prevent wheel twitching at very slow speeds
   */
  private SwerveModuleState[] applySteeringDeadband(SwerveModuleState[] states) {
    SwerveModuleState[] filteredStates = new SwerveModuleState[states.length];
    
    for (int i = 0; i < states.length; i++) {
      double speed = states[i].speedMetersPerSecond;
      Rotation2d angle = states[i].angle;
      
      // If speed is very slow (<3cm/s), don't rotate wheels - just hold current angle
      if (Math.abs(speed) < 0.03) {
        filteredStates[i] = new SwerveModuleState(0.0, angle); // Stop drive but keep angle
      } else {
        filteredStates[i] = states[i]; // Normal operation
      }
    }
    
    return filteredStates;
  }

  /**
   * SYSID: Test drive motors (translation)
   * Use this to characterize kS, kV, kA for driving forward/backward
   */
  public Command sysIdQuasistaticTranslation(SysIdRoutine.Direction direction) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            edu.wpi.first.units.Units.Volts.of(4),
            null,
            state -> {
                Logger.recordOutput("SysId/Translation/State", state.toString());
                System.out.println("SysId State: " + state.toString()); // NEW: Debug print
            }
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    ).quasistatic(direction);
  }

  public Command sysIdDynamicTranslation(SysIdRoutine.Direction direction) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            edu.wpi.first.units.Units.Volts.of(4),
            null,
            state -> {
                Logger.recordOutput("SysId/Translation/State", state.toString());
                System.out.println("SysId State: " + state.toString()); // NEW: Debug print
            }
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    ).dynamic(direction);
  }

  /**
   * SYSID: Test steering motors
   * Use this to characterize steering PID gains
   */
  public Command sysIdQuasistaticSteer(SysIdRoutine.Direction direction) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            edu.wpi.first.units.Units.Volts.of(7),
            null,
            state -> Logger.recordOutput("SysId/Steer/State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_steerCharacterization.withVolts(output)),
            null,
            this
        )
    ).quasistatic(direction);
  }

  public Command sysIdDynamicSteer(SysIdRoutine.Direction direction) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            edu.wpi.first.units.Units.Volts.of(7),
            null,
            state -> Logger.recordOutput("SysId/Steer/State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_steerCharacterization.withVolts(output)),
            null,
            this
        )
    ).dynamic(direction);
  }

  /**
   * SYSID: Test rotation (heading controller)
   * Use this to characterize rotation PID for field-centric facing angle
   */
  public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            edu.wpi.first.units.Units.Volts.of(Math.PI / 6).per(edu.wpi.first.units.Units.Second),
            edu.wpi.first.units.Units.Volts.of(Math.PI),
            null,
            state -> Logger.recordOutput("SysId/Rotation/State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(edu.wpi.first.units.Units.Volts)));
                Logger.recordOutput("SysId/Rotation/Rate", output.in(edu.wpi.first.units.Units.Volts));
            },
            null,
            this
        )
    ).quasistatic(direction);
  }

  public Command sysIdDynamicRotation(SysIdRoutine.Direction direction) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            edu.wpi.first.units.Units.Volts.of(Math.PI / 6).per(edu.wpi.first.units.Units.Second),
            edu.wpi.first.units.Units.Volts.of(Math.PI),
            null,
            state -> Logger.recordOutput("SysId/Rotation/State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(edu.wpi.first.units.Units.Volts)));
                Logger.recordOutput("SysId/Rotation/Rate", output.in(edu.wpi.first.units.Units.Volts));
            },
            null,
            this
        )
    ).dynamic(direction);
  }

  /**
   * Lock wheels in X-pattern to resist pushing/drift
   * Used after precision positioning to hold position
   */
  public void lockWheels() {
    // X-pattern: FL=45°, FR=-45°, BL=-45°, BR=45°
    // This makes robot very resistant to being pushed
    SwerveRequest.SwerveDriveBrake lockRequest = new SwerveRequest.SwerveDriveBrake();
    setControl(lockRequest);
    
    Logger.recordOutput("Drive/WheelsLocked", true);
  }
  
  /**
   * Normalize angle to [-180, +180] range for consistent graphing
   */
  private double normalizeAngle(double angleDegrees) {
    // Reduce to [-180, +180] range
    double normalized = angleDegrees % 360.0;
    
    if (normalized > 180.0) {
      normalized -= 360.0;
    } else if (normalized < -180.0) {
      normalized += 360.0;
    }
    
    return normalized;
  }
}
