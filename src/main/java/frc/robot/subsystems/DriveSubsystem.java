package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

// Swerve drivetrain subsystem - extends CTRE's CommandSwerveDrivetrain for low-level control
// Adds custom gyro integration (QuestNav/Pigeon failover), SysId routines, and helper methods
// This is the main interface for driving the robot in teleop and autonomous
public class DriveSubsystem extends CommandSwerveDrivetrain {
  private final GyroSubsystem gyro; // Hybrid QuestNav/Pigeon gyro with automatic failover
  private final RobotState robotState; // Global state tracker
  
  // Swerve drive requests - CTRE's way of commanding robot motion
  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric(); // Joystick relative to field
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric(); // Joystick relative to robot

  // SysId requests (inherited from parent class) - used for motor characterization
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  // Constructs drivetrain using Tuner X generated constants (auto-generated code)
  public DriveSubsystem(RobotState robotState, GyroSubsystem gyro) {
    super( // Call parent constructor with Tuner X module configs
        frc.robot.generated.TunerConstants.DrivetrainConstants,
        frc.robot.generated.TunerConstants.FrontLeft,
        frc.robot.generated.TunerConstants.FrontRight,
        frc.robot.generated.TunerConstants.BackLeft,
        frc.robot.generated.TunerConstants.BackRight
    );
    
    this.robotState = robotState;
    this.gyro = gyro;
  }

  @Override
  public void periodic() {
    super.periodic(); // CTRE's periodic (handles odometry, CAN updates, etc.)
    
    // Log gyro and pose for debugging
    Logger.recordOutput("Drive/GyroYawDeg", getGyroRotation().getDegrees());
    Logger.recordOutput("Drive/Pose", getState().Pose);
  }

  // Main drive method - converts joystick inputs to robot motion
  // fieldRelative: true = joystick forward always moves downfield (preferred)
  //                false = joystick forward moves in robot's current direction
  public void drive(
      double xVelocity, // Forward/back speed (m/s)
      double yVelocity, // Left/right speed (m/s)
      double rotationalVelocity, // Rotation speed (rad/s)
      boolean fieldRelative) {
    
    if (fieldRelative) {
      setControl(fieldCentricDrive
          .withVelocityX(xVelocity)
          .withVelocityY(yVelocity)
          .withRotationalRate(rotationalVelocity));
    } else {
      setControl(robotCentricDrive
          .withVelocityX(xVelocity)
          .withVelocityY(yVelocity)
          .withRotationalRate(rotationalVelocity));
    }
  }

  // Drive using robot-relative chassis speeds - used by PathPlanner for autonomous
  public void driveRobotRelative(ChassisSpeeds speeds) {
    setControl(robotCentricDrive
        .withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  // Get current robot speed - used by PathPlanner for path following
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return super.getKinematics().toChassisSpeeds(getState().ModuleStates);
  }

  // Get swerve module positions - used by pose estimator for odometry
  public edu.wpi.first.math.kinematics.SwerveModulePosition[] getModulePositions() {
    return getState().ModulePositions;
  }

  // Get current gyro rotation - uses our hybrid QuestNav/Pigeon system
  public Rotation2d getGyroRotation() {
    return gyro.getRotation();
  }

  // Reset gyro heading to zero
  public void zeroHeading() {
    gyro.resetHeading();
  }

  // Create command to orient robot to field - sets current direction as "forward" (0deg)
  // This is the "field orient" button - after pressing, joystick forward always points downfield
  public Command createOrientToFieldCommand(RobotState robotState) {
    return runOnce(() -> {
      gyro.resetHeading(); // Reset gyro to 0deg
      
      // Tell CTRE framework that current direction is now "operator forward"
      setOperatorPerspectiveForward(Rotation2d.fromDegrees(0.0));
      
      Logger.recordOutput("Drive/OrientToFieldButton", true);
      System.out.println("Field orientation set - current direction is now 0° (downfield)");
    });
  }

  // Apply deadband to prevent wheel micro-movements at very slow speeds
  // When robot is barely moving (<3cm/s), don't rotate wheels - just hold current angle
  private SwerveModuleState[] applySteeringDeadband(SwerveModuleState[] states) {
    SwerveModuleState[] filteredStates = new SwerveModuleState[states.length];
    
    for (int i = 0; i < states.length; i++) {
      double speed = states[i].speedMetersPerSecond;
      Rotation2d angle = states[i].angle;
      
      // If speed is very slow (<3cm/s), don't rotate wheels - just hold current angle
      if (Math.abs(speed) < 0.03) {
        filteredStates[i] = new SwerveModuleState(0.0, angle);
      } else {
        filteredStates[i] = states[i];
      }
    }
    
    return filteredStates;
  }

  // SysId: Characterize drive motors (translation) - finds kS, kV, kA for driving
  // Run Quasistatic (slow ramp) and Dynamic (fast step) in both directions
  // Logs data to .hoot file for analysis in SysId tool
  public Command sysIdQuasistaticTranslation(SysIdRoutine.Direction direction) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default ramp rate
            edu.wpi.first.units.Units.Volts.of(4), // Max voltage (safe)
            null,
            state -> {
                Logger.recordOutput("SysId/Translation/State", state.toString());
                System.out.println("SysId State: " + state.toString());
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
                System.out.println("SysId State: " + state.toString());
            }
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    ).dynamic(direction);
  }

  // SysId: Characterize steering motors - finds steering PID gains
  public Command sysIdQuasistaticSteer(SysIdRoutine.Direction direction) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            edu.wpi.first.units.Units.Volts.of(7), // Higher voltage OK for steering (lighter motors)
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

  // SysId: Characterize rotation heading controller - finds rotation PID for auto-rotation features
  // Note: Uses rad/s but SysId framework expects "volts" - we pretend volts = rad/s
  public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            edu.wpi.first.units.Units.Volts.of(Math.PI / 6).per(edu.wpi.first.units.Units.Second), // Ramp rate (rad/s^2)
            edu.wpi.first.units.Units.Volts.of(Math.PI), // Max rate (rad/s)
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
}
