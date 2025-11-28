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

/**
 * Swerve drivetrain extending CTRE's CommandSwerveDrivetrain.
 * Integrates with custom GyroSubsystem for QuestNav/Pigeon failover.
 */
public class DriveSubsystem extends CommandSwerveDrivetrain {
  private final GyroSubsystem gyro;
  private final RobotState robotState;
  
  // Swerve requests for different drive modes
  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

  // SysId requests (directly from parent class)
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

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
  }

  @Override
  public void periodic() {
    super.periodic(); // CTRE's periodic (handles odometry, operator perspective, etc.)
    
    // Additional logging
    Logger.recordOutput("Drive/GyroYawDeg", getGyroRotation().getDegrees());
    Logger.recordOutput("Drive/Pose", getState().Pose);
  }

  /**
   * Drive using field-relative or robot-relative control
   */
  public void drive(
      double xVelocity,
      double yVelocity,
      double rotationalVelocity,
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

  /**
   * Drive using robot-relative chassis speeds (for PathPlanner)
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
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
}
