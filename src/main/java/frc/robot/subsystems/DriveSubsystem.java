package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
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
}
