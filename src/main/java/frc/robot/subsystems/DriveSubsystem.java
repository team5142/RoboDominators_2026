package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          FRONT_LEFT_LOCATION,
          FRONT_RIGHT_LOCATION,
          BACK_LEFT_LOCATION,
          BACK_RIGHT_LOCATION);

  private final GyroSubsystem gyro;

  private final SwerveModule frontLeft =
      new SwerveModule(
          FRONT_LEFT_DRIVE_ID,
          FRONT_LEFT_STEER_ID,
          FRONT_LEFT_CANCODER_ID,
          FRONT_LEFT_OFFSET_ROTATIONS * 2.0 * Math.PI,
          FRONT_LEFT_DRIVE_INVERTED,
          FRONT_LEFT_STEER_INVERTED);
  private final SwerveModule frontRight =
      new SwerveModule(
          FRONT_RIGHT_DRIVE_ID,
          FRONT_RIGHT_STEER_ID,
          FRONT_RIGHT_CANCODER_ID,
          FRONT_RIGHT_OFFSET_ROTATIONS * 2.0 * Math.PI,
          FRONT_RIGHT_DRIVE_INVERTED,
          FRONT_RIGHT_STEER_INVERTED);
  private final SwerveModule backLeft =
      new SwerveModule(
          BACK_LEFT_DRIVE_ID,
          BACK_LEFT_STEER_ID,
          BACK_LEFT_CANCODER_ID,
          BACK_LEFT_OFFSET_ROTATIONS * 2.0 * Math.PI,
          BACK_LEFT_DRIVE_INVERTED,
          BACK_LEFT_STEER_INVERTED);
  private final SwerveModule backRight =
      new SwerveModule(
          BACK_RIGHT_DRIVE_ID,
          BACK_RIGHT_STEER_ID,
          BACK_RIGHT_CANCODER_ID,
          BACK_RIGHT_OFFSET_ROTATIONS * 2.0 * Math.PI,
          BACK_RIGHT_DRIVE_INVERTED,
          BACK_RIGHT_STEER_INVERTED);

  private final RobotState robotState;

  public DriveSubsystem(RobotState robotState, GyroSubsystem gyro) {
    this.robotState = robotState;
    this.gyro = gyro;
  }

  @Override
  public void periodic() {
    // Just log sensor data, pose estimation happens elsewhere
    Logger.recordOutput("Drive/GyroYawDeg", getGyroRotation().getDegrees());
    Logger.recordOutput("Drive/ModuleStates", getModuleStates());
  }

  public void drive(
      Translation2d translation,
      double omegaRadPerSec,
      boolean fieldRelative,
      double speedScale) {

    double scaledX = translation.getX() * speedScale;
    double scaledY = translation.getY() * speedScale;
    double scaledOmega = omegaRadPerSec * speedScale;

    ChassisSpeeds speeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                scaledX, scaledY, scaledOmega, getGyroRotation())
            : new ChassisSpeeds(scaledX, scaledY, scaledOmega);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_TRANSLATION_SPEED_MPS);
    setModuleStates(states, true);
  }

  /**
   * Drive using robot-relative chassis speeds (for PathPlanner)
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_TRANSLATION_SPEED_MPS);
    setModuleStates(states, false);  // Use closed-loop for auto
  }

  /**
   * Get robot-relative chassis speeds (for PathPlanner)
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop) {
    frontLeft.setDesiredState(desiredStates[0], openLoop);
    frontRight.setDesiredState(desiredStates[1], openLoop);
    backLeft.setDesiredState(desiredStates[2], openLoop);
    backRight.setDesiredState(desiredStates[3], openLoop);
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      new SwerveModuleState(frontLeft.getPosition().distanceMeters, frontLeft.getAngle()),
      new SwerveModuleState(frontRight.getPosition().distanceMeters, frontRight.getAngle()),
      new SwerveModuleState(backLeft.getPosition().distanceMeters, backLeft.getAngle()),
      new SwerveModuleState(backRight.getPosition().distanceMeters, backRight.getAngle())
    };
  }

  public Rotation2d getHeading() {
    return robotState.getRobotPose().getRotation();
  }

  public void zeroHeading() {
    gyro.resetHeading();
  }

  public Rotation2d getGyroRotation() {
    return gyro.getRotation();
  }

  public Command createOrientToFieldCommand(RobotState robotState) {
    return runOnce(
        () -> {
          gyro.resetHeading();
          Logger.recordOutput("Drive/OrientToFieldButton", true);
        });
  }
}
