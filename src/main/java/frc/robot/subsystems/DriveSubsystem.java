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
import frc.robot.TunableCTREGains;
import org.littletonrobotics.junction.Logger;

/**
 * Swerve drivetrain extending CTRE's CommandSwerveDrivetrain.
 * Integrates with custom GyroSubsystem for QuestNav/Pigeon failover.
 * 
 * DOES NOT contain AutoPilot logic - that lives in commands!
 */
public class DriveSubsystem extends CommandSwerveDrivetrain {
  private final GyroSubsystem gyro;
  private final RobotState robotState;
  
  private boolean operatorPerspectiveSetFromPose = false;
  
  // Swerve requests for different drive modes
  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

  // SysId requests
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  public DriveSubsystem(RobotState robotState, GyroSubsystem gyro) {
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
    if (!operatorPerspectiveSetFromPose) {
      super.periodic();
    }
    
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    
    // Log chassis speeds
    Logger.recordOutput("Drive/ChassisSpeed/VX", speeds.vxMetersPerSecond);
    Logger.recordOutput("Drive/ChassisSpeed/VY", speeds.vyMetersPerSecond);
    Logger.recordOutput("Drive/ChassisSpeed/Omega", speeds.omegaRadiansPerSecond);
    
    double translationSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    Logger.recordOutput("Drive/ChassisSpeed/TranslationMagnitude", translationSpeed);
    
    Logger.recordOutput("Drive/GyroYawDeg", getGyroRotation().getDegrees());
    Logger.recordOutput("Drive/Pose", getState().Pose);
    
    Rotation2d operatorForward = getOperatorPerspectiveForward();
    Pose2d currentPose = robotState.getRobotPose();
    Pose2d fieldForwardPose = new Pose2d(currentPose.getTranslation(), operatorForward);
    
    Logger.recordOutput("Drive/FieldForwardDirection", fieldForwardPose);
    Logger.recordOutput("Drive/FieldForwardDegrees", operatorForward.getDegrees());
    SmartDashboard.putNumber("Drive/FieldForward", operatorForward.getDegrees());
    
    // Log module states
    SwerveModuleState[] moduleStates = getState().ModuleStates;
    Logger.recordOutput("Drive/ModuleStates/FrontLeft/Angle", normalizeAngle(moduleStates[0].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleStates/FrontLeft/Speed", moduleStates[0].speedMetersPerSecond);
    Logger.recordOutput("Drive/ModuleStates/FrontRight/Angle", normalizeAngle(moduleStates[1].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleStates/FrontRight/Speed", moduleStates[1].speedMetersPerSecond);
    Logger.recordOutput("Drive/ModuleStates/BackLeft/Angle", normalizeAngle(moduleStates[2].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleStates/BackLeft/Speed", moduleStates[2].speedMetersPerSecond);
    Logger.recordOutput("Drive/ModuleStates/BackRight/Angle", normalizeAngle(moduleStates[3].angle.getDegrees()));
    Logger.recordOutput("Drive/ModuleStates/BackRight/Speed", moduleStates[3].speedMetersPerSecond);
    
    // Log module positions
    var modulePositions = getModulePositions();
    Logger.recordOutput("Drive/ModulePositions/FrontLeft", modulePositions[0].distanceMeters);
    Logger.recordOutput("Drive/ModulePositions/FrontRight", modulePositions[1].distanceMeters);
    Logger.recordOutput("Drive/ModulePositions/BackLeft", modulePositions[2].distanceMeters);
    Logger.recordOutput("Drive/ModulePositions/BackRight", modulePositions[3].distanceMeters);

    // Live-update CTRE gains
    if (TunableCTREGains.hasChanged()) {
      System.out.println("CTRE gains updated from AdvantageScope:");
      System.out.println("  Steer: kP=" + TunableCTREGains.STEER_KP.get() + " kD=" + TunableCTREGains.STEER_KD.get());
      System.out.println("  Drive: kP=" + TunableCTREGains.DRIVE_KP.get() + " kV=" + TunableCTREGains.DRIVE_KV.get());
    }
  }
  
  @Override
  public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
    super.setOperatorPerspectiveForward(fieldDirection);
    operatorPerspectiveSetFromPose = true;
    
    Logger.recordOutput("Drive/OperatorPerspectiveLocked", true);
    Logger.recordOutput("Drive/OperatorPerspectiveForward", fieldDirection.getDegrees());
    
    System.out.println("[DriveSubsystem] Operator perspective manually set and locked to: " + fieldDirection.getDegrees() + " deg");
  }
  
  private Rotation2d getOperatorPerspectiveForward() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return Rotation2d.fromDegrees(180.0);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }

  public void drive(double xVelocity, double yVelocity, double rotationalVelocity, boolean fieldRelative) {
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

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setControl(robotCentricDrive
        .withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return super.getKinematics().toChassisSpeeds(getState().ModuleStates);
  }

  public edu.wpi.first.math.kinematics.SwerveModulePosition[] getModulePositions() {
    return getState().ModulePositions;
  }

  public Rotation2d getGyroRotation() {
    return gyro.getRotation();
  }

  public void zeroHeading() {
    gyro.resetHeading();
  }

  public Command createOrientToFieldCommand(RobotState robotState) {
    return runOnce(() -> {
      gyro.resetHeading();
      setOperatorPerspectiveForward(Rotation2d.fromDegrees(0.0));
      Logger.recordOutput("Drive/OrientToFieldButton", true);
      System.out.println("Field orientation set - current direction is now 0° (downfield)");
    });
  }

  // ...existing SysId methods...
  
  public void lockWheels() {
    SwerveRequest.SwerveDriveBrake lockRequest = new SwerveRequest.SwerveDriveBrake();
    setControl(lockRequest);
    Logger.recordOutput("Drive/WheelsLocked", true);
  }
  
  private double normalizeAngle(double angleDegrees) {
    double normalized = angleDegrees % 360.0;
    if (normalized > 180.0) normalized -= 360.0;
    else if (normalized < -180.0) normalized += 360.0;
    return normalized;
  }

  // ...existing SysId command methods (sysIdQuasistaticTranslation, etc.)...
}
