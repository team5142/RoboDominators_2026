package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;
import org.littletonrobotics.junction.Logger;

// Swerve drivetrain - extends CTRE's CommandSwerveDrivetrain with GyroSubsystem integration.
// AutoPilot logic lives in commands, not here.
public class DriveSubsystem extends CommandSwerveDrivetrain {
  private final GyroSubsystem gyro;
  private final RobotState robotState;
  
  // Swerve requests for different drive modes
  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

  private int logCounter = 0;

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
    super.periodic();
    
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

    // Log module states every 5th cycle (not every cycle)
    if (logCounter % 5 == 0) {
      SwerveModuleState[] moduleStates = getState().ModuleStates;
      Logger.recordOutput("Drive/ModuleStates/FrontLeft/Angle", normalizeAngle(moduleStates[0].angle.getDegrees()));
      Logger.recordOutput("Drive/ModuleStates/FrontLeft/Speed", moduleStates[0].speedMetersPerSecond);
      Logger.recordOutput("Drive/ModuleStates/FrontRight/Angle", normalizeAngle(moduleStates[1].angle.getDegrees()));
      Logger.recordOutput("Drive/ModuleStates/FrontRight/Speed", moduleStates[1].speedMetersPerSecond);
      Logger.recordOutput("Drive/ModuleStates/BackLeft/Angle", normalizeAngle(moduleStates[2].angle.getDegrees()));
      Logger.recordOutput("Drive/ModuleStates/BackLeft/Speed", moduleStates[2].speedMetersPerSecond);
      Logger.recordOutput("Drive/ModuleStates/BackRight/Angle", normalizeAngle(moduleStates[3].angle.getDegrees()));
      Logger.recordOutput("Drive/ModuleStates/BackRight/Speed", moduleStates[3].speedMetersPerSecond);
    }
    
    // Log module positions
    var modulePositions = getModulePositions();
    Logger.recordOutput("Drive/ModulePositions/FrontLeft", modulePositions[0].distanceMeters);
    Logger.recordOutput("Drive/ModulePositions/FrontRight", modulePositions[1].distanceMeters);
    Logger.recordOutput("Drive/ModulePositions/BackLeft", modulePositions[2].distanceMeters);
    Logger.recordOutput("Drive/ModulePositions/BackRight", modulePositions[3].distanceMeters);

    logCounter++;
  }
  
  // Resets Pigeon yaw to 0.
  public void resetGyroToZero() {
    gyro.setHeading(0.0);
  }

  // Seeds the Pigeon to a specific heading so CTRE field-centric math uses the correct offset.
  public void setGyroHeading(Rotation2d heading) {
    gyro.setHeading(heading.getDegrees());
  }

  @Override
  public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
    super.setOperatorPerspectiveForward(fieldDirection);
    // Mark perspective as applied so CommandSwerveDrivetrain.periodic() won't overwrite it.
    m_hasAppliedOperatorPerspective = true;
    
    Logger.recordOutput("Drive/OperatorPerspectiveLocked", true);
    Logger.recordOutput("Drive/OperatorPerspectiveForward", fieldDirection.getDegrees());
    
    SmartLogger.logConsole("Operator perspective locked to: " + fieldDirection.getDegrees() + " deg", "Drive");
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

  public double getGyroPitchDegrees() {
    return gyro.getPitchDegrees();
  }

  public double getGyroRollDegrees() {
    return gyro.getRollDegrees();
  }

  public void zeroHeading() {
    gyro.resetHeading();
  }

  public Command createOrientToFieldCommand(RobotState robotState) {
    return runOnce(() -> {
      // Set operator perspective to face downfield for the current alliance.
      // Blue downfield = 0 deg (toward Red wall), Red downfield = 180 deg (toward Blue wall).
      // Does NOT reset the gyro - pose estimator state is unaffected.
      boolean isRed = DriverStation.getAlliance()
          .map(a -> a == DriverStation.Alliance.Red).orElse(false);
      Rotation2d downfield = Rotation2d.fromDegrees(isRed ? 180.0 : 0.0);
      setOperatorPerspectiveForward(downfield);
      SmartLogger.logConsole("[Drive] Field orientation reset - downfield is now "
          + downfield.getDegrees() + " deg", "Drive");
    });
  }

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

  // SysId characterization commands - wrap base class routines with explicit test selection
  public Command sysIdQuasistaticTranslation(SysIdRoutine.Direction direction) {
    return runOnce(() -> selectTranslationRoutine())
      .andThen(sysIdQuasistatic(direction));
  }

  public Command sysIdDynamicTranslation(SysIdRoutine.Direction direction) {
    return runOnce(() -> selectTranslationRoutine())
      .andThen(sysIdDynamic(direction));
  }

  public Command sysIdQuasistaticSteer(SysIdRoutine.Direction direction) {
    return runOnce(() -> selectSteerRoutine())
      .andThen(sysIdQuasistatic(direction));
  }

  public Command sysIdDynamicSteer(SysIdRoutine.Direction direction) {
    return runOnce(() -> selectSteerRoutine())
      .andThen(sysIdDynamic(direction));
  }

  public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
    return runOnce(() -> selectRotationRoutine())
      .andThen(sysIdQuasistatic(direction));
  }

  public Command sysIdDynamicRotation(SysIdRoutine.Direction direction) {
    return runOnce(() -> selectRotationRoutine())
      .andThen(sysIdDynamic(direction));
  }

  private void selectTranslationRoutine() {
    // Access inherited field to switch active routine
    try {
      var field = CommandSwerveDrivetrain.class.getDeclaredField("m_sysIdRoutineToApply");
      field.setAccessible(true);
      var translationRoutine = CommandSwerveDrivetrain.class.getDeclaredField("m_sysIdRoutineTranslation");
      translationRoutine.setAccessible(true);
      field.set(this, translationRoutine.get(this));
    } catch (Exception e) {
      SmartLogger.logConsoleError("[Drive] Failed to select translation SysId routine: " + e.getMessage());
    }
  }

  private void selectSteerRoutine() {
    try {
      var field = CommandSwerveDrivetrain.class.getDeclaredField("m_sysIdRoutineToApply");
      field.setAccessible(true);
      var steerRoutine = CommandSwerveDrivetrain.class.getDeclaredField("m_sysIdRoutineSteer");
      steerRoutine.setAccessible(true);
      field.set(this, steerRoutine.get(this));
    } catch (Exception e) {
      SmartLogger.logConsoleError("[Drive] Failed to select steer SysId routine: " + e.getMessage());
    }
  }

  private void selectRotationRoutine() {
    try {
      var field = CommandSwerveDrivetrain.class.getDeclaredField("m_sysIdRoutineToApply");
      field.setAccessible(true);
      var rotationRoutine = CommandSwerveDrivetrain.class.getDeclaredField("m_sysIdRoutineRotation");
      rotationRoutine.setAccessible(true);
      field.set(this, rotationRoutine.get(this));
    } catch (Exception e) {
      SmartLogger.logConsoleError("[Drive] Failed to select rotation SysId routine: " + e.getMessage());
    }
  }

  // Placeholder - CANcoder fusion cannot be toggled via API.
  // Before steer SysId: use Phoenix Tuner X to set CANcoder update freq to 0Hz,
  // OR change STEER_FEEDBACK_TYPE to SyncCANcoder in Constants.java, then reboot.
  public void disableCANcoderFusion() {
    SmartLogger.logConsole("CANcoder fusion disable not implemented - use Phoenix Tuner X manually", "Drive");
  }

  public void enableCANcoderFusion() {
    SmartLogger.logConsole("CANcoder fusion enable not implemented - restore via Phoenix Tuner X and reboot", "Drive");
  }
}
