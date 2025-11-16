package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Swerve;

public class SwerveModule {
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder cancoder;
  private final double steerOffsetRadians;

  // Control requests
  private final VelocityVoltage driveVelocityControl = new VelocityVoltage(0);
  private final VoltageOut driveVoltageControl = new VoltageOut(0);
  private final PositionVoltage steerPositionControl = new PositionVoltage(0);

  private final boolean driveInverted;
  private final boolean steerInverted;

  public SwerveModule(
      int driveId,
      int steerId,
      int cancoderId,
      double steerOffsetRadians,
      boolean driveInverted,
      boolean steerInverted) {
    this.driveMotor = new TalonFX(driveId, Swerve.CAN_BUS_NAME);
    this.steerMotor = new TalonFX(steerId, Swerve.CAN_BUS_NAME);
    this.cancoder = new CANcoder(cancoderId, Swerve.CAN_BUS_NAME);
    this.steerOffsetRadians = steerOffsetRadians;
    this.driveInverted = driveInverted;
    this.steerInverted = steerInverted;

    configureMotors();
  }

  private void configureMotors() {
    // Configure drive motor
    TalonFXConfiguration driveConfig = Swerve.createDriveMotorConfig();
    driveConfig.MotorOutput.Inverted =
        driveInverted
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Feedback.SensorToMechanismRatio = Swerve.DRIVE_GEAR_RATIO;
    driveConfig.CurrentLimits.StatorCurrentLimit = Swerve.SLIP_CURRENT.in(edu.wpi.first.units.Units.Amps);
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Configure steer motor
    TalonFXConfiguration steerConfig = Swerve.createSteerMotorConfig();
    steerConfig.MotorOutput.Inverted =
        steerInverted
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    steerConfig.Feedback.SensorToMechanismRatio = Swerve.STEER_GEAR_RATIO;
    steerConfig.Feedback.RotorToSensorRatio = Swerve.COUPLING_RATIO;

    // Configure CANcoder
    CANcoderConfiguration cancoderConfig = Swerve.createCancoderConfig();
    cancoderConfig.MagnetSensor.MagnetOffset = steerOffsetRadians / (2.0 * Math.PI);

    // Apply configs with retries
    StatusCode driveStatus = StatusCode.StatusCodeNotInitialized;
    StatusCode steerStatus = StatusCode.StatusCodeNotInitialized;
    StatusCode cancoderStatus = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; i++) {
      driveStatus = driveMotor.getConfigurator().apply(driveConfig);
      steerStatus = steerMotor.getConfigurator().apply(steerConfig);
      cancoderStatus = cancoder.getConfigurator().apply(cancoderConfig);
      if (driveStatus.isOK() && steerStatus.isOK() && cancoderStatus.isOK()) break;
    }

    if (!driveStatus.isOK() || !steerStatus.isOK() || !cancoderStatus.isOK()) {
      System.err.println(
          "Failed to configure swerve module: Drive="
              + driveStatus
              + " Steer="
              + steerStatus
              + " CANcoder="
              + cancoderStatus);
    }
  }

  public void setDesiredState(SwerveModuleState state, boolean openLoop) {
    // Prevent jitter when commanded speed is near zero
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      driveMotor.setControl(driveVoltageControl.withOutput(0.0));
      return;
    }

    // Optimize state to avoid spinning more than 90 degrees
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Convert speed from m/s to rotations/sec for the wheel
    double wheelRotationsPerSec =
        optimizedState.speedMetersPerSecond / (2.0 * Math.PI * Swerve.WHEEL_RADIUS_METERS);

    if (openLoop) {
      // Open loop: direct voltage control proportional to desired speed
      double percentOutput = wheelRotationsPerSec / (Swerve.MAX_TRANSLATION_SPEED_MPS / (2.0 * Math.PI * Swerve.WHEEL_RADIUS_METERS));
      driveMotor.setControl(driveVoltageControl.withOutput(percentOutput * 12.0));
    } else {
      // Closed loop: velocity control
      driveMotor.setControl(driveVelocityControl.withVelocity(wheelRotationsPerSec));
    }

    // Steer control - convert angle to rotations for motor position
    double steerRotations = optimizedState.angle.getRotations();
    steerMotor.setControl(steerPositionControl.withPosition(steerRotations));
  }

  public SwerveModulePosition getPosition() {
    // Get drive position in rotations, convert to meters
    double driveRotations = driveMotor.getPosition().getValueAsDouble();
    double distanceMeters = driveRotations * (2.0 * Math.PI * Swerve.WHEEL_RADIUS_METERS);

    return new SwerveModulePosition(distanceMeters, getAngle());
  }

  public Rotation2d getAngle() {
    // Steer motor already has CANcoder fused with offset applied via config
    return Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble());
  }
}
