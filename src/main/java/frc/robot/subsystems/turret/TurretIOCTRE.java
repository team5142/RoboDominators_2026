package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

// CTRE hardware implementation for the turret mechanism.
// All motor inversion is controlled via Constants.Turret.*_INVERTED booleans -
// flip those flags rather than changing wiring.
public class TurretIOCTRE implements TurretIO {
  private final TalonFX flywheelFrontMotor;
  private final TalonFX flywheelBackMotor;
  private final TalonFX hoodMotor;
  private final TalonFX turretMotor;

  private final DigitalInput hoodLimitSwitch; // fires at bottom stop (85 deg = position 0)
  private final DigitalInput hallCCW;

  private final DutyCycleOut flywheelFrontDutyCycle  = new DutyCycleOut(0.0);
  private final DutyCycleOut flywheelBackDutyCycle   = new DutyCycleOut(0.0);
  private final VoltageOut   flywheelFrontVoltageOut = new VoltageOut(0.0);
  private final VoltageOut   flywheelBackVoltageOut  = new VoltageOut(0.0);
  private final VelocityVoltage flywheelFrontVelocity = new VelocityVoltage(0.0).withSlot(0);
  private final VelocityVoltage flywheelBackVelocity  = new VelocityVoltage(0.0).withSlot(0);
  private final DutyCycleOut hoodDutyCycle          = new DutyCycleOut(0.0);
  private final MotionMagicVoltage hoodMotionMagic  = new MotionMagicVoltage(0.0).withSlot(0);
  private final DutyCycleOut turretDutyCycle        = new DutyCycleOut(0.0);
  private final VoltageOut   turretVoltageOut  = new VoltageOut(0.0);
  private final MotionMagicVoltage turretMotionMagic = new MotionMagicVoltage(0.0).withSlot(0);

  // Pre-subscribed sticky fault — latches true when motor boots while robot is enabled
  private com.ctre.phoenix6.StatusSignal<Boolean> bootDuringEnSignal;

  public TurretIOCTRE() {
    flywheelFrontMotor = new TalonFX(Constants.Turret.FLYWHEEL_FRONT_MOTOR_ID, new CANBus(Constants.Swerve.CAN_BUS_NAME));
    flywheelBackMotor  = new TalonFX(Constants.Turret.FLYWHEEL_BACK_MOTOR_ID,  new CANBus(Constants.Swerve.CAN_BUS_NAME));
    hoodMotor          = new TalonFX(Constants.Turret.HOOD_MOTOR_ID,           new CANBus(Constants.Swerve.CAN_BUS_NAME));

    // Flywheel motors: coast mode, separate inversion flags (back motor runs opposite to front)
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.CurrentLimits.StatorCurrentLimit       = 40.0;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit       = 30.0;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Slot 0: velocity feedforward + P gain from SysId 2026-03-07
    flywheelConfig.Slot0.kS = Constants.Turret.FLYWHEEL_FRONT_KS;
    flywheelConfig.Slot0.kV = Constants.Turret.FLYWHEEL_FRONT_KV;
    flywheelConfig.Slot0.kA = Constants.Turret.FLYWHEEL_FRONT_KA;
    flywheelConfig.Slot0.kP = Constants.Turret.FLYWHEEL_FRONT_KP;
    flywheelConfig.MotorOutput.Inverted = Constants.Turret.FLYWHEEL_FRONT_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    flywheelFrontMotor.getConfigurator().apply(flywheelConfig);
    flywheelConfig.Slot0.kS = Constants.Turret.FLYWHEEL_BACK_KS;
    flywheelConfig.Slot0.kV = Constants.Turret.FLYWHEEL_BACK_KV;
    flywheelConfig.Slot0.kA = Constants.Turret.FLYWHEEL_BACK_KA;
    flywheelConfig.Slot0.kP = Constants.Turret.FLYWHEEL_BACK_KP;
    flywheelConfig.Slot0.kD = Constants.Turret.FLYWHEEL_BACK_KD;
    flywheelConfig.MotorOutput.Inverted = Constants.Turret.FLYWHEEL_BACK_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    flywheelBackMotor.getConfigurator().apply(flywheelConfig);

    // Hood motor: MotionMagic position control, brake mode
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = Constants.Turret.HOOD_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    hoodConfig.CurrentLimits.StatorCurrentLimit       = 40.0;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.Slot0.kS = Constants.Turret.HOOD_KS;
    hoodConfig.Slot0.kV = Constants.Turret.HOOD_KV;
    hoodConfig.Slot0.kP = Constants.Turret.HOOD_KP;
    MotionMagicConfigs hoodMM = new MotionMagicConfigs();
    hoodMM.MotionMagicCruiseVelocity = Constants.Turret.HOOD_CRUISE_VELOCITY_RPS;
    hoodMM.MotionMagicAcceleration   = Constants.Turret.HOOD_ACCELERATION_RPS2;
    hoodConfig.MotionMagic = hoodMM;
    hoodMotor.getConfigurator().apply(hoodConfig);

    turretMotor = new TalonFX(Constants.Turret.TURRET_MOTOR_ID, new CANBus(Constants.Swerve.CAN_BUS_NAME));
    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    MotorOutputConfigs turretOutput = new MotorOutputConfigs();
    turretOutput.Inverted = Constants.Turret.TURRET_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turretOutput.NeutralMode = NeutralModeValue.Brake;
    turretConfig.MotorOutput = turretOutput;
    turretConfig.CurrentLimits.StatorCurrentLimit = 40.0; // raised from 20A — energy chain drag requires more torque
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Disable limit-hit beep on the turret motor — soft limits are enforced in software
    AudioConfigs turretAudio = new AudioConfigs();
    turretAudio.BeepOnBoot = false;
    turretAudio.BeepOnConfig = false;
    turretAudio.AllowMusicDurDisable = false;
    turretConfig.Audio = turretAudio;

    // Slot 0: MotionMagic PID + feedforward gains — tune kP in AdvantageScope
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kS = Constants.Turret.TURRET_KS;
    slot0.kV = Constants.Turret.TURRET_KV;
    slot0.kP = Constants.Turret.TURRET_KP;
    slot0.kI = Constants.Turret.TURRET_KI;
    slot0.kD = Constants.Turret.TURRET_KD;
    turretConfig.Slot0 = slot0;

    // MotionMagic profile — safety speeds for commissioning; raise in Constants once confirmed
    MotionMagicConfigs mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = Constants.Turret.TURRET_CRUISE_VELOCITY_RPS;
    mm.MotionMagicAcceleration   = Constants.Turret.TURRET_ACCELERATION_RPS2;
    mm.MotionMagicJerk           = Constants.Turret.TURRET_JERK_RPS3;
    turretConfig.MotionMagic = mm;

    turretMotor.getConfigurator().apply(turretConfig);

    // Raise position and velocity signal rates for SysId data quality
    BaseStatusSignal.setUpdateFrequencyForAll(100,
        turretMotor.getPosition(), turretMotor.getVelocity());
    turretMotor.optimizeBusUtilization();

    // Subscribe sticky fault at 50Hz — checked each loop to detect mid-match motor reboot
    bootDuringEnSignal = turretMotor.getStickyFault_BootDuringEnable();
    bootDuringEnSignal.setUpdateFrequency(50);

    hoodLimitSwitch = new DigitalInput(Constants.Turret.HOOD_LIMIT_SWITCH_DIO);
    hallCCW         = new DigitalInput(Constants.Turret.HALL_SENSOR_CCW_DIO);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.hoodLimitSwitchRaw = hoodLimitSwitch.get();
    inputs.hallCCWRaw         = !hallCCW.get(); // active-low — true when magnet is sensed

    inputs.hoodMotorPositionRotations = hoodMotor.getPosition().getValueAsDouble();
    inputs.hoodMotorCurrentAmps       = hoodMotor.getStatorCurrent().getValueAsDouble();

    inputs.turretAbsolutePositionRotations = turretMotor.getPosition().getValueAsDouble();
    inputs.turretVelocityRps               = turretMotor.getVelocity().getValueAsDouble();
    inputs.turretMotorCurrentAmps          = turretMotor.getStatorCurrent().getValueAsDouble();

    double frontRps = flywheelFrontMotor.getVelocity().getValueAsDouble();
    inputs.flywheelVelocityRpm = frontRps * 60.0;

    double backRps = flywheelBackMotor.getVelocity().getValueAsDouble();
    inputs.flywheelBackVelocityRpm = backRps * 60.0;
    // Read and immediately clear the sticky fault so it fires for exactly one loop
    bootDuringEnSignal.refresh();
    inputs.turretBootDuringEn = bootDuringEnSignal.getValue();
    if (inputs.turretBootDuringEn) {
      turretMotor.clearStickyFault_BootDuringEnable();
    }
  }

  @Override
  public void setFlywheelPercent(double percent) {
    flywheelFrontMotor.setControl(flywheelFrontDutyCycle.withOutput(percent));
    flywheelBackMotor.setControl(flywheelBackDutyCycle.withOutput(percent));
  }

  @Override
  public void setFlywheelFrontPercent(double percent) {
    flywheelFrontMotor.setControl(flywheelFrontDutyCycle.withOutput(percent));
  }

  @Override
  public void setFlywheelBackPercent(double percent) {
    flywheelBackMotor.setControl(flywheelBackDutyCycle.withOutput(percent));
  }

  @Override
  public void setFlywheelFrontVoltage(double volts) {
    flywheelFrontMotor.setControl(flywheelFrontVoltageOut.withOutput(volts));
  }

  @Override
  public void setFlywheelBackVoltage(double volts) {
    flywheelBackMotor.setControl(flywheelBackVoltageOut.withOutput(volts));
  }

  @Override
  public void setFlywheelFrontRps(double rps) {
    flywheelFrontMotor.setControl(flywheelFrontVelocity.withVelocity(rps));
  }

  @Override
  public void setFlywheelBackRps(double rps) {
    flywheelBackMotor.setControl(flywheelBackVelocity.withVelocity(rps));
  }

  @Override
  public void setHoodPercent(double percent) {
    hoodMotor.setControl(hoodDutyCycle.withOutput(percent));
  }

  @Override
  public void setHoodPosition(double motorRotations) {
    hoodMotor.setControl(hoodMotionMagic.withPosition(motorRotations));
  }

  @Override
  public void setTurretPercent(double percent) {
    turretMotor.setControl(turretDutyCycle.withOutput(percent));
  }

  @Override
  public void setTurretVoltage(double volts) {
    turretMotor.setControl(turretVoltageOut.withOutput(volts));
  }

  @Override
  public void setTurretPosition(double motorRotations) {
    turretMotor.setControl(turretMotionMagic.withPosition(motorRotations));
  }

  // During SysId the normal 20A stator limit would clamp the motor before it can accelerate
  // freely, corrupting kA. Raise it to 60A for the duration of the test, then restore.
  @Override
  public void setSysIdActive(boolean active) {
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
    limits.StatorCurrentLimit       = active ? 60.0 : 20.0;
    limits.StatorCurrentLimitEnable = true;
    turretMotor.getConfigurator().apply(limits);
  }

  @Override
  public void zeroTurretEncoder() {
    // Set to the hall sensor offset so the encoder reads 0 at the CCW hard stop.
    // When mechanical relocates the sensor to the CCW stop, set TURRET_HALL_OFFSET_MOTOR_ROT = 0.
    turretMotor.setPosition(0.0);
  }

  @Override
  public void restoreTurretEncoder(double motorRotations) {
    turretMotor.setPosition(motorRotations);
  }

  @Override
  public void zeroHoodEncoder() {
    hoodMotor.setPosition(Constants.Turret.HOOD_HOME_ROTATIONS);
  }
}
