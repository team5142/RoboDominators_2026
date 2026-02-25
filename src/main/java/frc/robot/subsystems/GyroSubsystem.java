package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import frc.robot.util.SmartLogger;

// Pigeon2 IMU for robot heading. Provides yaw/pitch/roll and always-available heading
// (does not lose tracking like QuestNav during power cycles).
public class GyroSubsystem extends SubsystemBase {
  private final Pigeon2 pigeon;

  public GyroSubsystem() {
    pigeon = new Pigeon2(PIGEON_CAN_ID, new CANBus(CAN_BUS_NAME));
    pigeon.reset();
    SmartLogger.logConsole("Pigeon2 initialized on CAN ID: " + PIGEON_CAN_ID, "Gyro");
  }

  @Override
  public void periodic() {
    Rotation2d rotation = getRotation();
    
    Logger.recordOutput("Gyro/Rotation", rotation);
    Logger.recordOutput("Gyro/RotationDegrees", rotation.getDegrees());
    Logger.recordOutput("Gyro/RotationRadians", rotation.getRadians());
    Logger.recordOutput("Gyro/PigeonYawDegrees", pigeon.getYaw().getValueAsDouble());
    Logger.recordOutput("Gyro/PigeonPitchDegrees", getPitchDegrees());
    Logger.recordOutput("Gyro/PigeonRollDegrees", getRollDegrees());
  }

  // Get current heading from Pigeon2.
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
  }

  public double getPitchDegrees() {
    return pigeon.getPitch().getValueAsDouble();
  }

  public double getRollDegrees() {
    return pigeon.getRoll().getValueAsDouble();
  }

  // Reset heading to zero. Logs to AKit and console.
  public void resetHeading() {
    pigeon.reset();
    
    if (DriverStation.isAutonomous()) {
      Logger.recordOutput("Gyro/HeadingReset/Auto", true);
      SmartLogger.logConsole("Pigeon reset in AUTONOMOUS mode", "Gyro");
    } else {
      Logger.recordOutput("Gyro/HeadingReset/Teleop", true);
      SmartLogger.logConsole("Pigeon reset in TELEOP mode", "Gyro");
    }
    
    Logger.recordOutput("Gyro/HeadingReset", true);
  }

  // Set heading to a specific angle in degrees.
  public void setHeading(double angleDegrees) {
    pigeon.setYaw(angleDegrees);
    Logger.recordOutput("Gyro/HeadingSet", angleDegrees);
  }
}