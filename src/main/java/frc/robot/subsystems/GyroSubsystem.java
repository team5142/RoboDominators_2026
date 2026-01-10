package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Gyro Subsystem - Pigeon2 IMU for robot heading
 * 
 * Provides:
 * - Yaw (rotation around vertical axis)
 * - Always-available heading (doesn't lose tracking like QuestNav)
 * - High update rate (200Hz+)
 * 
 * Note: QuestNav SLAM is now in QuestNavSubsystem (separate from gyro)
 */
public class GyroSubsystem extends SubsystemBase {
  private final Pigeon2 pigeon;

  public GyroSubsystem() {
    System.out.println("=== GyroSubsystem Initialization ===");
    
    pigeon = new Pigeon2(PIGEON_CAN_ID, CAN_BUS_NAME);
    pigeon.reset();
    
    System.out.println("Pigeon2 initialized on CAN ID: " + PIGEON_CAN_ID);
    System.out.println("=====================================\n");
  }

  @Override
  public void periodic() {
    Rotation2d rotation = getRotation();
    
    Logger.recordOutput("Gyro/Rotation", rotation);
    Logger.recordOutput("Gyro/RotationDegrees", rotation.getDegrees());
    Logger.recordOutput("Gyro/RotationRadians", rotation.getRadians());
    Logger.recordOutput("Gyro/PigeonYawDegrees", pigeon.getYaw().getValueAsDouble());
  }

  /**
   * Get current heading from Pigeon2
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
  }

  /**
   * Reset heading to zero
   */
  public void resetHeading() {
    pigeon.reset();
    
    if (DriverStation.isAutonomous()) {
      Logger.recordOutput("Gyro/HeadingReset/Auto", true);
      System.out.println("Pigeon reset in AUTONOMOUS mode");
    } else {
      Logger.recordOutput("Gyro/HeadingReset/Teleop", true);
      System.out.println("Pigeon reset in TELEOP mode");
    }
    
    Logger.recordOutput("Gyro/HeadingReset", true);
  }

  /**
   * Set heading to a specific angle
   */
  public void setHeading(double angleDegrees) {
    pigeon.setYaw(angleDegrees);
    Logger.recordOutput("Gyro/HeadingSet", angleDegrees);
  }
}