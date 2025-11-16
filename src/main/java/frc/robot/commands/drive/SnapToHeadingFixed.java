package frc.robot.commands.drive;

import static frc.robot.Constants.Swerve.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class SnapToHeadingFixed extends Command {
  private final DriveSubsystem driveSubsystem;
  private final RobotState robotState;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier targetHeadingDegreesSupplier;

  private final ProfiledPIDController headingController;

  public SnapToHeadingFixed(
      DriveSubsystem driveSubsystem,
      RobotState robotState,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier targetHeadingDegreesSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.robotState = robotState;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.targetHeadingDegreesSupplier = targetHeadingDegreesSupplier;

    // Create PID controller for heading
    headingController = new ProfiledPIDController(
        5.0, 0.0, 0.1,
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RAD_PER_SEC,
            MAX_ANGULAR_SPEED_RAD_PER_SEC * 2.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    headingController.reset(driveSubsystem.getGyroRotation().getRadians()); // ✅ Use gyro, not pose
  }

  @Override
  public void execute() {
    // Apply deadband to translation
    double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), JOYSTICK_DEADBAND);
    double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), JOYSTICK_DEADBAND);

    // Square inputs
    x = Math.copySign(x * x, x);
    y = Math.copySign(y * y, y);

    // Convert to m/s
    double xMetersPerSec = x * MAX_TRANSLATION_SPEED_MPS;
    double yMetersPerSec = y * MAX_TRANSLATION_SPEED_MPS;

    // Calculate heading correction - USE GYRO, NOT POSE
    double targetHeadingRad = Math.toRadians(targetHeadingDegreesSupplier.getAsDouble());
    double currentHeadingRad = driveSubsystem.getGyroRotation().getRadians(); // ✅ Changed from robotState
    double omegaRadPerSec = headingController.calculate(currentHeadingRad, targetHeadingRad);

    // Drive with normal speed scale
    driveSubsystem.drive(
        xMetersPerSec,
        yMetersPerSec,
        omegaRadPerSec,
        true);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until released
  }
}
