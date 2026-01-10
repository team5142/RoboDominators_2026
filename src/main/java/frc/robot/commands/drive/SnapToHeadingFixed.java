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

// Auto-rotation to cardinal directions (0, 90, 180, 270 degrees)
// Driver controls translation (left stick), command controls rotation to snap heading
// Uses profiled PID for smooth deceleration instead of sudden stop
// Bound to D-pad: Up=0°, Right=-90°, Down=180°, Left=90°
public class SnapToHeadingFixed extends Command {
  private final DriveSubsystem driveSubsystem;
  private final RobotState robotState;
  private final DoubleSupplier xSupplier; // Forward/back from joystick
  private final DoubleSupplier ySupplier; // Strafe from joystick
  private final DoubleSupplier targetHeadingDegreesSupplier; // Target angle

  private final ProfiledPIDController headingController; // PID with acceleration limits

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

    // Create PID controller with motion profiling (limits acceleration)
    headingController = new ProfiledPIDController(
        5.0, // kP - proportional gain (increase for faster response)
        0.0, // kI - integral (not needed for heading hold)
        0.1, // kD - derivative (damping)
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RAD_PER_SEC, // Max rotation speed
            MAX_ANGULAR_SPEED_RAD_PER_SEC * 2.0)); // Max rotation acceleration
    headingController.enableContinuousInput(-Math.PI, Math.PI); // Handle wraparound at 180°/-180°

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    headingController.reset(driveSubsystem.getGyroRotation().getRadians()); // Start from current heading
  }

  @Override
  public void execute() {
    // Apply deadband and square translation inputs (same as DriveWithJoysticks)
    double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), JOYSTICK_DEADBAND);
    double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), JOYSTICK_DEADBAND);

    x = Math.copySign(x * x, x); // Square for finer control
    y = Math.copySign(y * y, y);

    // Convert to m/s
    double xMetersPerSec = x * MAX_TRANSLATION_SPEED_MPS;
    double yMetersPerSec = y * MAX_TRANSLATION_SPEED_MPS;

    // Calculate rotation correction using PID
    double targetHeadingRad = Math.toRadians(targetHeadingDegreesSupplier.getAsDouble());
    double currentHeadingRad = driveSubsystem.getGyroRotation().getRadians();
    double omegaRadPerSec = headingController.calculate(currentHeadingRad, targetHeadingRad);

    // Drive with manual translation + automatic rotation
    driveSubsystem.drive(
        xMetersPerSec,
        yMetersPerSec,
        omegaRadPerSec, // PID-controlled rotation
        true); // Field-relative
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0, true); // Stop all motion
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until button released
  }
}
