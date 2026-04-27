package frc.robot.commands.drive;

import static frc.robot.Constants.Swerve.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

// Auto-rotation to cardinal directions (0, 90, 180, 270 degrees).
// Driver controls translation (left stick), command controls rotation.
// Uses a ProfiledPIDController for smooth deceleration instead of a sudden stop.

/*
 * TASK 31 - Read Through This File
 * -----------------------------------------------------------------------
 * This is your first named Command class. Before changing anything, read
 * through it and understand:
 *   - What is the difference between initialize() and execute()?
 *     When does each one run?
 *   - What does ProfiledPIDController do that a plain PID does not?
 *   - Why is enableContinuousInput(-Math.PI, Math.PI) needed?
 *     What would happen at 180 degrees without it?
 *   - Why does isFinished() return false?
 *
 * Nothing to write - understand it. When done: move to Task 32.
 * -----------------------------------------------------------------------
 */

/*
 * TASK 32 - Tune the PID Gains
 * -----------------------------------------------------------------------
 * The three gains control how aggressively the controller rotates:
 *   kP = proportional - how hard it pushes toward the target
 *   kI = integral     - builds up over time if it never quite reaches target
 *   kD = derivative   - dampens overshoot
 *
 * Try each of these and note what you feel:
 *   kP = 2.0  (sluggish, undershoots)
 *   kP = 10.0 (aggressive, may oscillate)
 *   kP = 5.0  (current value - the baseline)
 *
 * [ROBOT OPTIONAL] Change kP, deploy, hold a D-pad direction.
 * Watch how fast and smoothly the robot snaps to heading.
 *
 * When done: move to Task 33 in RobotContainer.java.
 * -----------------------------------------------------------------------
 */

public class SnapToHeadingFixed extends Command {
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier targetHeadingDegreesSupplier;

  private final ProfiledPIDController headingController;

  public SnapToHeadingFixed(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier targetHeadingDegreesSupplier) {
    this.driveSubsystem = driveSubsystem;
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
