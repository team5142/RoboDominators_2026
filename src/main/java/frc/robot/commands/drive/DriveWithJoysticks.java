package frc.robot.commands.drive;

import static frc.robot.Constants.Swerve.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveWithJoysticks extends Command {
  private final DriveSubsystem driveSubsystem;
  private final RobotState robotState;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final BooleanSupplier fieldRelativeSupplier;
  private final BooleanSupplier precisionModeSupplier;

  // NEW: Track previous translation direction to prevent micro-steering
  private double lastTranslationX = 0.0;
  private double lastTranslationY = 0.0;
  private static final double TRANSLATION_DEADBAND = 0.05; // INCREASED from 0.02 to 0.05 (5%)
  private static final double SLOW_SPEED_THRESHOLD = 0.4; // INCREASED from 0.3 to 0.4 (40%)

  public DriveWithJoysticks(
      DriveSubsystem driveSubsystem,
      RobotState robotState,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fieldRelativeSupplier,
      BooleanSupplier precisionModeSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.robotState = robotState;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.fieldRelativeSupplier = fieldRelativeSupplier;
    this.precisionModeSupplier = precisionModeSupplier;

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    // Apply deadband
    double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), JOYSTICK_DEADBAND);
    double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), JOYSTICK_DEADBAND);
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), JOYSTICK_DEADBAND);

    // Square inputs for finer control while preserving sign
    x = Math.copySign(x * x, x);
    y = Math.copySign(y * y, y);
    omega = Math.copySign(omega * omega, omega);

    // NEW: Prevent steering micro-adjustments for tiny stick movements
    // Only update translation direction if stick moved significantly
    double translationMagnitude = Math.hypot(x, y);
    
    if (translationMagnitude > 0.0) {
      // Check if direction change is significant enough to warrant re-steering wheels
      double deltaX = Math.abs(x - lastTranslationX);
      double deltaY = Math.abs(y - lastTranslationY);
      double directionChange = Math.hypot(deltaX, deltaY);
      
      // If direction barely changed AND we're moving slowly, keep old direction
      if (directionChange < TRANSLATION_DEADBAND && translationMagnitude < SLOW_SPEED_THRESHOLD) {
        // Use previous direction to prevent steering flutter
        x = lastTranslationX;
        y = lastTranslationY;
      } else {
        // Significant direction change - allow steering adjustment
        lastTranslationX = x;
        lastTranslationY = y;
      }
    } else {
      // Stopped - reset tracking
      lastTranslationX = 0.0;
      lastTranslationY = 0.0;
    }

    // Desaturate combined translation + rotation to prevent jerky movement
    double combinedMagnitude = Math.hypot(translationMagnitude, omega);
    
    if (combinedMagnitude > 1.0) {
      // Scale down to keep combined magnitude at 1.0
      double scale = 1.0 / combinedMagnitude;
      x *= scale;
      y *= scale;
      omega *= scale;
    }

    // Convert to m/s and rad/s
    double xMetersPerSec = x * MAX_TRANSLATION_SPEED_MPS;
    double yMetersPerSec = y * MAX_TRANSLATION_SPEED_MPS;
    double omegaRadPerSec = omega * MAX_ANGULAR_SPEED_RAD_PER_SEC;

    // Determine speed scale
    double speedScale = precisionModeSupplier.getAsBoolean() 
        ? PRECISION_SPEED_SCALE 
        : NORMAL_SPEED_SCALE;

    // Drive
    driveSubsystem.drive(
        xMetersPerSec * speedScale,
        yMetersPerSec * speedScale,
        omegaRadPerSec * speedScale,
        fieldRelativeSupplier.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0, true);
  }
}
