package frc.robot.commands.drive;

import static frc.robot.Constants.Swerve.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// Joystick teleoperated driving - runs as default command on DriveSubsystem
// Applies deadbands, squaring, direction smoothing to prevent wheel twitching
public class DriveWithJoysticks extends Command {
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final BooleanSupplier fieldRelativeSupplier;
  private final BooleanSupplier precisionModeSupplier;

  // Direction smoothing - prevents wheels from micro-steering on tiny stick movements
  private double lastTranslationX = 0.0;
  private double lastTranslationY = 0.0;
  private static final double TRANSLATION_DEADBAND = 0.05;
  private static final double SLOW_SPEED_THRESHOLD = 0.4;

  public DriveWithJoysticks(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fieldRelativeSupplier,
      BooleanSupplier precisionModeSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.fieldRelativeSupplier = fieldRelativeSupplier;
    this.precisionModeSupplier = precisionModeSupplier;

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    // Apply deadband to ignore stick drift
    double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), JOYSTICK_DEADBAND);
    double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), JOYSTICK_DEADBAND);
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), JOYSTICK_DEADBAND);

    // x^1.5 curve: more responsive than squaring at low speeds, less twitchy than linear
    x = Math.copySign(Math.pow(Math.abs(x), 1.5), x);
    y = Math.copySign(Math.pow(Math.abs(y), 1.5), y);
    omega = Math.copySign(Math.pow(Math.abs(omega), 1.5), omega);

    // Direction smoothing - prevents steering micro-adjustments during slow movement
    double translationMagnitude = Math.hypot(x, y);
    
    if (translationMagnitude > 0.0) {
      // Check if direction change is significant enough to warrant re-steering wheels
      double deltaX = Math.abs(x - lastTranslationX);
      double deltaY = Math.abs(y - lastTranslationY);
      double directionChange = Math.hypot(deltaX, deltaY);
      
      // If direction barely changed AND we're moving slowly, keep old direction
      if (directionChange < TRANSLATION_DEADBAND && translationMagnitude < SLOW_SPEED_THRESHOLD) {
        x = lastTranslationX; // Use previous direction to prevent steering flutter
        y = lastTranslationY;
      } else {
        lastTranslationX = x; // Significant direction change - allow steering adjustment
        lastTranslationY = y;
      }
    } else {
      lastTranslationX = 0.0; // Stopped - reset tracking
      lastTranslationY = 0.0;
    }

    // Desaturate combined translation + rotation to prevent module over-speed
    double combinedMagnitude = Math.hypot(translationMagnitude, omega);
    
    if (combinedMagnitude > 1.0) {
      double scale = 1.0 / combinedMagnitude; // Scale down to keep magnitude at 1.0
      x *= scale;
      y *= scale;
      omega *= scale;
    }

    // Convert normalized inputs to m/s and rad/s
    double xMetersPerSec = x * MAX_TRANSLATION_SPEED_MPS;
    double yMetersPerSec = y * MAX_TRANSLATION_SPEED_MPS;
    double omegaRadPerSec = omega * MAX_ANGULAR_SPEED_RAD_PER_SEC;

    // Apply speed scaling based on precision mode
    boolean precision = precisionModeSupplier.getAsBoolean();
    double translationScale = precision ? PRECISION_SPEED_SCALE     : NORMAL_SPEED_SCALE;
    double rotationScale    = precision ? PRECISION_ROTATION_SCALE  : NORMAL_ROTATION_SCALE;

    // Send drive command
    driveSubsystem.drive(
        xMetersPerSec     * translationScale,
        yMetersPerSec     * translationScale,
        omegaRadPerSec    * rotationScale,
        fieldRelativeSupplier.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0, true); // Stop robot when command ends
  }
}
