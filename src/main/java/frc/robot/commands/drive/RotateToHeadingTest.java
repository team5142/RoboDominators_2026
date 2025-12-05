package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Simple rotation-only test command for tuning rotation PID.
 * 
 * Takes a RELATIVE heading change (e.g., +90 or -90 degrees) and rotates to it.
 * Uses gyro as absolute truth and a simple PID controller.
 * 
 * TO TUNE:
 * - Adjust kP: higher = faster spin-up, but can overshoot
 * - Adjust kD: higher = more damping, reduces overshoot/oscillation
 * - Once tuned, copy final values to Constants.Auto.ROTATION_KP/KD for PathPlanner
 * 
 * TO REMOVE LATER:
 * - Delete this file
 * - Remove LB/RB bindings in RobotContainer
 * - Restore SmartDrive bindings for bumpers if needed
 */
public class RotateToHeadingTest extends Command {
  private final DriveSubsystem drive;
  private final GyroSubsystem gyro;
  private final PIDController thetaController;
  
  private final double relativeHeadingChangeDeg; // +90 or -90, etc.
  private double targetHeadingDeg; // computed in initialize()
  
  // TUNING PARAMETERS - adjust these to tune rotation behavior
  private static final double KP = 6.00;  // INCREASED from 0.02 → should give ~4.5°/s per degree of error
  private static final double KD = 0.15; // INCREASED proportionally
  private static final double MAX_OMEGA_RAD_PER_SEC = Math.PI; // ~180°/s max
  private static final double TOLERANCE_DEG = 2.0; // Finish when within 2°
  
  private int executeCounter = 0; // NEW: track execute calls
  
  /**
   * @param drive DriveSubsystem
   * @param gyro GyroSubsystem (absolute truth for heading)
   * @param relativeHeadingChangeDeg Relative rotation (e.g., +90 for CCW, -90 for CW)
   */
  public RotateToHeadingTest(DriveSubsystem drive, GyroSubsystem gyro, double relativeHeadingChangeDeg) {
    this.drive = drive;
    this.gyro = gyro;
    this.relativeHeadingChangeDeg = relativeHeadingChangeDeg;
    
    thetaController = new PIDController(KP, 0.0, KD);
    thetaController.enableContinuousInput(-180.0, 180.0);
    thetaController.setTolerance(TOLERANCE_DEG);
    
    addRequirements(drive);
    
    // DEBUG: Print when command is constructed
    System.out.println("[RotateTest] Command CONSTRUCTED for " + relativeHeadingChangeDeg + "° rotation");
  }
  
  @Override
  public void initialize() {
    executeCounter = 0; // Reset counter
    System.out.println("========================================");
    System.out.println("[RotateTest] INITIALIZE() CALLED");
    System.out.println("========================================");
    
    double currentHeadingDeg = gyro.getRotation().getDegrees();
    targetHeadingDeg = currentHeadingDeg + relativeHeadingChangeDeg;
    
    // Normalize to -180 to +180
    while (targetHeadingDeg > 180.0) targetHeadingDeg -= 360.0;
    while (targetHeadingDeg < -180.0) targetHeadingDeg += 360.0;
    
    System.out.println(String.format(
        "[RotateTest] Start: %.1f° → Target: %.1f° (change: %.1f°)",
        currentHeadingDeg, targetHeadingDeg, relativeHeadingChangeDeg));
    
    Logger.recordOutput("RotateTest/StartHeadingDeg", currentHeadingDeg);
    Logger.recordOutput("RotateTest/TargetHeadingDeg", targetHeadingDeg);
    Logger.recordOutput("RotateTest/RelativeChangeDeg", relativeHeadingChangeDeg);
  }
  
  @Override
  public void execute() {
    executeCounter++;
    
    // AGGRESSIVE DEBUG: Print EVERY execute call for first 10 cycles
    if (executeCounter <= 10) {
      System.out.println("[RotateTest] EXECUTE() cycle #" + executeCounter);
    }
    
    double currentHeadingDeg = gyro.getRotation().getDegrees();
    double errorDeg = targetHeadingDeg - currentHeadingDeg;
    
    // PID outputs in deg/s
    double omegaDegPerSec = thetaController.calculate(currentHeadingDeg, targetHeadingDeg);
    double omegaRadPerSec = Math.toRadians(omegaDegPerSec);
    
    // Clamp to max speed
    omegaRadPerSec = Math.max(-MAX_OMEGA_RAD_PER_SEC,
                              Math.min(MAX_OMEGA_RAD_PER_SEC, omegaRadPerSec));
    
    // Drive with zero translation, only rotation
    drive.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, omegaRadPerSec));
    
    // Print every cycle for first 10, then every ~5 cycles
    if (executeCounter <= 10 || Logger.getTimestamp() % 0.2 < 0.02) {
      System.out.println(String.format(
          "[RotateTest] Current: %.1f° | Target: %.1f° | Error: %.1f° | Omega: %.2f rad/s",
          currentHeadingDeg, targetHeadingDeg, errorDeg, omegaRadPerSec));
    }
    
    // Log for tuning analysis
    Logger.recordOutput("RotateTest/CurrentHeadingDeg", currentHeadingDeg);
    Logger.recordOutput("RotateTest/ErrorDeg", errorDeg);
    Logger.recordOutput("RotateTest/OmegaDegPerSec", omegaDegPerSec);
    Logger.recordOutput("RotateTest/OmegaRadPerSec", omegaRadPerSec);
    Logger.recordOutput("RotateTest/AtSetpoint", thetaController.atSetpoint());
  }
  
  @Override
  public void end(boolean interrupted) {
    // Stop rotation
    drive.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    
    double finalHeadingDeg = gyro.getRotation().getDegrees();
    double finalErrorDeg = targetHeadingDeg - finalHeadingDeg;
    
    System.out.println("========================================");
    System.out.println(String.format(
        "[RotateTest] END() | %s | Final: %.1f° | Error: %.1f°",
        interrupted ? "INTERRUPTED" : "COMPLETE",
        finalHeadingDeg,
        finalErrorDeg));
    System.out.println("========================================");
    
    Logger.recordOutput("RotateTest/FinalHeadingDeg", finalHeadingDeg);
    Logger.recordOutput("RotateTest/FinalErrorDeg", finalErrorDeg);
    Logger.recordOutput("RotateTest/Interrupted", interrupted);
  }
  
  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint();
  }
}
