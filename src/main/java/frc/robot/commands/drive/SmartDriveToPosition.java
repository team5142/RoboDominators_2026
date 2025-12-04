package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;
import frc.robot.subsystems.QuestNavSubsystem; // ADD THIS
import org.littletonrobotics.junction.Logger;

/**
 * Smart navigation to a target pose with safety checks and logging.
 */
public class SmartDriveToPosition {
  
  // Vision acquisition distance, speed multipliers, field bounds, etc.
  private static final double IDEAL_VISION_DISTANCE_METERS = .75; // ~3.3 feet
  private static final double FAST_SPEED_MULT = 0.40;      // 40% speed for approach
  private static final double PRECISION_SPEED_MULT = 0.4;  // was 0.2 → 0.3 → 0.4
  private static final double FIELD_LENGTH_METERS = Units.feetToMeters(54.0);
  private static final double FIELD_WIDTH_METERS = Units.feetToMeters(27.0);
  private static final double FIELD_MARGIN_METERS = 0.3; // Stay 1ft from edges
  private static final double MAX_PATH_DISTANCE_METERS = 10.0; // Reject paths >33ft

  private static final boolean TUNING_MODE_IGNORE_FIELD_LIMITS = true;
  // ==========================================================================
  
  public static Command create(
      Pose2d target,
      PoseEstimatorSubsystem poseEstimator,
      TagVisionSubsystem tagVision,
      RobotState robotState,
      XboxController driverController,
      DriveSubsystem driveSubsystem,
      QuestNavSubsystem questNavSubsystem) {
    
    Pose2d start = poseEstimator.getEstimatedPose();
    double distanceToTarget = start.getTranslation().getDistance(target.getTranslation());
    
    // ===== PRE-FLIGHT CHECKS =====
    
    if (distanceToTarget > MAX_PATH_DISTANCE_METERS) {
      DriverStation.reportError(
          String.format("Target too far: %.1fm (max: %.1fm)", 
              distanceToTarget, MAX_PATH_DISTANCE_METERS), 
          false);
      
      return new SequentialCommandGroup(
          Commands.print("ERROR: Target unreachable - too far"),
          Commands.runOnce(() -> {
            SmartDashboard.putString("SmartDrive/Status", "ERROR: Too far");
            robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
          })
      );
    }
    
    // Field bounds check (bypass in tuning mode)
    if (!TUNING_MODE_IGNORE_FIELD_LIMITS && !isWithinField(target.getTranslation())) {
      DriverStation.reportError("Target outside field boundaries", false);
      
      return new SequentialCommandGroup(
          Commands.print("ERROR: Target outside field"),
          Commands.runOnce(() -> {
            SmartDashboard.putString("SmartDrive/Status", "ERROR: Out of bounds");
            robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
          })
      );
    } else if (TUNING_MODE_IGNORE_FIELD_LIMITS && !isWithinField(target.getTranslation())) {
      DriverStation.reportWarning("SmartDrive TUNING: target outside field limits - CHECK BEFORE COMPETITION", false);
      SmartDashboard.putString("SmartDrive/Status", "TUNING: Out-of-field target allowed");
      Logger.recordOutput("SmartDrive/Tuning/OutOfFieldTarget", target);
    }
    
    // ===== SINGLE-PHASE DIRECT PATH TO TARGET =====
    System.out.println("=== SMART DRIVE: DIRECT PATH (Single Phase) ===");
    System.out.println("Total distance: " + String.format("%.2fm", distanceToTarget));
    System.out.println("Target: " + target);
    
    return new SequentialCommandGroup(
        Commands.runOnce(() -> {
          robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
          SmartDashboard.putString("SmartDrive/Status", "Direct path to target");
          SmartDashboard.putNumber("SmartDrive/Progress", 0.0);
          Logger.recordOutput("SmartDrive/Mode", "SinglePhaseDirect");
        }),
        
        // Single PathPlanner pathfind all the way to the target
        AutoBuilder.pathfindToPose(target, createPrecisionConstraints())
            .withTimeout(8.0) // safety timeout; adjust if needed
    ).finallyDo((interrupted) -> {
      robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
      SmartDashboard.putString("SmartDrive/Status", interrupted ? "Interrupted" : "Complete");
      Logger.recordOutput("SmartDrive/Interrupted", interrupted);
      Logger.recordOutput("SmartDrive/Complete", !interrupted);
    });
  }

  // ===== FIELD VALIDATION HELPERS =====
  
  private static boolean isWithinField(Translation2d point) {
    return point.getX() > FIELD_MARGIN_METERS &&
           point.getX() < FIELD_LENGTH_METERS - FIELD_MARGIN_METERS &&
           point.getY() > FIELD_MARGIN_METERS &&
           point.getY() < FIELD_WIDTH_METERS - FIELD_MARGIN_METERS;
  }
  
  private static PathConstraints createPrecisionConstraints() {
    return new PathConstraints(
        Constants.Swerve.MAX_TRANSLATION_SPEED_MPS * PRECISION_SPEED_MULT, // 40% speed
        3.5, // was 2.0 → 3.0 → 3.5
        Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SEC * 0.5,
        Math.PI / 2 // Max angular accel
    );
  }
}
