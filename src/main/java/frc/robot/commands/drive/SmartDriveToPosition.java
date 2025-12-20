package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.QuestNavSubsystem;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import frc.robot.commands.drive.AutoPilotToTargetCommand; // ADD THIS

/**
 * Smart navigation to a target pose with optional precision path.
 * 
 * THREE-PHASE NAVIGATION:
 * - Phase 1: Pathfind to staging pose (dynamic obstacle avoidance)
 * - Phase 2: Follow pre-recorded GUI path for precision (if provided)
 * - Phase 3: QuestNav-based final correction (< 3cm accuracy)
 * 
 * If precisionPathFile is null, only Phase 1 runs (direct pathfind to target).
 */
public class SmartDriveToPosition {
  
  private static final double PRECISION_SPEED_MULT = 0.6; // CHANGED: 60% speed (was 0.4 - 40%)
  private static final double FIELD_LENGTH_METERS = Units.feetToMeters(54.0);
  private static final double FIELD_WIDTH_METERS = Units.feetToMeters(27.0);
  private static final double FIELD_MARGIN_METERS = 0.3;
  private static final double MAX_PATH_DISTANCE_METERS = 10.0;
  private static final double QUESTNAV_SETTLE_TIME_SECONDS = 0.5; // CHANGED: Max wait time (was 0.33s)
  private static final int QUESTNAV_MAX_WAIT_LOOPS = 25; // NEW: 25 loops * 20ms = 500ms max
  private static final double AUTOPILOT_MAX_VELOCITY = 2.0; // Max velocity in meters per second
  private static final double AUTOPILOT_MAX_ACCELERATION = 1.0; // Max acceleration in meters per second squared
  private static final double AUTOPILOT_MAX_JERK = 3.0; // m/s³ (NEW)
  private static final boolean ENABLE_PHASE_3_CORRECTION = false; // Set to true to re-enable

  /**
   * Create a smart navigation command
   * 
   * @param stagingPose Target pose for Phase 1 pathfinding (intermediate waypoint)
   * @param finalTargetPose Target pose for Phase 2 AutoPilot (final precision target)
   * @param precisionPathFile Optional PathPlanner .path file (currently unused, will be removed later)
   * @param poseEstimator Pose estimator subsystem
   * @param tagVision Tag vision subsystem
   * @param robotState Robot state
   * @param driverController Driver controller
   * @param driveSubsystem Drive subsystem
   * @param questNavSubsystem QuestNav subsystem
   * @return Command that navigates to the target
   */
  public static Command create(
      Pose2d stagingPose,
      Pose2d finalTargetPose, // NEW: Separate target for AutoPilot
      String precisionPathFile,
      PoseEstimatorSubsystem poseEstimator,
      TagVisionSubsystem tagVision,
      RobotState robotState,
      XboxController driverController,
      DriveSubsystem driveSubsystem,
      QuestNavSubsystem questNavSubsystem) {
    
    Pose2d start = poseEstimator.getEstimatedPose();
    double distanceToTarget = start.getTranslation().getDistance(stagingPose.getTranslation());
    
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
    
    if (!isWithinField(stagingPose.getTranslation())) {
      DriverStation.reportError("Target outside field boundaries", false);
      
      return new SequentialCommandGroup(
          Commands.print("ERROR: Target outside field"),
          Commands.runOnce(() -> {
            SmartDashboard.putString("SmartDrive/Status", "ERROR: Out of bounds");
            robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
          })
      );
    }
    
    Logger.recordOutput("SmartDrive/StagingPose", stagingPose);
    Logger.recordOutput("SmartDrive/DistanceToTarget", distanceToTarget);

    // ===== THREE-PHASE PRECISION PATH (if precision file provided) =====
    if (precisionPathFile != null && !precisionPathFile.isEmpty()) {
      Logger.recordOutput("SmartDrive/Precision/HasPrecisionPath", true);
      Logger.recordOutput("SmartDrive/Precision/PathFile", precisionPathFile);
      Logger.recordOutput("SmartDrive/FinalTargetPose", finalTargetPose); // NEW
      
      // Phase 1: Pathfind to staging pose
      Command toStaging = AutoBuilder.pathfindToPose(
          stagingPose,
          createPrecisionConstraints())
          .withTimeout(8.0);

      // Phase 2: Use AutoPilot COMMAND instead of DriveSubsystem method
      Command followPrecision = new AutoPilotToTargetCommand(
          finalTargetPose, 
          driveSubsystem, 
          poseEstimator,
          AUTOPILOT_MAX_VELOCITY,
          AUTOPILOT_MAX_ACCELERATION,
          AUTOPILOT_MAX_JERK);

      // Phase 3: QuestNav-based final correction (OPTIONAL)
      Command finalCorrection = ENABLE_PHASE_3_CORRECTION 
          ? PostPathCorrection.attemptCorrection(poseEstimator, questNavSubsystem)
          : Commands.none(); // Skip Phase 3 if disabled

      return new SequentialCommandGroup(
          Commands.runOnce(() -> {
            robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
            SmartDashboard.putString("SmartDrive/Status", "Phase 1: Pathfinding");
            SmartDashboard.putNumber("SmartDrive/Progress", 0.0);
            Logger.recordOutput("SmartDrive/Mode", "ThreePhase: AutoPilot");
          }),
          toStaging,
          
          // IMPROVED: FORCE-ACCEPT QUESTNAV POSE (no waiting!)
          Commands.runOnce(() -> {
            System.out.println("========== FORCE QUESTNAV POSE UPDATE ==========");
            Logger.recordOutput("SmartDrive/Phase", "ForceQuestNavUpdate");
            
            // Stop motion completely
            driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
            
            // Force-accept QuestNav pose with very high trust
            boolean success = poseEstimator.forceAcceptQuestNavPose();
            
            if (success) {
              Pose2d updatedPose = poseEstimator.getEstimatedPose();
              System.out.println("Pose force-updated to: " + formatPose(updatedPose));
              Logger.recordOutput("SmartDrive/ForcedPose", updatedPose);
            } else {
              System.err.println("WARNING: Failed to force-accept QuestNav pose!");
              System.err.println("Proceeding with current estimate (may be inaccurate)");
              Logger.recordOutput("SmartDrive/ForceFailed", true);
            }
            
            System.out.println("===============================================");
          }),
          
          // Small delay to ensure pose is fully settled (100ms)
          Commands.waitSeconds(0.1),
          
          Commands.runOnce(() -> {
            robotState.setNavigationPhase(RobotState.NavigationPhase.PRECISION_PATH);
            SmartDashboard.putString("SmartDrive/Status", "Phase 2: AutoPilot precision path");
            SmartDashboard.putNumber("SmartDrive/Progress", 0.5);
            
            Pose2d startingPose = poseEstimator.getEstimatedPose();
            System.out.println("[SmartDrive] Starting AutoPilot from: " + formatPose(startingPose));
            System.out.println("[SmartDrive] AutoPilot targeting: " + formatPose(finalTargetPose));
          }),
          followPrecision,
          
          // Phase 3: Conditional final correction
          Commands.runOnce(() -> {
            if (ENABLE_PHASE_3_CORRECTION) {
              robotState.setNavigationPhase(RobotState.NavigationPhase.POST_CORRECTION);
              SmartDashboard.putString("SmartDrive/Status", "Phase 3: QuestNav correction");
              SmartDashboard.putNumber("SmartDrive/Progress", 0.9);
            } else {
              System.out.println("[SmartDrive] Phase 3 DISABLED - AutoPilot complete");
              robotState.setNavigationPhase(RobotState.NavigationPhase.LOCKED);
              SmartDashboard.putString("SmartDrive/Status", "Complete - AutoPilot only");
              SmartDashboard.putNumber("SmartDrive/Progress", 1.0);
            }
          }),
          finalCorrection, // No-op if disabled
          
          Commands.runOnce(() -> {
            robotState.setNavigationPhase(RobotState.NavigationPhase.LOCKED);
            SmartDashboard.putString("SmartDrive/Status", "Complete - Locked");
            SmartDashboard.putNumber("SmartDrive/Progress", 1.0);
            driveSubsystem.lockWheels(); // X-pattern brake
          })
      ).finallyDo((interrupted) -> {
        robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
        SmartDashboard.putString("SmartDrive/Status", interrupted ? "Interrupted" : "Complete");
        Logger.recordOutput("SmartDrive/Interrupted", interrupted);
        Logger.recordOutput("SmartDrive/Complete", !interrupted);
        Logger.recordOutput("SmartDrive/Phase3Enabled", ENABLE_PHASE_3_CORRECTION);
      });
    }

    // ===== SINGLE-PHASE DIRECT PATH (no precision file) =====
    Logger.recordOutput("SmartDrive/Precision/HasPrecisionPath", false);
    
    return new SequentialCommandGroup(
        Commands.runOnce(() -> {
          robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
          SmartDashboard.putString("SmartDrive/Status", "Direct pathfind to target");
          SmartDashboard.putNumber("SmartDrive/Progress", 0.0);
          Logger.recordOutput("SmartDrive/Mode", "SinglePhase");
        }),
        AutoBuilder.pathfindToPose(stagingPose, createPrecisionConstraints())
            .withTimeout(8.0)
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
        Constants.Swerve.MAX_TRANSLATION_SPEED_MPS * PRECISION_SPEED_MULT,
        3.5,
        Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SEC * 0.8,
        Math.PI
    );
  }
  
  // NEW: Format pose for logging
  private static String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }
}
