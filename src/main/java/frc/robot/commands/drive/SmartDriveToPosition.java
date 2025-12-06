package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
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

/**
 * Smart navigation to a target pose with optional precision path.
 * 
 * TWO-PHASE NAVIGATION:
 * - Phase 1: Pathfind to staging pose (dynamic obstacle avoidance)
 * - Phase 2: Follow pre-recorded GUI path for precision (if provided)
 * 
 * If precisionPathFile is null, only Phase 1 runs (direct pathfind to target).
 */
public class SmartDriveToPosition {
  
  private static final double PRECISION_SPEED_MULT = 0.4;
  private static final double FIELD_LENGTH_METERS = Units.feetToMeters(54.0);
  private static final double FIELD_WIDTH_METERS = Units.feetToMeters(27.0);
  private static final double FIELD_MARGIN_METERS = 0.3;
  private static final double MAX_PATH_DISTANCE_METERS = 10.0;

  /**
   * Create a smart navigation command
   * 
   * @param stagingPose Target pose for Phase 1 pathfinding (also used as final target if no precision path)
   * @param precisionPathFile Optional PathPlanner .path file for Phase 2 precision approach (null = skip Phase 2)
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

    // ===== TWO-PHASE PRECISION PATH (if precision file provided) =====
    if (precisionPathFile != null && !precisionPathFile.isEmpty()) {
      Logger.recordOutput("SmartDrive/Precision/HasPrecisionPath", true);
      Logger.recordOutput("SmartDrive/Precision/PathFile", precisionPathFile);
      
      // Phase 1: Pathfind to staging pose
      Command toStaging = AutoBuilder.pathfindToPose(
          stagingPose,
          createPrecisionConstraints())
          .withTimeout(8.0);

      // Phase 2: Follow the pre-recorded precision path
      PathPlannerPath precisionPath;
      try {
        precisionPath = PathPlannerPath.fromPathFile(precisionPathFile);
      } catch (Exception e) {
        DriverStation.reportError("Failed to load precision path: " + precisionPathFile, e.getStackTrace());
        return Commands.none();
      }
      
      Command followPrecision = AutoBuilder.followPath(precisionPath)
          .withTimeout(5.0);

      return new SequentialCommandGroup(
          Commands.runOnce(() -> {
            robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
            SmartDashboard.putString("SmartDrive/Status", "Pathfind + PrecisionPath");
            SmartDashboard.putNumber("SmartDrive/Progress", 0.0);
            Logger.recordOutput("SmartDrive/Mode", "TwoPhase: " + precisionPathFile);
          }),
          toStaging,
          followPrecision
      ).finallyDo((interrupted) -> {
        robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
        SmartDashboard.putString("SmartDrive/Status", interrupted ? "Interrupted" : "Complete");
        Logger.recordOutput("SmartDrive/Interrupted", interrupted);
        Logger.recordOutput("SmartDrive/Complete", !interrupted);
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
}
