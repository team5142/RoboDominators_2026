package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.util.SmartLogger;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.drive.AutoPilotToTargetCommand;

// Factory for hybrid PathPlanner + AutoPilot navigation commands
// Creates 3-phase sequence: (1) PP pathfind to staging area, (2) lock pose to QuestNav, (3) AP precision to target
public class SmartDriveToPosition {
  private static final double PATHFINDER_SPEED_MULT = 0.6; // 60% max speed for safe approach with obstacle avoidance
  private static final double PATHFINDER_TIMEOUT_S = 8.0; // Max time for Phase 1 pathfinding
  private static final double PATHFINDER_ACCEL = 3.5; // m/s^2 linear acceleration
  private static final double PATHFINDER_ROTATION_MULT = 0.8; // 80% max rotation speed
  private static final double PATHFINDER_ROTATION_ACCEL = Math.PI; // rad/s^2 angular acceleration
  private static final double QUESTNAV_WAIT_TIMEOUT_S = 3.0; // Max wait for fresh QuestNav pose

  // Subsystems shared across all SmartDrive commands (factory pattern avoids passing 4+ params every time)
  private static PoseEstimatorSubsystem s_poseEstimator;
  private static RobotState s_robotState;
  private static DriveSubsystem s_driveSubsystem;
  private static QuestNavSubsystem s_questNavSubsystem;

  // Call once in RobotContainer constructor to configure subsystem references
  public static void configure(
      PoseEstimatorSubsystem poseEstimator,
      RobotState robotState,
      DriveSubsystem driveSubsystem,
      QuestNavSubsystem questNavSubsystem) {
    s_poseEstimator = poseEstimator;
    s_robotState = robotState;
    s_driveSubsystem = driveSubsystem;
    s_questNavSubsystem = questNavSubsystem;
  }

  // Creates command sequence: PathPlanner pathfind -> Force QuestNav pose -> AutoPilot precision
  public static Command create(Pose2d stagingPose, Pose2d finalTargetPose) {
    if (s_poseEstimator == null) {
      throw new IllegalStateException("SmartDriveToPosition not configured! Call configure() in RobotContainer first.");
    }

    SmartLogger.logReplay("SmartDrive/StagingPose", stagingPose);
    SmartLogger.logReplay("SmartDrive/FinalTargetPose", finalTargetPose);
    
    // Phase 1: PathPlanner dynamic pathfinding to staging area (8s max, handles obstacles)
    Command toStaging = AutoBuilder.pathfindToPose(stagingPose, createPathPlannerConstraints()).withTimeout(PATHFINDER_TIMEOUT_S);
    
    // Phase 3: AutoPilot precision navigation (sub-5cm accuracy, no timeout - will complete when at target)
    Command precisionNav = new AutoPilotToTargetCommand(finalTargetPose, s_driveSubsystem, s_poseEstimator, 0, 0, 0);

    return new SequentialCommandGroup(
        Commands.runOnce(() -> {
          s_robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
          SmartLogger.logReplay("SmartDrive/Status", "Phase 1: PathPlanner");
          SmartLogger.logReplay("SmartDrive/Phase", "PathPlanner");
        }),
        toStaging,
        
        // Phase 2: Active wait for fresh QuestNav pose that PASSES Kalman filter
        Commands.runOnce(() -> {
          s_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
          SmartLogger.logConsole("[SmartDrive] Waiting for QuestNav pose (processed by filter, max 3s)...");
        }),
        Commands.waitUntil(() -> {
          boolean questNavHasPose = s_poseEstimator.forceAcceptQuestNavPose();
          
          if (!questNavHasPose) {
            return false;
          }
          
          // FIXED: Check QuestNav fusion specifically, not generic update time
          double timeSinceQuestNavFusion = s_poseEstimator.getTimeSinceLastQuestNavFusion();
          
          if (timeSinceQuestNavFusion < 0.1) { // QuestNav fusion happened within 100ms
            Pose2d lockedPose = s_poseEstimator.getEstimatedPose();
            SmartLogger.logConsole("[SmartDrive] QuestNav fusion confirmed: " + formatPose(lockedPose));
            SmartLogger.logReplay("SmartDrive/ForcedPose", lockedPose);
            SmartLogger.logReplay("SmartDrive/ForceAcceptSuccess", true);
            return true;
          }
          
          SmartLogger.logReplay("SmartDrive/ForceAcceptRejectedByGate", true);
          return false;
        }).withTimeout(QUESTNAV_WAIT_TIMEOUT_S),
        
        Commands.runOnce(() -> {
          // Log final state after wait loop
          boolean finalSuccess = s_questNavSubsystem.isTracking();
          if (!finalSuccess) {
            SmartLogger.logConsoleError("[SmartDrive] WARNING: QuestNav never provided fresh pose (timeout after 3s)");
            SmartLogger.logReplay("SmartDrive/ForceAcceptTimeout", true);
          }
          
          s_robotState.setNavigationPhase(RobotState.NavigationPhase.PRECISION_PATH);
          SmartLogger.logReplay("SmartDrive/Status", "Phase 2: AutoPilot");
          SmartLogger.logConsole("[SmartDrive] AP start: " + formatPose(s_poseEstimator.getEstimatedPose()));
          SmartLogger.logConsole("[SmartDrive] AP target: " + formatPose(finalTargetPose));
          SmartLogger.logReplay("SmartDrive/Phase", "AutoPilot");
        }),
        precisionNav,
        Commands.runOnce(() -> {
          s_robotState.setNavigationPhase(RobotState.NavigationPhase.LOCKED);
          SmartLogger.logReplay("SmartDrive/Status", "Complete");
          s_driveSubsystem.lockWheels();
          SmartLogger.logReplay("SmartDrive/Complete", true);
        })
    ).finallyDo((interrupted) -> {
      s_robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
      SmartLogger.logReplay("SmartDrive/Status", interrupted ? "Interrupted" : "Complete");
      SmartLogger.logReplay("SmartDrive/Interrupted", interrupted);
    });
  }
  
  private static PathConstraints createPathPlannerConstraints() {
    return new PathConstraints(
        Constants.Swerve.MAX_TRANSLATION_SPEED_MPS * PATHFINDER_SPEED_MULT, PATHFINDER_ACCEL,
        Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SEC * PATHFINDER_ROTATION_MULT, PATHFINDER_ROTATION_ACCEL);
  }
  
  private static String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }
}
