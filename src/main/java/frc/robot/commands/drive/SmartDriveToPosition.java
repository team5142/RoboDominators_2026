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
  private static final double PATHFINDER_TIMEOUT_S = 8.0; // Max time for Phase 1 pathfinding
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
    
    return new SequentialCommandGroup(
        Commands.runOnce(() -> {
          s_robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
          SmartLogger.logReplay("SmartDrive/Status", "Phase 1: PathPlanner");
          SmartLogger.logReplay("SmartDrive/Phase", "PathPlanner");
        }),
        toStaging,
        
        // Phase 2+3: Shared precision logic
        createPrecisionPhase(finalTargetPose),
        
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
  
  /**
   * Creates Phase 2+3 (QuestNav lock + AutoPilot precision)
   * Shared by both create() and PathPlanner event markers
   * 
   * Usage:
   * 1. Teleop: SmartDriveToPosition.create() calls this after PathPlanner
   * 2. Auto: PathPlanner event "SmartPrecision:Tag17" calls this directly
   * 
   * @param finalTargetPose The precision target (e.g., PRECISE_17_POSE)
   * @return Command that waits for QuestNav lock, then uses AutoPilot
   */
  public static Command createPrecisionPhase(Pose2d finalTargetPose) {
    if (s_poseEstimator == null) {
      throw new IllegalStateException("SmartDriveToPosition not configured!");
    }
    
    return new SequentialCommandGroup(
        // Phase 2: Wait for QuestNav lock
        Commands.runOnce(() -> {
          s_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
          SmartLogger.logConsole("[SmartDrive] Phase 2: Waiting for QuestNav lock...");
          s_robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
        }),
        Commands.waitUntil(() -> {
          boolean questNavHasPose = s_poseEstimator.forceAcceptQuestNavPose();
          if (!questNavHasPose) return false;
          
          double timeSinceQuestNavFusion = s_poseEstimator.getTimeSinceLastQuestNavFusion();
          if (timeSinceQuestNavFusion < 0.1) {
            SmartLogger.logConsole("[SmartDrive] QuestNav locked: " + formatPose(s_poseEstimator.getEstimatedPose()));
            return true;
          }
          return false;
        }).withTimeout(QUESTNAV_WAIT_TIMEOUT_S),
        
        Commands.runOnce(() -> {
          boolean finalSuccess = s_questNavSubsystem.isTracking();
          if (!finalSuccess) {
            SmartLogger.logConsoleError("[SmartDrive] WARNING: QuestNav timeout (3s)");
            SmartLogger.logReplay("SmartDrive/ForceAcceptTimeout", true);
          }
        }),
        
        // Phase 3: AutoPilot precision
        Commands.runOnce(() -> {
          s_robotState.setNavigationPhase(RobotState.NavigationPhase.PRECISION_PATH);
          SmartLogger.logConsole("[SmartDrive] Phase 3: AutoPilot to " + formatPose(finalTargetPose));
          SmartLogger.logReplay("SmartDrive/PrecisionTarget", finalTargetPose);
          SmartLogger.logReplay("SmartDrive/Phase", "AutoPilot");
        }),
        new AutoPilotToTargetCommand(finalTargetPose, s_driveSubsystem, s_poseEstimator, 0, 0, 0),
        
        Commands.runOnce(() -> {
          SmartLogger.logConsole("[SmartDrive] Precision complete!");
          SmartLogger.logReplay("SmartDrive/PrecisionComplete", true);
        })
    ).finallyDo((interrupted) -> {
      if (interrupted) {
        SmartLogger.logConsole("[SmartDrive] Precision phase interrupted");
        SmartLogger.logReplay("SmartDrive/PrecisionInterrupted", true);
      }
    });
  }
  
  private static PathConstraints createPathPlannerConstraints() {
    // Use settings.json values directly (no file read needed)
    SmartLogger.logConsole("Using PathPlanner constraints: 3.5 m/s, 540 deg/s");
    
    return new PathConstraints(
        3.5,                     // defaultMaxVel (m/s) from settings.json
        3.5,                     // defaultMaxAccel (m/s²)
        Math.toRadians(540.0),   // defaultMaxAngVel (rad/s) - converted from deg/s
        Math.toRadians(720.0));  // defaultMaxAngAccel (rad/s²) - converted from deg/s²
  }
  
  private static String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }
}
