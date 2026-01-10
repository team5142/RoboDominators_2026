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

public class SmartDriveToPosition {
  private static final double PATHFINDER_TIMEOUT_S = 8.0;
  private static final double QUESTNAV_WAIT_TIMEOUT_S = 3.0;

  private static PoseEstimatorSubsystem s_poseEstimator;
  private static RobotState s_robotState;
  private static DriveSubsystem s_driveSubsystem;
  private static QuestNavSubsystem s_questNavSubsystem;

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

  public static Command create(Pose2d stagingPose, Pose2d finalTargetPose) {
    if (s_poseEstimator == null) {
      throw new IllegalStateException("SmartDriveToPosition not configured! Call configure() in RobotContainer first.");
    }

    SmartLogger.logReplay("SmartDrive/StagingPose", stagingPose);
    SmartLogger.logReplay("SmartDrive/FinalTargetPose", finalTargetPose);
    
    Command toStaging = AutoBuilder.pathfindToPose(stagingPose, createPathPlannerConstraints()).withTimeout(PATHFINDER_TIMEOUT_S);
    
    return new SequentialCommandGroup(
        Commands.runOnce(() -> {
          s_robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
          SmartLogger.logReplay("SmartDrive/Status", "Phase 1: PathPlanner");
          SmartLogger.logReplay("SmartDrive/Phase", "PathPlanner");
        }),
        toStaging,
        
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
  
  public static Command createPrecisionPhase(Pose2d finalTargetPose) {
    if (s_poseEstimator == null) {
      throw new IllegalStateException("SmartDriveToPosition not configured!");
    }
    
    final boolean[] lockAcquired = {false};
    final boolean[] timedOut = {false};
    
    return new SequentialCommandGroup(
        Commands.runOnce(() -> {
          s_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
          s_questNavSubsystem.pauseFusion();
          SmartLogger.logConsole("[SmartDrive] Phase 2: Paused normal fusion, waiting for fresh Quest pose...");
          s_robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
        }),
        
        Commands.sequence(
            Commands.waitUntil(() -> {
              boolean tracking = s_questNavSubsystem.isTracking();
              double age = s_questNavSubsystem.getMeasurementAge();
              
              boolean freshPose = tracking && (age < 0.2);
              
              SmartLogger.logReplay("SmartDrive/QuestTracking", tracking);
              SmartLogger.logReplay("SmartDrive/QuestMeasurementAge", age);
              SmartLogger.logReplay("SmartDrive/WaitingForFreshPose", !freshPose);
              
              return freshPose;
            }),
            Commands.runOnce(() -> {
              boolean tracking = s_questNavSubsystem.isTracking();
              double age = s_questNavSubsystem.getMeasurementAge();
              boolean accepted = s_poseEstimator.forceAcceptQuestNavPose();
              lockAcquired[0] = accepted;
              
              if (accepted) {
                SmartLogger.logConsole("[SmartDrive] QuestNav locked: " + SmartLogger.formatPose(s_poseEstimator.getEstimatedPose()));
                SmartLogger.logReplay("SmartDrive/ForceAcceptSuccess", true);
              } else {
                SmartLogger.logConsole("[SmartDrive] WARNING: Force-accept failed after fresh pose detected");
                SmartLogger.logConsole("  Tracking: " + tracking + " | Age: " + String.format("%.3fs", age));
                SmartLogger.logReplay("SmartDrive/ForceAcceptFailed", true);
                SmartLogger.logReplay("SmartDrive/ForceAcceptFailed/Tracking", tracking);
                SmartLogger.logReplay("SmartDrive/ForceAcceptFailed/Age", age);
              }
            })
        ).raceWith(
            Commands.waitSeconds(QUESTNAV_WAIT_TIMEOUT_S).andThen(
                Commands.runOnce(() -> timedOut[0] = true)
            )
        ),
        
        Commands.runOnce(() -> {
          s_questNavSubsystem.resumeFusion();
          
          if (timedOut[0]) {
            SmartLogger.logConsoleError("[SmartDrive] QuestNav timeout (3s) - proceeding with odometry-only");
            SmartLogger.logReplay("SmartDrive/Phase2Timeout", true);
          } else if (!lockAcquired[0]) {
            SmartLogger.logConsole("[SmartDrive] Fresh pose detected but lock failed - proceeding with caution");
            SmartLogger.logReplay("SmartDrive/Phase2LockFailed", true);
          } else {
            SmartLogger.logConsole("[SmartDrive] QuestNav lock acquired successfully");
            SmartLogger.logReplay("SmartDrive/Phase2Success", true);
          }
        }),
        
        Commands.runOnce(() -> {
          s_robotState.setNavigationPhase(RobotState.NavigationPhase.PRECISION_PATH);
          SmartLogger.logConsole("[SmartDrive] Phase 3: AutoPilot to " + SmartLogger.formatPose(finalTargetPose));
          SmartLogger.logReplay("SmartDrive/PrecisionTarget", finalTargetPose);
          SmartLogger.logReplay("SmartDrive/Phase", "AutoPilot");
        }),
        new AutoPilotToTargetCommand(finalTargetPose, s_driveSubsystem, s_poseEstimator, 0, 0, 0),
        
        Commands.runOnce(() -> {
          SmartLogger.logConsole("[SmartDrive] Precision complete!");
          SmartLogger.logReplay("SmartDrive/PrecisionComplete", true);
        })
    ).finallyDo((interrupted) -> {
      s_questNavSubsystem.resumeFusion();
      
      if (interrupted) {
        SmartLogger.logConsole("[SmartDrive] Precision phase interrupted - fusion resumed");
        SmartLogger.logReplay("SmartDrive/PrecisionInterrupted", true);
      }
    });
  }
  
  private static PathConstraints createPathPlannerConstraints() {
    SmartLogger.logConsole("Using PathPlanner constraints: 3.5 m/s, 540 deg/s");
    
    return new PathConstraints(
        3.5,
        3.5,
        Math.toRadians(540.0),
        Math.toRadians(720.0));
  }
}