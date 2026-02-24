package frc.robot.commands.drive;

// TODO (next session with robot):
// 1. Live test the forceAcceptQuestNavPose() path in phase 2 - confirm the Quest lock
//    actually improves final position accuracy vs skipping it.

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.util.SmartLogger;

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
          SmartLogger.logConsole("Phase 2: waiting for fresh Quest pose", "SmartDrive");
          s_robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
        }),

        Commands.sequence(
            Commands.waitUntil(() -> {
              boolean tracking = s_questNavSubsystem.isTracking();
              double age = s_questNavSubsystem.getMeasurementAge();
              return tracking && (age < 0.2);
            }),
            Commands.runOnce(() -> {
              boolean tracking = s_questNavSubsystem.isTracking();
              double age = s_questNavSubsystem.getMeasurementAge();
              boolean accepted = s_poseEstimator.forceAcceptQuestNavPose();
              lockAcquired[0] = accepted;
              
              if (accepted) {
                SmartLogger.logConsole("QuestNav locked: " + SmartLogger.formatPose(s_poseEstimator.getEstimatedPose()), "SmartDrive");
                SmartLogger.logReplay("SmartDrive/ForceAcceptSuccess", true);
              } else {
                SmartLogger.logConsoleError("SmartDrive: Force-accept failed | tracking=" + tracking + " age=" + String.format("%.3fs", age));
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
            SmartLogger.logConsoleError("SmartDrive: QuestNav timeout - proceeding with odometry-only");
            SmartLogger.logReplay("SmartDrive/Phase2Timeout", true);
          } else if (!lockAcquired[0]) {
            SmartLogger.logConsole("Quest pose lock failed - proceeding with caution", "SmartDrive");
            SmartLogger.logReplay("SmartDrive/Phase2LockFailed", true);
          } else {
            SmartLogger.logConsole("QuestNav lock acquired", "SmartDrive");
            SmartLogger.logReplay("SmartDrive/Phase2Success", true);
          }
        }),

        Commands.runOnce(() -> {
          s_robotState.setNavigationPhase(RobotState.NavigationPhase.PRECISION_PATH);
          SmartLogger.logConsole("Phase 3: AutoPilot to " + SmartLogger.formatPose(finalTargetPose), "SmartDrive");
          SmartLogger.logReplay("SmartDrive/PrecisionTarget", finalTargetPose);
          SmartLogger.logReplay("SmartDrive/Phase", "AutoPilot");
        }),
        new AutoPilotToTargetCommand(finalTargetPose, s_driveSubsystem, s_poseEstimator, 0, 0, 0),

        Commands.runOnce(() -> {
          SmartLogger.logConsole("Precision complete", "SmartDrive");
          SmartLogger.logReplay("SmartDrive/PrecisionComplete", true);
        })
    ).finallyDo((interrupted) -> {
      s_questNavSubsystem.resumeFusion();

      if (interrupted) {
        SmartLogger.logConsole("Precision phase interrupted - fusion resumed", "SmartDrive");
        SmartLogger.logReplay("SmartDrive/PrecisionInterrupted", true);
      }
    });
  }

  private static PathConstraints createPathPlannerConstraints() {
    // 3.5 m/s, 3.5 m/s^2, 540 deg/s, 720 deg/s^2
    return new PathConstraints(
        3.5,
        3.5,
        Math.toRadians(540.0),
        Math.toRadians(720.0));
  }
}