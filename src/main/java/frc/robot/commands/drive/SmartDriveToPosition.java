package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * Smart navigation to a target pose with safety checks and logging.
 */
public class SmartDriveToPosition {
  
  // Speed multipliers, field bounds, etc.
  private static final double IDEAL_VISION_DISTANCE_METERS = .75; // unused for now
  private static final double FAST_SPEED_MULT = 0.40;
  private static final double PRECISION_SPEED_MULT = 0.4;
  private static final double FIELD_LENGTH_METERS = Units.feetToMeters(54.0);
  private static final double FIELD_WIDTH_METERS = Units.feetToMeters(27.0);
  private static final double FIELD_MARGIN_METERS = 0.3;
  private static final double MAX_PATH_DISTANCE_METERS = 10.0;

  // Optional precision paths: targetPose -> (stagingPose, precisionPathName)
  private static final Map<Pose2d, PrecisionPathConfig> PRECISION_PATHS = new HashMap<>();

  // Simple container for precision config
  private static class PrecisionPathConfig {
    final Pose2d stagingPose;
    final String pathFileName;
    PrecisionPathConfig(Pose2d stagingPose, String pathFileName) {
      this.stagingPose = stagingPose;
      this.pathFileName = pathFileName;
    }
  }

  static {
    // EXAMPLE stub; you will fill this in once you have created the path in the GUI.
    // Suppose you create a path named "BlueReef17_Precision" that starts at a staging pose
    // about 1.5m away from BLUE_REEF_TAG_17.
    //
    // Replace the Pose2d below with the staging pose reported by the GUI,
    // and "BlueReef17_Precision" with your actual path file name.
    //
    // PRECISION_PATHS.put(
    //     Constants.StartingPositions.BLUE_REEF_TAG_17,
    //     new PrecisionPathConfig(
    //         new Pose2d(/* stagingX */, /* stagingY */, /* stagingHeading */),
    //         "BlueReef17_Precision"));

    // Precision path for BLUE_REEF_TAG_17:
    //   Path file: "Stage17toPrecise17"
    //   Staging pose: (3.359, 2.077, 60°)
    PRECISION_PATHS.put(
        Constants.StartingPositions.BLUE_REEF_TAG_17,
        new PrecisionPathConfig(
            new Pose2d(3.359, 2.077, Rotation2d.fromDegrees(60.0)),
            "Stage17toPrecise17"));
  }

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
    
    if (!isWithinField(target.getTranslation())) {
      DriverStation.reportError("Target outside field boundaries", false);
      
      return new SequentialCommandGroup(
          Commands.print("ERROR: Target outside field"),
          Commands.runOnce(() -> {
            SmartDashboard.putString("SmartDrive/Status", "ERROR: Out of bounds");
            robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
          })
      );
    }
    
    Logger.recordOutput("SmartDrive/TargetPose", target);
    Logger.recordOutput("SmartDrive/DistanceToTarget", distanceToTarget);

    // ===== LOOK FOR A PRECISION PATH FOR THIS TARGET (optional) =====
    PrecisionPathConfig precisionConfig = PRECISION_PATHS.get(target);
    if (precisionConfig != null) {
      Logger.recordOutput("SmartDrive/Precision/HasPrecisionPath", true);
      Logger.recordOutput("SmartDrive/Precision/StagingPose", precisionConfig.stagingPose);
      // Phase 1: pathfind to the staging pose
      Command toStaging = AutoBuilder.pathfindToPose(
          precisionConfig.stagingPose,
          createPrecisionConstraints())
          .withTimeout(8.0);

      // Phase 2: follow the short GUI precision path
      PathPlannerPath precisionPath;
      try {
        precisionPath = PathPlannerPath.fromPathFile(precisionConfig.pathFileName);
      } catch (Exception e) {
        DriverStation.reportError("Failed to load precision path: " + precisionConfig.pathFileName, e.getStackTrace());
        return Commands.none();
      }
      Command followPrecision = AutoBuilder.followPath(precisionPath)
          .withTimeout(5.0); // adjust if needed

      return new SequentialCommandGroup(
          Commands.runOnce(() -> {
            robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
            SmartDashboard.putString("SmartDrive/Status", "Pathfind + PrecisionPath");
            SmartDashboard.putNumber("SmartDrive/Progress", 0.0);
            Logger.recordOutput("SmartDrive/Mode", "PathfindThenPrecisionPath");
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

    Logger.recordOutput("SmartDrive/Precision/HasPrecisionPath", false);

    // ===== DEFAULT: SINGLE-PHASE DIRECT PATH TO TARGET =====
    return new SequentialCommandGroup(
        Commands.runOnce(() -> {
          robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
          SmartDashboard.putString("SmartDrive/Status", "Direct path to target");
          SmartDashboard.putNumber("SmartDrive/Progress", 0.0);
          Logger.recordOutput("SmartDrive/Mode", "SinglePhaseDirect");
        }),
        AutoBuilder.pathfindToPose(target, createPrecisionConstraints())
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
        Constants.Swerve.MAX_TRANSLATION_SPEED_MPS * PRECISION_SPEED_MULT, // 40% translational speed
        3.5, // translational accel (m/s^2)
        Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SEC * 0.8, // was 0.5 -> allow faster rotation
        Math.PI // was PI/2 -> double angular accel (~180°/s^2)
    );
  }
}
