package frc.robot.commands.drive;

import static frc.robot.Constants.QuestNav.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Post-path precision correction using QuestNav
 * 
 * After PathPlanner completes a path, compare final odometry pose vs QuestNav pose.
 * If error exceeds threshold (3cm), generate a small correction move to align precisely.
 * 
 * This is Phase 3 of the precision navigation strategy:
 * - Phase 1: Pathfind to staging pose (fast, rough)
 * - Phase 2: Follow precision path (predefined, accurate)
 * - Phase 3: QuestNav-based final correction (< 3cm accuracy)
 */
public class PostPathCorrection {
  
  /**
   * Attempt precision correction after path completion
   * 
   * @param poseEstimator Current pose from odometry
   * @param questNav QuestNav subsystem for ground truth
   * @return Command that corrects pose or does nothing if already accurate
   */
  public static Command attemptCorrection(
      PoseEstimatorSubsystem poseEstimator,
      QuestNavSubsystem questNav) {
    
    return Commands.sequence(
        // Wait for robot to settle after path
        Commands.waitSeconds(POST_PATH_SETTLE_TIME_S),
        
        // Check if correction needed
        Commands.either(
            // Correction needed - generate small move
            createCorrectionMove(poseEstimator, questNav),
            
            // No correction needed - already accurate
            Commands.runOnce(() -> {
              System.out.println("Post-path correction: Already within tolerance");
              Logger.recordOutput("PostPathCorrection/Needed", false);
            }),
            
            // Condition: Is error > threshold?
            () -> needsCorrection(poseEstimator, questNav)
        )
    );
  }
  
  /**
   * Check if correction is needed
   */
  private static boolean needsCorrection(
      PoseEstimatorSubsystem poseEstimator,
      QuestNavSubsystem questNav) {
    
    Pose2d odometryPose = poseEstimator.getEstimatedPose();
    Pose2d questNavPose = questNav.getRobotPose().orElse(null);
    
    if (questNavPose == null) {
      System.err.println("Post-path correction: QuestNav unavailable");
      Logger.recordOutput("PostPathCorrection/QuestNavUnavailable", true);
      return false;
    }
    
    double posError = odometryPose.getTranslation().getDistance(questNavPose.getTranslation());
    
    Logger.recordOutput("PostPathCorrection/OdometryPose", odometryPose);
    Logger.recordOutput("PostPathCorrection/QuestNavPose", questNavPose);
    Logger.recordOutput("PostPathCorrection/Error", posError);
    
    // Check if error is in correctable range
    if (posError < POST_PATH_CORRECTION_THRESHOLD_M) {
      System.out.println(String.format("Post-path: Already accurate (%.1fcm error)", posError * 100));
      return false;
    }
    
    if (posError > POST_PATH_CORRECTION_MAX_M) {
      System.err.println(String.format("Post-path: Error too large (%.1fcm) - something went wrong!", posError * 100));
      Logger.recordOutput("PostPathCorrection/ErrorTooLarge", true);
      return false;
    }
    
    System.out.println(String.format("Post-path: Correction needed (%.1fcm error)", posError * 100));
    return true;
  }
  
  /**
   * Create a small correction move to QuestNav position
   */
  private static Command createCorrectionMove(
      PoseEstimatorSubsystem poseEstimator,
      QuestNavSubsystem questNav) {
    
    return Commands.runOnce(() -> {
      Pose2d odometryPose = poseEstimator.getEstimatedPose();
      Pose2d questNavPose = questNav.getRobotPose().orElse(null);
      
      if (questNavPose == null) {
        System.err.println("Post-path correction: QuestNav lost during correction");
        return;
      }
      
      double posError = odometryPose.getTranslation().getDistance(questNavPose.getTranslation());
      
      System.out.println("========== POST-PATH CORRECTION ==========");
      System.out.println(String.format("Odometry: (%.2f, %.2f, %.1f deg)", 
          odometryPose.getX(), odometryPose.getY(), odometryPose.getRotation().getDegrees()));
      System.out.println(String.format("QuestNav: (%.2f, %.2f, %.1f deg)", 
          questNavPose.getX(), questNavPose.getY(), questNavPose.getRotation().getDegrees()));
      System.out.println(String.format("Error: %.1f cm", posError * 100));
      System.out.println("Generating correction move...");
      System.out.println("=========================================");
      
      // Generate small correction path to QuestNav position
      PathConstraints correctionConstraints = new PathConstraints(
          Constants.Swerve.MAX_TRANSLATION_SPEED_MPS * 0.2,  // 20% speed (gentle)
          2.0,  // Low acceleration
          Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SEC * 0.3,  // 30% rotation speed
          Math.PI / 2  // Low angular acceleration
      );
      
      Command correctionPath = AutoBuilder.pathfindToPose(
          questNavPose,
          correctionConstraints)
          .withTimeout(POST_PATH_CORRECTION_TIMEOUT_S);
      
      correctionPath.schedule();
      
      Logger.recordOutput("PostPathCorrection/Applied", true);
      Logger.recordOutput("PostPathCorrection/ErrorCorrected", posError);
    });
  }
}
