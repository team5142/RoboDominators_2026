package frc.robot.subsystems.pose;

import static frc.robot.Constants.QuestNav.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem; // ADD THIS IMPORT
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.Logger;

/**
 * Fuses QuestNav SLAM measurements into pose estimator
 * 
 * KEY PHILOSOPHY:
 * - QuestNav is treated like HIGH-ACCURACY VISION (like AprilTags)
 * - NOT treated like real-time odometry (too much latency)
 * - Only fused when robot is nearly stationary (strict velocity gating)
 * - Latency-compensated timestamps prevent controller chasing stale data
 * - Gentle correction (moderate trust) prevents fighting PathPlanner
 * 
 * Responsibilities:
 * - Check for new QuestNav frames every cycle (called from PoseEstimatorSubsystem.periodic())
 * - Process ONLY the latest frame (discard older stale frames)
 * - Strict velocity gating (reject during motion, especially rotation)
 * - Apply robot-to-camera transform
 * - Innovation gating (reject outliers)
 * - Provide different trust levels for different scenarios
 */
public class QuestNavFusion {
  
  private final QuestNavSubsystem questNavSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final SwerveDrivePoseEstimator swervePoseEstimator; // WPILib estimator
  private final PoseEstimatorSubsystem poseEstimatorSubsystem; // OUR subsystem (for notifications)
  
  private final Transform3d robotToQuest;
  
  public QuestNavFusion(
      QuestNavSubsystem questNavSubsystem,
      DriveSubsystem driveSubsystem,
      SwerveDrivePoseEstimator swervePoseEstimator,
      PoseEstimatorSubsystem poseEstimatorSubsystem) { // ADD THIS PARAMETER
    
    this.questNavSubsystem = questNavSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.swervePoseEstimator = swervePoseEstimator;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem; // STORE IT
    
    // Store transform for manual application
    this.robotToQuest = new Transform3d(
        new Translation3d(QUEST_X_METERS, QUEST_Y_METERS, QUEST_Z_METERS),
        new Rotation3d(0, 0, Math.toRadians(QUEST_YAW_DEG))
    );
  }
  
  /**
   * Check for new QuestNav frames and process all of them
   * Called every cycle (20ms) from PoseEstimatorSubsystem.periodic()
   */
  public void processFrames() {
    PoseFrame[] frames = questNavSubsystem.getAllUnreadFrames();
    
    if (frames == null || frames.length == 0) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return;
    }
    
    // Process ALL frames (sister team's approach)
    int processedCount = 0;
    int rejectedCount = 0;
    Pose2d lastAcceptedPose = null; // NEW: Track last accepted pose
    
    for (PoseFrame frame : frames) {
      if (frame == null) continue;
      
      Pose3d cameraPose3d = frame.questPose3d();
      if (cameraPose3d == null) continue;
      
      // Transform to robot pose
      Pose3d robotPose3d = cameraPose3d.transformBy(robotToQuest.inverse());
      Pose2d questPose2d = new Pose2d(
          robotPose3d.getX(),
          robotPose3d.getY(),
          robotPose3d.getRotation().toRotation2d());
      
      // Latency compensation
      double captureTimestamp = frame.dataTimestamp();
      double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      double actualAge = currentTime - captureTimestamp;
      
      // Reject if too old (> 500ms)
      if (actualAge > 0.5 || captureTimestamp > currentTime) {
        rejectedCount++;
        continue;
      }
      
      // Innovation gating (STRICT)
      if (!gateQuestNavMeasurement(questPose2d, captureTimestamp)) {
        rejectedCount++;
        continue;
      }
      
      // Get trust level based on robot state
      Matrix<N3, N1> stdDevs = getTrustForCurrentState();
      
      // Add to pose estimator
      swervePoseEstimator.addVisionMeasurement(questPose2d, captureTimestamp, stdDevs);
      processedCount++;
      lastAcceptedPose = questPose2d; // NEW: Store for logging
    }
    
    // NEW: Log the latest accepted pose
    if (lastAcceptedPose != null) {
      Logger.recordOutput("PoseEstimator/QuestNav/LatestPose", lastAcceptedPose);
      poseEstimatorSubsystem.notifyQuestNavFusionOccurred(); // NOW THIS WORKS!
    }
    
    Logger.recordOutput("PoseEstimator/QuestNav/FramesProcessed", processedCount);
    Logger.recordOutput("PoseEstimator/QuestNav/FramesRejected", rejectedCount);
    Logger.recordOutput("PoseEstimator/QuestNavUsed", processedCount > 0);
  }
  
  /**
   * Innovation gating - reject measurements too far from prediction
   */
  private boolean gateQuestNavMeasurement(Pose2d measurement, double timestamp) {
    Pose2d predictedPose = swervePoseEstimator.getEstimatedPosition(); // Use swervePoseEstimator
    
    double posError = measurement.getTranslation().getDistance(predictedPose.getTranslation());
    double rotError = Math.abs(measurement.getRotation().minus(predictedPose.getRotation()).getRadians());
    
    // STRICT gates (50cm position, 15° rotation)
    double posGate = 0.5;  // FIX: 50cm (not 2m!)
    double rotGate = Math.toRadians(15.0);  // FIX: 15° (not 30°!)
    
    boolean passed = posError <= posGate && rotError <= rotGate;
    
    if (!passed) {
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", 
          String.format("Gate failed: pos=%.2fm (gate=%.2fm), rot=%.1f deg (gate=%.1f deg)", 
              posError, posGate, Math.toDegrees(rotError), Math.toDegrees(rotGate)));
      Logger.recordOutput("PoseEstimator/QuestNav/RejectedPose", measurement);
    }
    
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/PosError", posError);
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/RotError", Math.toDegrees(rotError));
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/Passed", passed);
    
    return passed;
  }
  
  /**
   * Get standard deviations for stopped robot
   * High trust in both XY and theta (QuestNav excels when stationary)
   */
  private Matrix<N3, N1> getStoppedQuestNavStdDevs() {
    double xyTrust = QUESTNAV_STD_DEVS_STOPPED[0];     // 2cm
    double thetaTrust = QUESTNAV_STD_DEVS_STOPPED[2];  // ~2 deg
    
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/XY", xyTrust);
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/Theta", thetaTrust);
    
    return VecBuilder.fill(xyTrust, xyTrust, thetaTrust);
  }
  
  /**
   * Get standard deviations for initial alignment (very high trust)
   * Used at auto start for one-time pose initialization
   */
  public Matrix<N3, N1> getInitialAlignmentStdDevs() {
    double xyTrust = QUESTNAV_STD_DEVS_INITIAL[0];     // 1cm
    double thetaTrust = QUESTNAV_STD_DEVS_INITIAL[2];  // ~1 deg
    
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/XY_Initial", xyTrust);
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/Theta_Initial", thetaTrust);
    
    return VecBuilder.fill(xyTrust, xyTrust, thetaTrust);
  }
  
  /**
   * Get trust level based on current robot state
   */
  private Matrix<N3, N1> getTrustForCurrentState() {
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);
    
    // High trust when stopped
    if (linearSpeed < 0.05 && angularSpeed < 0.05) {
      return VecBuilder.fill(
          QUESTNAV_STD_DEVS_STOPPED[0],  // 2cm XY
          QUESTNAV_STD_DEVS_STOPPED[1],  
          QUESTNAV_STD_DEVS_STOPPED[2]); // 2° theta
    }
    
    // Moderate trust during motion
    return VecBuilder.fill(
        QUESTNAV_STD_DEVS[0],  // 8cm XY
        QUESTNAV_STD_DEVS[1],  
        QUESTNAV_STD_DEVS[2]); // 4° theta
  }
}
