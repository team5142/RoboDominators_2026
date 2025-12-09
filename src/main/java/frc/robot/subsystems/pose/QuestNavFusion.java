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
  private final SwerveDrivePoseEstimator poseEstimator;
  
  // Transform for manual application (older API)
  private final Transform3d robotToQuest;
  
  public QuestNavFusion(
      QuestNavSubsystem questNavSubsystem,
      DriveSubsystem driveSubsystem,
      SwerveDrivePoseEstimator poseEstimator) {
    
    this.questNavSubsystem = questNavSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.poseEstimator = poseEstimator;
    
    // Store transform for manual application
    this.robotToQuest = new Transform3d(
        new Translation3d(QUEST_X_METERS, QUEST_Y_METERS, QUEST_Z_METERS),
        new Rotation3d(0, 0, Math.toRadians(QUEST_YAW_DEG))
    );
  }
  
  /**
   * Check for new QuestNav frames and process the latest one
   * Called every cycle (20ms) from PoseEstimatorSubsystem.periodic()
   * 
   * Only processes the MOST RECENT frame - older frames are discarded
   * to prevent accumulating stale measurements
   */
  public void processFrames() {
    // STRICT VELOCITY GATING - only accept QuestNav when nearly stationary
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double rotationSpeed = Math.abs(speeds.omegaRadiansPerSecond);
    
    // Reject if moving too fast (especially during rotation - latency kills accuracy)
    if (linearSpeed > QUESTNAV_MAX_LINEAR_SPEED_MPS || rotationSpeed > QUESTNAV_MAX_OMEGA_RAD_PER_SEC) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", 
          String.format("Moving (linear=%.3f m/s, omega=%.3f rad/s)", linearSpeed, rotationSpeed));
      Logger.recordOutput("PoseEstimator/QuestNav/RejectedDueToSpeed", true);
      return;
    }
    
    // Robot is nearly stopped - QuestNav is safe to use
    Logger.recordOutput("PoseEstimator/QuestNav/RejectedDueToSpeed", false);
    
    PoseFrame[] frames = questNavSubsystem.getAllUnreadFrames();
    
    if (frames == null || frames.length == 0) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return;
    }
    
    // Only process most recent frame (discard stale data)
    PoseFrame latestFrame = frames[frames.length - 1];
    
    if (frames.length > 1) {
      Logger.recordOutput("PoseEstimator/QuestNavFramesSkipped", frames.length - 1);
    }
    
    if (latestFrame == null) {
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", "Null Frame");
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return;
    }
    
    // Get camera pose from QuestNav
    Pose3d cameraPose3d = latestFrame.questPose3d();
    if (cameraPose3d == null) {
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", "Null Pose3d");
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return;
    }
    
    // Transform to robot pose
    Pose3d robotPose3d = cameraPose3d.transformBy(robotToQuest.inverse());
    
    Pose2d questPose2d = new Pose2d(
        robotPose3d.getX(),
        robotPose3d.getY(),
        robotPose3d.getRotation().toRotation2d());
    
    // LATENCY COMPENSATION - critical for preventing controller instability
    double rawTimestamp = latestFrame.dataTimestamp();
    double compensatedTimestamp = rawTimestamp - (QUESTNAV_LATENCY_MS / 1000.0);
    
    Logger.recordOutput("PoseEstimator/QuestNav/RawTimestamp", rawTimestamp);
    Logger.recordOutput("PoseEstimator/QuestNav/CompensatedTimestamp", compensatedTimestamp);
    Logger.recordOutput("PoseEstimator/QuestNav/LatencyMS", QUESTNAV_LATENCY_MS);
    
    // Innovation gating - reject outliers
    if (!gateQuestNavMeasurement(questPose2d, compensatedTimestamp)) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return;
    }
    
    // Get trust level (stopped robot has high trust)
    Matrix<N3, N1> stdDevs = getStoppedQuestNavStdDevs();
    
    // Add to pose estimator with latency-compensated timestamp
    poseEstimator.addVisionMeasurement(questPose2d, compensatedTimestamp, stdDevs);
    
    Logger.recordOutput("PoseEstimator/QuestNavUsed", true);
    Logger.recordOutput("PoseEstimator/QuestNav/LatestPose", questPose2d);
    Logger.recordOutput("PoseEstimator/QuestNav/LinearSpeed", linearSpeed);
    Logger.recordOutput("PoseEstimator/QuestNav/RotationSpeed", rotationSpeed);
  }
  
  /**
   * Innovation gating - reject measurements too far from prediction
   */
  private boolean gateQuestNavMeasurement(Pose2d measurement, double timestamp) {
    Pose2d predictedPose = poseEstimator.getEstimatedPosition();
    
    double posError = measurement.getTranslation().getDistance(predictedPose.getTranslation());
    double rotError = Math.abs(measurement.getRotation().minus(predictedPose.getRotation()).getRadians());
    
    // Conservative gates (robot should be stopped, so error should be small)
    double posGate = 0.5;  // 50cm max position error
    double rotGate = Math.toRadians(15.0);  // 15 deg max rotation error
    
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
}
