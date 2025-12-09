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
 * Responsibilities:
 * - Process all unread QuestNav frames each cycle
 * - Apply robot-to-camera transform (manual for old API)
 * - Innovation gating (reject bad measurements)
 * - Speed-adaptive trust (lower trust at high speeds)
 */
public class QuestNavFusion {
  
  private final QuestNavSubsystem questNavSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final SwerveDrivePoseEstimator poseEstimator;
  
  // Transform for manual application (older API)
  private final Transform3d robotToQuest;
  
  // Speed threshold for accepting QuestNav
  private static final double QUESTNAV_SPEED_THRESHOLD_MPS = 0.15; // Accept only when nearly stopped
  
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
   * Process all available QuestNav frames
   * Called every cycle from PoseEstimatorSubsystem.periodic()
   */
  public void processFrames() {
    // Check current speed - only accept QuestNav when stopped/slow
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double currentSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double rotationSpeed = Math.abs(speeds.omegaRadiansPerSecond);
    
    // Reject QuestNav if robot is moving or rotating quickly
    if (currentSpeed > QUESTNAV_SPEED_THRESHOLD_MPS || rotationSpeed > 0.2) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", 
          String.format("Moving too fast (speed=%.2f m/s, rotation=%.2f rad/s)", currentSpeed, rotationSpeed));
      Logger.recordOutput("PoseEstimator/QuestNav/CurrentSpeed", currentSpeed);
      Logger.recordOutput("PoseEstimator/QuestNav/RejectedDueToSpeed", true);
      return;
    }
    
    // Robot is stopped - accept QuestNav
    Logger.recordOutput("PoseEstimator/QuestNav/RejectedDueToSpeed", false);
    
    PoseFrame[] frames = questNavSubsystem.getAllUnreadFrames();
    
    if (frames == null || frames.length == 0) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return;
    }
    
    // Only process the most recent frame
    PoseFrame latestFrame = frames[frames.length - 1];
    
    if (frames.length > 1) {
      Logger.recordOutput("PoseEstimator/QuestNavFramesSkipped", frames.length - 1);
    }
    
    if (latestFrame == null) {
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", "Null Frame");
      Logger.recordOutput("PoseEstimator/QuestNavFramesProcessed", 0);
      Logger.recordOutput("PoseEstimator/QuestNavFramesRejected", 1);
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return;
    }
    
    Pose3d cameraPose3d = latestFrame.questPose3d();
    if (cameraPose3d == null) {
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", "Null Pose3d");
      Logger.recordOutput("PoseEstimator/QuestNavFramesProcessed", 0);
      Logger.recordOutput("PoseEstimator/QuestNavFramesRejected", 1);
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return;
    }
    
    Pose3d robotPose3d = cameraPose3d.transformBy(robotToQuest.inverse());
    
    Pose2d questPose2d = new Pose2d(
        robotPose3d.getX(),
        robotPose3d.getY(),
        robotPose3d.getRotation().toRotation2d());
    
    double timestamp = latestFrame.dataTimestamp();
    
    // Innovation gating
    if (!gateQuestNavMeasurement(questPose2d, timestamp, currentSpeed)) {
      Logger.recordOutput("PoseEstimator/QuestNavFramesProcessed", 0);
      Logger.recordOutput("PoseEstimator/QuestNavFramesRejected", 1);
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return;
    }
    
    // Get standard deviations (lower trust since robot is stopped - QuestNav is most reliable here)
    Matrix<N3, N1> stdDevs = getStoppedQuestNavStdDevs();
    
    // Add to pose estimator
    poseEstimator.addVisionMeasurement(questPose2d, timestamp, stdDevs);
    
    Logger.recordOutput("PoseEstimator/QuestNavFramesProcessed", 1);
    Logger.recordOutput("PoseEstimator/QuestNavFramesRejected", 0);
    Logger.recordOutput("PoseEstimator/QuestNavUsed", true);
    Logger.recordOutput("PoseEstimator/CurrentSpeed", currentSpeed);
    Logger.recordOutput("PoseEstimator/QuestNav/LatestPose", questPose2d);
    Logger.recordOutput("PoseEstimator/QuestNav/LatestTimestamp", timestamp);
  }
  
  /**
   * Innovation gating - reject measurements too far from prediction
   */
  private boolean gateQuestNavMeasurement(Pose2d measurement, double timestamp, double currentSpeed) {
    Pose2d predictedPose = poseEstimator.getEstimatedPosition();
    
    double posError = measurement.getTranslation().getDistance(predictedPose.getTranslation());
    double rotError = Math.abs(measurement.getRotation().minus(predictedPose.getRotation()).getRadians());
    
    // Dynamic gates based on speed (allow more error when moving fast)
    double posGate = 0.6 + 0.5 * currentSpeed + 0.1;
    double rotGate = Math.toRadians(15.0);
    
    boolean passed = posError <= posGate && rotError <= rotGate;
    
    if (!passed) {
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", 
          String.format("Gate failed: pos=%.2fm (gate=%.2fm), rot=%.1f° (gate=%.1f°)", 
              posError, posGate, Math.toDegrees(rotError), Math.toDegrees(rotGate)));
      Logger.recordOutput("PoseEstimator/QuestNav/RejectedPose", measurement);
    }
    
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/PosError", posError);
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/RotError", Math.toDegrees(rotError));
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/PosGate", posGate);
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/Passed", passed);
    
    return passed;
  }
  
  /**
   * Get standard deviations for stopped robot
   * Higher trust (lower std dev) when robot is stopped since QuestNav is most accurate
   */
  private Matrix<N3, N1> getStoppedQuestNavStdDevs() {
    // When stopped, trust QuestNav more (lower std dev = higher trust)
    double xyTrust = QUESTNAV_STD_DEVS[0] * 0.5; // 50% of normal (higher trust)
    double thetaTrust = QUESTNAV_STD_DEVS[2] * 0.5;
    
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/XY", xyTrust);
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/Theta", thetaTrust);
    
    return VecBuilder.fill(xyTrust, xyTrust, thetaTrust);
  }
}
