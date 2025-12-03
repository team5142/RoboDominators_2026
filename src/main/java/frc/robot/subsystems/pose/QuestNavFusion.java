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
    PoseFrame[] frames = questNavSubsystem.getAllUnreadFrames();
    
    if (frames == null || frames.length == 0) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return;
    }
    
    int framesProcessed = 0;
    int framesRejected = 0;
    
    // Get current speed for adaptive trust
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double currentSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    
    for (PoseFrame frame : frames) {
      if (frame == null) continue;
      
      // Get camera pose and transform to robot pose
      Pose3d cameraPose3d = frame.questPose3d();
      if (cameraPose3d == null) {
        Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", "Null Pose3d");
        framesRejected++;
        continue;
      }
      
      // Apply inverse transform to get robot pose
      Pose3d robotPose3d = cameraPose3d.transformBy(robotToQuest.inverse());
      
      Pose2d questPose2d = new Pose2d(
          robotPose3d.getX(),
          robotPose3d.getY(),
          robotPose3d.getRotation().toRotation2d());
      
      double timestamp = frame.dataTimestamp();
      
      // Innovation gating - reject outliers
      if (!gateQuestNavMeasurement(questPose2d, timestamp, currentSpeed)) {
        framesRejected++;
        continue;
      }
      
      // Get speed-adaptive standard deviations
      Matrix<N3, N1> stdDevs = getSpeedAdaptiveStdDevs(currentSpeed);
      
      // Add to pose estimator
      poseEstimator.addVisionMeasurement(questPose2d, timestamp, stdDevs);
      framesProcessed++;
    }
    
    Logger.recordOutput("PoseEstimator/QuestNavFramesProcessed", framesProcessed);
    Logger.recordOutput("PoseEstimator/QuestNavFramesRejected", framesRejected);
    Logger.recordOutput("PoseEstimator/QuestNavUsed", framesProcessed > 0);
    Logger.recordOutput("PoseEstimator/CurrentSpeed", currentSpeed);
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
   * Get speed-adaptive standard deviations
   * Higher speed = lower trust (QuestNav has more latency)
   */
  private Matrix<N3, N1> getSpeedAdaptiveStdDevs(double speedMPS) {
    double baseTrustX = QUESTNAV_STD_DEVS[0];
    double baseTrustY = QUESTNAV_STD_DEVS[1];
    double baseTrustTheta = QUESTNAV_STD_DEVS[2];
    
    // Penalize trust at high speeds
    double speedPenalty = 0.05 * speedMPS;
    double xyTrust = baseTrustX + speedPenalty;
    double thetaTrust = baseTrustTheta + (speedPenalty * 0.5);
    
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/XY", xyTrust);
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/Theta", thetaTrust);
    
    return VecBuilder.fill(xyTrust, xyTrust, thetaTrust);
  }
}
