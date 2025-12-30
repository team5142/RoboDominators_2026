package frc.robot.subsystems.pose;

import static frc.robot.Constants.QuestNav.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * QuestNav SLAM Fusion - Treats QuestNav like HIGH-ACCURACY, LATENT VISION
 * 
 * KEY PHILOSOPHY (matches AprilTag vision fusion strategy):
 * 
 * 1) VELOCITY GATING (ENFORCED):
 *    - ONLY fuse when robot is nearly stationary (< 0.12 m/s linear, < 0.3 rad/s angular)
 *    - Prevents fighting PathPlanner during aggressive autonomous motion
 *    - Allows convergence during brief pauses between path segments
 *    - Checked BEFORE peeking measurement to preserve it for next stationary period
 * 
 * 2) INNOVATION GATING (TIME-AWARE, WITH REACQUIRE):
 *    - Compare QuestNav measurement to CURRENT pose estimate (not historical)
 *    - MOVING: Tight gates (10cm base + motion expansion)
 *    - STOPPED: Wide reacquire gates (65cm pos, 25° rot) for rapid convergence
 *    - Conservative approximation (simpler than pose history buffer)
 * 
 * 3) TRUST MODELING (MOTION-DEPENDENT):
 *    - STOPPED: Very high trust (2cm XY, 1.7° theta) - like stationary AprilTag
 *    - MOVING: Degraded trust based on speed - gentle correction, not rejection
 *    - INITIAL: Maximum trust (1cm XY, 1.1° theta) - one-time alignment at auto start
 * 
 * 4) LATENCY COMPENSATION:
 *    - Timestamps are RECEIVE-ALIGNED (FPGA time when frame arrived)
 *    - QuestNav does NOT provide reliable capture timestamps
 *    - Conservative approach: treat latency as zero rather than guessing
 *    - SwerveDrivePoseEstimator applies measurement at receive time
 * 
 * 5) FRAME HANDLING (PEEK-THEN-ACKNOWLEDGE):
 *    - QuestNavSubsystem drains ALL frames, caches ONLY latest (single consumer)
 *    - Each frame has monotonic sequence number
 *    - peekLatestMeasurement() allows inspection WITHOUT consuming
 *    - acknowledgeMeasurement() marks consumed ONLY after successful fusion
 *    - Rejected measurements preserved for next cycle (no frame burning)
 * 
 * COORDINATE FRAMES:
 * - QuestNavSubsystem outputs FIELD-SPACE robot poses (transform already applied)
 * - No additional transforms needed in fusion layer
 * 
 * EXPECTED BEHAVIOR:
 * - No "tug-of-war" during PathPlanner paths (velocity gating prevents fusion)
 * - Rapid convergence when stopped (reacquire mode with wide gates)
 * - No frame starvation (peek/ack preserves rejected measurements)
 * - Each QuestNav frame fused AT MOST ONCE (consume-once semantics)
 */
public class QuestNavFusion {
  
  private final QuestNavSubsystem questNavSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final SwerveDrivePoseEstimator swervePoseEstimator;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  
  public QuestNavFusion(
      QuestNavSubsystem questNavSubsystem,
      DriveSubsystem driveSubsystem,
      SwerveDrivePoseEstimator swervePoseEstimator,
      PoseEstimatorSubsystem poseEstimatorSubsystem) {
    
    this.questNavSubsystem = questNavSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.swervePoseEstimator = swervePoseEstimator;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
  }
  
  /**
   * PEEK-THEN-ACKNOWLEDGE: Process latest unconsumed QuestNav measurement
   * Called every cycle (20ms) from PoseEstimatorSubsystem.periodic()
   * 
   * NEW ORDERING (fixes starvation):
   * 1. Check velocity gate (preserves measurement if moving)
   * 2. PEEK measurement (non-consuming check)
   * 3. Age check
   * 4. Innovation gating (WIDENED when stopped for reacquire)
   * 5. If accepted -> ACKNOWLEDGE + addVisionMeasurement
   */
  public void processFrames() {
    // 1) VELOCITY GATING: Check BEFORE peeking
    if (!velocityGatePass()) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", "Velocity gate failed");
      return;
    }
    
    // 2) PEEK (non-consuming): Check if measurement exists
    java.util.Optional<QuestNavSubsystem.QuestMeasurement> measurement = 
        questNavSubsystem.peekLatestMeasurement();
    
    if (!measurement.isPresent()) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", "No unconsumed measurement");
      return;
    }
    
    QuestNavSubsystem.QuestMeasurement meas = measurement.get();
    Pose2d pose = meas.pose;
    double timestamp = meas.timestampFPGA;
    long sequence = meas.sequence;
    
    // 3) Age check
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double age = currentTime - timestamp;
    
    if (age > 0.5 || age < 0) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", 
          "Stale/invalid age: " + String.format("%.3fs", age));
      return; // Do NOT acknowledge - preserve for next cycle
    }
    
    // 4) INNOVATION GATING (with REACQUIRE mode when stopped)
    if (!innovationGatePass(pose, age)) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return; // Do NOT acknowledge - preserve for next cycle
    }
    
    // 5) ACCEPTED - Now we can acknowledge + fuse
    if (!questNavSubsystem.acknowledgeMeasurement(sequence)) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", "Acknowledge failed (race condition)");
      return;
    }
    
    // Get trust level based on robot state
    Matrix<N3, N1> stdDevs = getTrustForCurrentState(age);
    
    // Add to pose estimator with RECEIVE-ALIGNED timestamp
    swervePoseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    
    // Notify parent subsystem for tracking
    poseEstimatorSubsystem.notifyQuestNavFusionOccurred(timestamp);
    
    Logger.recordOutput("PoseEstimator/QuestNav/LatestPose", pose);
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAge", age);
    Logger.recordOutput("PoseEstimator/QuestNav/Timestamp", timestamp);
    Logger.recordOutput("PoseEstimator/QuestNav/FrameSequence", (double) sequence);
    Logger.recordOutput("PoseEstimator/QuestNavUsed", true);
    Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", ""); // Clear
  }
  
  /**
   * 1) VELOCITY GATING: ONLY fuse when robot is nearly stationary
   * Prevents fighting PathPlanner during aggressive motion
   * 
   * THRESHOLDS:
   * - Linear: 0.12 m/s (~5 in/s) - allows brief pauses between path segments
   * - Angular: 0.3 rad/s (~17 deg/s) - allows settling after rotation
   * 
   * @return true if robot is slow enough for fusion, false to reject
   */
  private boolean velocityGatePass() {
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);
    
    boolean linearOk = linearSpeed <= MAX_LINEAR_SPEED_FOR_FUSION_MPS;
    boolean angularOk = angularSpeed <= MAX_ANGULAR_SPEED_FOR_FUSION_RAD_PER_SEC;
    
    Logger.recordOutput("PoseEstimator/QuestNav/VelocityGate/LinearSpeed", linearSpeed);
    Logger.recordOutput("PoseEstimator/QuestNav/VelocityGate/AngularSpeed", angularSpeed);
    Logger.recordOutput("PoseEstimator/QuestNav/VelocityGate/LinearOk", linearOk);
    Logger.recordOutput("PoseEstimator/QuestNav/VelocityGate/AngularOk", angularOk);
    Logger.recordOutput("PoseEstimator/QuestNav/VelocityGate/Passed", linearOk && angularOk);
    
    return linearOk && angularOk;
  }
  
  /**
   * 2) + 3) INNOVATION GATING with REACQUIRE mode
   * 
   * REACQUIRE MODE (when stopped):
   * - Position gate: 65cm (was 10cm)
   * - Rotation gate: 25° (was 5°)
   * - Allows rapid convergence when robot stops moving
   * 
   * MOVING MODE (normal):
   * - Tight gates with conservative motion-based expansion
   */
  private boolean innovationGatePass(Pose2d measurement, double age) {
    Pose2d predictedPose = swervePoseEstimator.getEstimatedPosition();
    
    double posError = measurement.getTranslation().getDistance(predictedPose.getTranslation());
    double rotError = Math.abs(measurement.getRotation().minus(predictedPose.getRotation()).getRadians());
    
    // Get current robot motion
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);
    
    // Determine if robot is stopped (for reacquire mode)
    boolean isStopped = (linearSpeed < REACQUIRE_STOPPED_LINEAR_THRESHOLD && 
                         angularSpeed < REACQUIRE_STOPPED_ANGULAR_THRESHOLD);
    
    double posGate, rotGate;
    
    if (isStopped) {
      // 3) REACQUIRE MODE: Wide gates for rapid convergence
      posGate = REACQUIRE_POS_GATE_METERS;
      rotGate = Math.toRadians(REACQUIRE_ROT_GATE_DEGREES);
      Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/Mode", "REACQUIRE");
    } else {
      // MOVING MODE: Tight gates with motion-based expansion
      posGate = INNOVATION_GATE_POS_BASE_METERS + INNOVATION_GATE_POS_PER_SPEED * linearSpeed * age;
      rotGate = Math.toRadians(INNOVATION_GATE_ROT_BASE_DEGREES) + INNOVATION_GATE_ROT_PER_OMEGA * angularSpeed * age;
      Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/Mode", "MOVING");
    }
    
    boolean passed = posError <= posGate && rotError <= rotGate;
    
    if (!passed) {
      Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", 
          String.format("Innovation gate: pos=%.3fm (gate=%.3fm), rot=%.1f° (gate=%.1f°), mode=%s", 
              posError, posGate, Math.toDegrees(rotError), Math.toDegrees(rotGate), 
              isStopped ? "REACQUIRE" : "MOVING"));
      Logger.recordOutput("PoseEstimator/QuestNav/RejectedPose", measurement);
    }
    
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/PosError", posError);
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/RotError", Math.toDegrees(rotError));
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/PosGate", posGate);
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/RotGate", Math.toDegrees(rotGate));
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/Passed", passed);
    Logger.recordOutput("PoseEstimator/QuestNav/InnovationGate/IsStopped", isStopped);
    
    return passed;
  }
  
  /**
   * 4) TRUST MODELING: Motion-dependent covariance (degrades trust, doesn't reject)
   * 
   * STRATEGY:
   * - STOPPED: Highest trust (2cm XY, 1.7° theta) - like stationary AprilTag
   * - MOVING: Degraded trust based on speed - allows gentle correction
   * - ROTATION: Reduce theta trust significantly during spin
   * - LATENCY: Degrade trust with age (older = less reliable)
   * 
   * INTENT: Motion reduces trust (larger std dev = less weight in Kalman filter)
   * but doesn't hard-reject. Allows slow convergence during motion if needed.
   * 
   * @param age Measurement latency in seconds
   * @return Standard deviation matrix [XY, XY, Theta] in meters and radians
   */
  private Matrix<N3, N1> getTrustForCurrentState(double age) {
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);
    
    // Start with moving trust (moderate)
    double xyStdDev = QUESTNAV_STD_DEVS[0]; // 8cm
    double thetaStdDev = QUESTNAV_STD_DEVS[2]; // 4°
    
    // STOPPED: Very high trust (like stationary AprilTag)
    if (linearSpeed < 0.05 && angularSpeed < 0.05) {
      xyStdDev = QUESTNAV_STD_DEVS_STOPPED[0]; // 2cm
      thetaStdDev = QUESTNAV_STD_DEVS_STOPPED[2]; // 1.7°
      Logger.recordOutput("PoseEstimator/QuestNav/TrustMode", "STOPPED");
    }
    // MOVING: Degrade trust based on speed (not rejection!)
    else {
      // 4) IMPROVED: Degrade XY trust with linear speed
      xyStdDev *= (1.0 + linearSpeed * 0.5); // +50% per m/s of linear motion
      
      // 4) IMPROVED: Heavily degrade theta trust during rotation
      if (angularSpeed > 0.3) {
        thetaStdDev *= (1.0 + angularSpeed * 2.0); // +200% per rad/s of rotation
      }
      
      Logger.recordOutput("PoseEstimator/QuestNav/TrustMode", "MOVING");
    }
    
    // Degrade trust with age (measurement latency)
    xyStdDev *= (1.0 + age * 2.0); // +100% per second of age
    thetaStdDev *= (1.0 + age * 3.0); // +200% per second of age
    
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/XY", xyStdDev);
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/Theta", Math.toDegrees(thetaStdDev));
    Logger.recordOutput("PoseEstimator/QuestNav/LinearSpeed", linearSpeed);
    Logger.recordOutput("PoseEstimator/QuestNav/AngularSpeed", angularSpeed);
    
    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }
  
  /**
   * Get standard deviations for initial alignment (MAXIMUM trust)
   * Used at auto start for one-time pose initialization
   * 
   * TRUST: 1cm XY, 1.1° theta (highest trust in entire system)
   * 
   * @return Standard deviation matrix for initial alignment
   */
  public Matrix<N3, N1> getInitialAlignmentStdDevs() {
    double xyTrust = QUESTNAV_STD_DEVS_INITIAL[0];     // 1cm
    double thetaTrust = QUESTNAV_STD_DEVS_INITIAL[2];  // 1.1°
    
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/XY_Initial", xyTrust);
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/Theta_Initial", thetaTrust);
    
    return VecBuilder.fill(xyTrust, xyTrust, thetaTrust);
  }
}