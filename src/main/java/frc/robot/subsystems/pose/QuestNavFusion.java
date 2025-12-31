package frc.robot.subsystems.pose;

import static frc.robot.Constants.QuestNav.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.SmartLogger;
import org.littletonrobotics.junction.Logger;

// QuestNav fusion with mode-aware validation: COMP_SEED (anchored to known start) vs SHOP_RESUME (stability check)
public class QuestNavFusion {

  public enum QuestHealthState {
    UNVALIDATED,  // Need validation after seed/tracking-loss
    HEALTHY,      // Agreeing with odom
    DEGRADED,     // Persistent divergence, reduced trust
    UNHEALTHY     // Teleports or severe disagreement
  }

  private QuestHealthState healthState = QuestHealthState.UNVALIDATED;
  private int consecutiveDivergence = 0;
  private int consecutiveTeleports = 0;
  private double lastAcceptedQuestTimestamp = -1.0;
  private Pose2d lastAcceptedQuestPose = null;
  private double lastResetTime = -1.0;

  // Validation state
  private Constants.QuestNav.InitMode validationMode = null;
  private Pose2d expectedSeedPose = null;
  private boolean validationInProgress = false;
  private double validationStartTime = -1.0;
  private Pose2d validationSeedPose = null;
  private int validationPassStreak = 0;

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

  // Main fusion loop: velocity gate → peek → age → teleport → divergence → validation → fuse
  public void processFrames() {
    // Cache current time for this cycle
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    
    // Set clean defaults for this cycle (prevent stale values)
    Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAccepted", false);
    Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", "");
    Logger.recordOutput("PoseEstimator/QuestNav/TeleportRejected", false);
    Logger.recordOutput("PoseEstimator/QuestNav/HealthState", healthState.toString());
    
    if (!velocityGatePass()) {
      reject("Velocity gate failed");
      return;
    }

    // Peek measurement (non-consuming until we acknowledge)
    // Rejected frames remain available until a newer one arrives
    java.util.Optional<QuestNavSubsystem.QuestMeasurement> measurement =
        questNavSubsystem.peekLatestMeasurement();

    if (!measurement.isPresent()) {
      reject("No unconsumed measurement");
      return;
    }

    QuestNavSubsystem.QuestMeasurement meas = measurement.get();
    Pose2d questPose = meas.pose;
    double timestamp = meas.receiveTimestampFPGA; // FPGA receive time (NOT capture time)
    long sequence = meas.sequence;

    double measurementAge = currentTime - timestamp;

    if (measurementAge > 0.5 || measurementAge < 0) {
      reject("Stale/invalid age: " + String.format("%.3fs", measurementAge));
      Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAge", measurementAge);
      return;
    }

    double dt = (lastAcceptedQuestTimestamp > 0) ? (timestamp - lastAcceptedQuestTimestamp) : 0.0;
    boolean inGracePeriod = (lastResetTime > 0) && ((currentTime - lastResetTime) < POST_RESET_GRACE_SEC);

    Logger.recordOutput("PoseEstimator/QuestNav/DT", dt);
    Logger.recordOutput("PoseEstimator/QuestNav/InGracePeriod", inGracePeriod);

    if (!inGracePeriod && !teleportGatePass(questPose, dt)) {
      return;
    }

    if (!inGracePeriod) {
      updateHealthState(questPose, dt);
    }

    if (healthState == QuestHealthState.UNVALIDATED) {
      if (!validationInProgress) {
        startValidation(questPose, timestamp);
      }

      boolean validationPassed = checkValidation(questPose);

      if (validationPassed) {
        healthState = QuestHealthState.HEALTHY;
        validationInProgress = false;
        Logger.recordOutput("PoseEstimator/QuestNav/ValidationPassed", true);
        logStateTransition("UNVALIDATED -> HEALTHY", "Validation passed");
      } else if ((currentTime - validationStartTime) > VALIDATION_TIMEOUT_SEC) {
        healthState = QuestHealthState.UNHEALTHY;
        validationInProgress = false;
        Logger.recordOutput("PoseEstimator/QuestNav/ValidationFailed", "Timeout");
        reject("Validation failed (timeout)");
        logStateTransition("UNVALIDATED -> UNHEALTHY", "Validation timeout");
        return;
      }
    }

    if (healthState == QuestHealthState.UNHEALTHY) {
      reject("Health state: UNHEALTHY");
      return;
    }

    // Only acknowledge (consume) after passing all gates
    if (!questNavSubsystem.acknowledgeMeasurement(sequence)) {
      reject("Acknowledge failed (already consumed)");
      return;
    }

    Matrix<N3, N1> stdDevs = getHealthAwareTrust(measurementAge);
    swervePoseEstimator.addVisionMeasurement(questPose, timestamp, stdDevs);

    lastAcceptedQuestTimestamp = timestamp;
    lastAcceptedQuestPose = questPose;
    poseEstimatorSubsystem.notifyQuestNavFusionOccurred(timestamp);

    Logger.recordOutput("PoseEstimator/QuestNav/LatestPose", questPose);
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAge", measurementAge);
    Logger.recordOutput("PoseEstimator/QuestNav/Timestamp", timestamp);
    Logger.recordOutput("PoseEstimator/QuestNav/FrameSequence", (double) sequence);
    Logger.recordOutput("PoseEstimator/QuestNavUsed", true);
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAccepted", true);
  }

  // Centralized rejection helper (consistent telemetry)
  private void reject(String reason) {
    Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAccepted", false);
    Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", reason);
  }

  private boolean teleportGatePass(Pose2d questPose, double dt) {
    if (lastAcceptedQuestPose == null) {
      Logger.recordOutput("PoseEstimator/QuestNav/TeleportCheck", "SKIPPED (first)");
      return true;
    }

    double translationError = questPose.getTranslation().getDistance(lastAcceptedQuestPose.getTranslation());
    double rotationError = Math.abs(questPose.getRotation().minus(lastAcceptedQuestPose.getRotation()).getRadians());

    Logger.recordOutput("PoseEstimator/QuestNav/TranslationError", translationError);
    Logger.recordOutput("PoseEstimator/QuestNav/RotationError", Math.toDegrees(rotationError));

    if (translationError > TELEPORT_TRANSLATION_METERS || rotationError > TELEPORT_ROTATION_RADIANS) {
      consecutiveTeleports++;
      if (consecutiveTeleports > 3) {
        healthState = QuestHealthState.UNHEALTHY;
        logStateTransition("-> UNHEALTHY", "Teleport threshold exceeded (3 consecutive)");
      }

      Logger.recordOutput("PoseEstimator/QuestNav/TeleportRejected", true);
      String reason = String.format("TELEPORT: trans=%.2fm (max=%.2fm), rot=%.1f° (max=%.1f°)",
          translationError, TELEPORT_TRANSLATION_METERS,
          Math.toDegrees(rotationError), Math.toDegrees(TELEPORT_ROTATION_RADIANS));
      reject(reason);
      Logger.recordOutput("PoseEstimator/QuestNav/ConsecutiveTeleports", consecutiveTeleports);
      return false;
    }

    if (dt > MIN_DT_FOR_IMPLIED_VELOCITY) {
      double impliedSpeed = translationError / dt;
      double impliedOmega = rotationError / dt;

      double maxSpeed = MAX_PHYSICAL_SPEED_MPS * PHYSICAL_PLAUSIBILITY_MARGIN;
      double maxOmega = MAX_PHYSICAL_OMEGA_RAD_PER_SEC * PHYSICAL_PLAUSIBILITY_MARGIN;

      Logger.recordOutput("PoseEstimator/QuestNav/ImpliedSpeed", impliedSpeed);
      Logger.recordOutput("PoseEstimator/QuestNav/ImpliedOmega", impliedOmega);

      if (impliedSpeed > maxSpeed || impliedOmega > maxOmega) {
        consecutiveTeleports++;
        if (consecutiveTeleports > 3) {
          healthState = QuestHealthState.UNHEALTHY;
          logStateTransition("-> UNHEALTHY", "Implied velocity threshold exceeded");
        }

        Logger.recordOutput("PoseEstimator/QuestNav/TeleportRejected", true);
        String reason = String.format("IMPLIED VELOCITY: speed=%.2fm/s (max=%.2f), omega=%.1frad/s (max=%.1f)",
            impliedSpeed, maxSpeed, impliedOmega, maxOmega);
        reject(reason);
        Logger.recordOutput("PoseEstimator/QuestNav/ConsecutiveTeleports", consecutiveTeleports);
        return false;
      }
    } else {
      Logger.recordOutput("PoseEstimator/QuestNav/ImpliedVelocityCheck", "SKIPPED (dt too small)");
    }

    consecutiveTeleports = 0;
    Logger.recordOutput("PoseEstimator/QuestNav/TeleportCheck", "PASSED");
    return true;
  }

  private void updateHealthState(Pose2d questPose, double dt) {
    if (lastAcceptedQuestPose == null || dt < MIN_DT_FOR_IMPLIED_VELOCITY) {
      Logger.recordOutput("PoseEstimator/QuestNav/DivergenceCheck", "SKIPPED (insufficient data)");
      return;
    }

    double questDeltaXY = questPose.getTranslation().getDistance(lastAcceptedQuestPose.getTranslation());
    double questDeltaTheta = Math.abs(questPose.getRotation().minus(lastAcceptedQuestPose.getRotation()).getRadians());

    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double odomDeltaXY = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) * dt;
    double odomDeltaTheta = Math.abs(speeds.omegaRadiansPerSecond) * dt;

    double questMetric = questDeltaXY + questDeltaTheta * DIVERGENCE_ANGULAR_WEIGHT;
    double odomMetric = odomDeltaXY + odomDeltaTheta * DIVERGENCE_ANGULAR_WEIGHT;
    double divergenceMetric = Math.abs(questMetric - odomMetric);

    Logger.recordOutput("PoseEstimator/QuestNav/QuestDeltaXY", questDeltaXY);
    Logger.recordOutput("PoseEstimator/QuestNav/OdomDeltaXY", odomDeltaXY);
    Logger.recordOutput("PoseEstimator/QuestNav/DivergenceMetric", divergenceMetric);

    if (divergenceMetric > DIVERGENCE_THRESHOLD_METERS) {
      consecutiveDivergence++;

      if (consecutiveDivergence >= DIVERGENCE_PATIENCE_CYCLES) {
        if (healthState == QuestHealthState.HEALTHY) {
          healthState = QuestHealthState.DEGRADED;
          Logger.recordOutput("PoseEstimator/QuestNav/HealthTransition", "HEALTHY -> DEGRADED");
          logStateTransition("HEALTHY -> DEGRADED", "Persistent divergence from odometry");
        }
      }
    } else {
      if (consecutiveDivergence > 0) {
        consecutiveDivergence--;
      }

      if (healthState == QuestHealthState.DEGRADED && consecutiveDivergence == 0) {
        healthState = QuestHealthState.HEALTHY;
        Logger.recordOutput("PoseEstimator/QuestNav/HealthTransition", "DEGRADED -> HEALTHY");
        logStateTransition("DEGRADED -> HEALTHY", "Divergence resolved");
      }
    }

    Logger.recordOutput("PoseEstimator/QuestNav/ConsecutiveDivergence", consecutiveDivergence);
  }

  private void startValidation(Pose2d initialQuestPose, double timestamp) {
    validationInProgress = true;
    validationStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    
    // Choose validation reference based on mode
    if (validationMode == Constants.QuestNav.InitMode.COMP_SEED && expectedSeedPose != null) {
      validationSeedPose = expectedSeedPose;
      logStateTransition("Starting validation", "COMP_SEED mode: validating against " + formatPose(expectedSeedPose));
    } else {
      validationSeedPose = initialQuestPose;
      String modeStr = (validationMode != null) ? validationMode.toString() : "UNKNOWN->SHOP_DEFAULT";
      Logger.recordOutput("PoseEstimator/QuestNav/ValidationMode", modeStr);
      logStateTransition("Starting validation", "SHOP_RESUME mode: stability check");
    }
    
    validationPassStreak = 0;

    Logger.recordOutput("PoseEstimator/QuestNav/ValidationStarted", true);
    Logger.recordOutput("PoseEstimator/QuestNav/ValidationSeedPose", validationSeedPose);
  }

  private boolean checkValidation(Pose2d questPose) {
    if (validationSeedPose == null) return false;

    double error = questPose.getTranslation().getDistance(validationSeedPose.getTranslation());
    
    // Choose tolerance based on mode (null safety: default to SHOP)
    double tolerance = (validationMode == Constants.QuestNav.InitMode.COMP_SEED) 
        ? COMP_VALIDATION_TOLERANCE_METERS 
        : SHOP_STABILITY_TOLERANCE_METERS;

    Logger.recordOutput("PoseEstimator/QuestNav/ValidationError", error);
    Logger.recordOutput("PoseEstimator/QuestNav/ValidationStreak", validationPassStreak);
    Logger.recordOutput("PoseEstimator/QuestNav/ValidationTolerance", tolerance);

    if (error < tolerance) {
      validationPassStreak++;
      if (validationPassStreak >= VALIDATION_REQUIRED_STREAK) {
        return true;
      }
    } else {
      validationPassStreak = 0;
    }

    return false;
  }

  private Matrix<N3, N1> getHealthAwareTrust(double age) {
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);

    double baseXY, baseTheta;
    if (linearSpeed < 0.05 && angularSpeed < 0.05) {
      baseXY = QUESTNAV_STD_DEVS_STOPPED[0];
      baseTheta = QUESTNAV_STD_DEVS_STOPPED[2];
      Logger.recordOutput("PoseEstimator/QuestNav/TrustMode", "STOPPED");
    } else {
      baseXY = QUESTNAV_STD_DEVS[0];
      baseTheta = QUESTNAV_STD_DEVS[2];

      if (linearSpeed > 1.0 || angularSpeed > 1.0) {
        baseXY *= MOVING_TRUST_DEGRADATION_FACTOR;
        baseTheta *= MOVING_TRUST_DEGRADATION_FACTOR;
        Logger.recordOutput("PoseEstimator/QuestNav/TrustMode", "MOVING_FAST");
      } else {
        Logger.recordOutput("PoseEstimator/QuestNav/TrustMode", "MOVING_SLOW");
      }
    }

    double healthFactor = 1.0;
    switch (healthState) {
      case HEALTHY:
        healthFactor = 1.0;
        break;
      case DEGRADED:
        healthFactor = DEGRADED_TRUST_FACTOR;
        Logger.recordOutput("PoseEstimator/QuestNav/TrustDegraded", true);
        break;
      case UNHEALTHY:
        healthFactor = UNHEALTHY_TRUST_FACTOR;
        Logger.recordOutput("PoseEstimator/QuestNav/TrustUnhealthy", true);
        break;
      case UNVALIDATED:
        baseXY = QUESTNAV_STD_DEVS_INITIAL[0];
        baseTheta = QUESTNAV_STD_DEVS_INITIAL[2];
        break;
    }

    double finalXY = baseXY * healthFactor * (1.0 + age * 2.0);
    double finalTheta = baseTheta * healthFactor * (1.0 + age * 3.0);

    Logger.recordOutput("PoseEstimator/QuestNav/StdDevXY", finalXY);
    Logger.recordOutput("PoseEstimator/QuestNav/StdDevTheta", Math.toDegrees(finalTheta));
    Logger.recordOutput("PoseEstimator/QuestNav/HealthFactor", healthFactor);

    return VecBuilder.fill(finalXY, finalXY, finalTheta);
  }

  public void notifyEstimatorReset() {
    lastResetTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    lastAcceptedQuestTimestamp = -1.0;
    lastAcceptedQuestPose = null;
    consecutiveDivergence = 0;
    consecutiveTeleports = 0;

    Logger.recordOutput("PoseEstimator/QuestNav/ResetGraceStarted", true);
  }

  public void notifyTrackingLost() {
    healthState = QuestHealthState.UNVALIDATED;
    validationInProgress = false;

    Logger.recordOutput("PoseEstimator/QuestNav/TrackingLost", true);
    Logger.recordOutput("PoseEstimator/QuestNav/HealthState", "UNVALIDATED");
    logStateTransition("-> UNVALIDATED", "Tracking lost");
  }

  public void notifyTrackingRegained() {
    healthState = QuestHealthState.UNVALIDATED;
    validationInProgress = false;
    consecutiveDivergence = 0;
    consecutiveTeleports = 0;

    Logger.recordOutput("PoseEstimator/QuestNav/TrackingRegained", true);
    Logger.recordOutput("PoseEstimator/QuestNav/HealthState", "UNVALIDATED");
    logStateTransition("-> UNVALIDATED", "Tracking regained - revalidation required");
  }

  public void setExpectedSeedPose(Pose2d pose) {
    this.expectedSeedPose = pose;
    Logger.recordOutput("PoseEstimator/QuestNav/ExpectedSeedPose", pose);
  }

  public void setValidationMode(Constants.QuestNav.InitMode mode) {
    this.validationMode = mode;
    Logger.recordOutput("PoseEstimator/QuestNav/ValidationMode", mode.toString());
  }

  public QuestHealthState getHealthState() {
    return healthState;
  }

  private boolean velocityGatePass() {
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);

    boolean linearOk = linearSpeed <= MAX_LINEAR_SPEED_FOR_FUSION_MPS;
    boolean angularOk = angularSpeed <= MAX_ANGULAR_SPEED_FOR_FUSION_RAD_PER_SEC;

    Logger.recordOutput("PoseEstimator/QuestNav/VelocityGate/LinearSpeed", linearSpeed);
    Logger.recordOutput("PoseEstimator/QuestNav/VelocityGate/AngularSpeed", angularSpeed);
    Logger.recordOutput("PoseEstimator/QuestNav/VelocityGate/Passed", linearOk && angularOk);

    return linearOk && angularOk;
  }

  public Matrix<N3, N1> getInitialAlignmentStdDevs() {
    double xyTrust = QUESTNAV_STD_DEVS_INITIAL[0];
    double thetaTrust = QUESTNAV_STD_DEVS_INITIAL[2];

    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/XY_Initial", xyTrust);
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/Theta_Initial", thetaTrust);

    return VecBuilder.fill(xyTrust, xyTrust, thetaTrust);
  }

  public boolean forceAcceptMeasurement() {
    if (!questNavSubsystem.isTracking()) {
      SmartLogger.logConsoleError("[ForceAccept] Quest not tracking");
      return false;
    }

    var questMeas = questNavSubsystem.peekLatestMeasurement();
    if (!questMeas.isPresent()) {
      SmartLogger.logConsoleError("[ForceAccept] No unconsumed measurement");
      return false;
    }

    Pose2d forcedPose = questMeas.get().pose;
    double timestamp = questMeas.get().receiveTimestampFPGA; // FPGA receive time
    long sequence = questMeas.get().sequence;

    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double measurementAge = currentTime - timestamp;

    if (measurementAge < 0 || measurementAge > 0.25) {
      SmartLogger.logConsoleError("[ForceAccept] Stale measurement (age: " + 
          String.format("%.3fs)", measurementAge));
      return false;
    }

    if (!questNavSubsystem.acknowledgeMeasurement(sequence)) {
      SmartLogger.logConsoleError("[ForceAccept] Acknowledge failed");
      return false;
    }

    var veryHighTrust = VecBuilder.fill(0.01, 0.01, Math.toRadians(1.0));
    swervePoseEstimator.addVisionMeasurement(forcedPose, timestamp, veryHighTrust);

    lastAcceptedQuestTimestamp = timestamp;
    lastAcceptedQuestPose = forcedPose;
    poseEstimatorSubsystem.notifyQuestNavFusionOccurred(timestamp);

    SmartLogger.logConsole("[ForceAccept] Success: " + formatPose(forcedPose));
    Logger.recordOutput("PoseEstimator/ForceAccept/Success", true);

    return true;
  }

  // Console logging helper: Only log state transitions (not steady-state)
  private void logStateTransition(String transition, String reason) {
    SmartLogger.logConsole("[QuestNav Health] " + transition + " - " + reason);
  }

  private String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", 
        pose.getX(), 
        pose.getY(), 
        pose.getRotation().getDegrees());
  }
}