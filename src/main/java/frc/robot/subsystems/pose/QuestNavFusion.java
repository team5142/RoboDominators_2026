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

// Decides whether to fuse each QuestNav frame into the pose estimator.
// Uses a health state machine to adjust trust level based on how well Quest agrees with odometry.
public class QuestNavFusion {

  // Health states control how much we trust Quest measurements.
  // UNVALIDATED: just seeded or tracking regained — must pass streak check before fusing
  // HEALTHY:     agreeing with odometry — full trust
  // DEGRADED:    persistently diverging — reduced trust (higher std devs)
  // UNHEALTHY:   teleport detected or validation failed — rejected, watching for recovery
  public enum QuestHealthState {
    UNVALIDATED,
    HEALTHY,
    DEGRADED,
    UNHEALTHY
  }

  private QuestHealthState healthState = QuestHealthState.UNVALIDATED;
  private QuestHealthState lastLoggedHealthState = QuestHealthState.UNVALIDATED;
  private int consecutiveDivergence = 0;
  private int consecutiveTeleports = 0;
  private double lastAcceptedQuestTimestamp = -1.0;
  private Pose2d lastAcceptedQuestPose = null;
  private double lastResetTime = -1.0;

  // Validation state (used during UNVALIDATED to confirm Quest is stable before trusting it)
  private Constants.QuestNav.InitMode validationMode = null;
  private Pose2d expectedSeedPose = null;
  private boolean validationInProgress = false;
  private double validationStartTime = -1.0;
  private Pose2d validationSeedPose = null;
  private int validationPassStreak = 0;

  // After a manual seed, skip gates on the very next post-seed frame so estimator snaps immediately
  private boolean acceptNextValidFrame = false;
  private double manualSeedTimestamp = -1.0;
  private static final double POST_SEED_EPSILON_SEC = 0.020;

  // UNHEALTHY recovery: collect Quest samples while stopped; snap estimator if Quest is self-stable
  private final java.util.ArrayDeque<Pose2d> recoveryBuffer = new java.util.ArrayDeque<>();

  // Throttle periodic telemetry to ~5Hz (every 10 loops at 50Hz)
  private int logCounter = 0;

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

  // Called every periodic loop. Pulls the latest Quest frame and decides whether to fuse it.
  public void processFrames() {
    logCounter++;
    logPeriodicState();

    if (questNavSubsystem.isFusionPaused()) {
      reject("Fusion paused");
      return;
    }

    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    if (!velocityGatePass()) {
      reject("Velocity gate failed");
      return;
    }

    var measurement = questNavSubsystem.peekLatestMeasurement();
    if (!measurement.isPresent()) {
      reject("No unconsumed measurement");
      return;
    }

    QuestNavSubsystem.QuestMeasurement meas = measurement.get();
    Pose2d questPose = meas.pose;
    double timestamp = meas.measurementTimestamp;
    long frameCount = meas.frameCount;
    double measurementAge = currentTime - timestamp;

    // Reject frames that are too old or from the future (clock skew)
    if (measurementAge > 0.5 || measurementAge < 0) {
      reject("Stale/invalid age: " + String.format("%.3fs", measurementAge));
      return;
    }

    // Drop frames that arrived before the seed — they carry pre-seed pose data
    if (acceptNextValidFrame && (timestamp + POST_SEED_EPSILON_SEC) < manualSeedTimestamp) {
      reject("Pre-seed frame dropped");
      return;
    }

    double dt = (lastAcceptedQuestTimestamp > 0) ? (timestamp - lastAcceptedQuestTimestamp) : 0.0;
    boolean inGracePeriod = (lastResetTime > 0) && ((currentTime - lastResetTime) < POST_RESET_GRACE_SEC);

    // First post-seed frame: bypass all gates and hard-reset the estimator so it snaps immediately.
    // addVisionMeasurement alone is too gentle — it won't move the pose if odometry hasn't moved.
    if (acceptNextValidFrame && (timestamp + POST_SEED_EPSILON_SEC) >= manualSeedTimestamp) {
      if (!questNavSubsystem.acknowledgeMeasurement(frameCount)) {
        reject("Acknowledge failed (already consumed)");
        acceptNextValidFrame = false;
        return;
      }
      Matrix<N3, N1> highTrust = VecBuilder.fill(0.02, 0.02, Math.toRadians(2.0));
      swervePoseEstimator.addVisionMeasurement(questPose, timestamp, highTrust);
      swervePoseEstimator.resetPosition(
          driveSubsystem.getGyroRotation(),
          driveSubsystem.getModulePositions(),
          questPose);
      acceptNextValidFrame = false;
      healthState = QuestHealthState.HEALTHY;
      validationInProgress = false;
      lastAcceptedQuestTimestamp = timestamp;
      lastAcceptedQuestPose = questPose;
      poseEstimatorSubsystem.notifyQuestNavFusionOccurred(timestamp);
      SmartLogger.logConsole("[QuestNav] Post-seed frame accepted: " + SmartLogger.formatPose(questPose), "QuestNav");
      logAccepted(questPose, timestamp, frameCount, dt, measurementAge);
      return;
    }

    // Teleport gate: reject if Quest jumped an impossible distance since last accepted frame
    if (!inGracePeriod && !teleportGatePass(questPose, dt)) return;

    // Divergence tracking: compare Quest motion to odometry motion each frame
    if (!inGracePeriod && !acceptNextValidFrame) updateHealthState(questPose, dt);

    // Validation: require a streak of consistent Quest frames before trusting after seed/regain
    if (healthState == QuestHealthState.UNVALIDATED) {
      if (!validationInProgress) startValidation(questPose, timestamp);
      if (checkValidation(questPose)) {
        healthState = QuestHealthState.HEALTHY;
        validationInProgress = false;
        logStateTransition("UNVALIDATED -> HEALTHY", "Validation passed");
      } else if ((currentTime - validationStartTime) > VALIDATION_TIMEOUT_SEC) {
        healthState = QuestHealthState.UNHEALTHY;
        validationInProgress = false;
        reject("Validation timed out");
        logStateTransition("UNVALIDATED -> UNHEALTHY", "Validation timeout");
        return;
      }
    }

    // UNHEALTHY: don't fuse, but watch for Quest to self-stabilize so we can recover
    if (healthState == QuestHealthState.UNHEALTHY) {
      checkUnhealthyRecovery(questPose);
      reject("UNHEALTHY");
      return;
    }

    if (!questNavSubsystem.acknowledgeMeasurement(frameCount)) {
      reject("Acknowledge failed (already consumed)");
      return;
    }

    // Fuse — std devs scale up when moving fast or health is degraded
    Matrix<N3, N1> stdDevs = getHealthAwareTrust(measurementAge);
    swervePoseEstimator.addVisionMeasurement(questPose, timestamp, stdDevs);
    lastAcceptedQuestTimestamp = timestamp;
    lastAcceptedQuestPose = questPose;
    poseEstimatorSubsystem.notifyQuestNavFusionOccurred(timestamp);
    logAccepted(questPose, timestamp, frameCount, dt, measurementAge);
  }

  // --- Gate methods (pure logic, no Logger calls) ---

  // Only fuse when robot is moving slowly — fast motion degrades Quest accuracy
  private boolean velocityGatePass() {
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double linear  = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double angular = Math.abs(speeds.omegaRadiansPerSecond);
    return linear <= MAX_LINEAR_SPEED_FOR_FUSION_MPS
        && angular <= MAX_ANGULAR_SPEED_FOR_FUSION_RAD_PER_SEC;
  }

  // Reject if Quest pose jumped farther than the robot physically could have moved
  private boolean teleportGatePass(Pose2d questPose, double dt) {
    if (isTilted()) return true; // ramp tilt causes false teleport readings
    if (TELEPORT_GATE_ONLY_WHEN_MOVING && !isMovingFast()) return true;

    Pose2d estimatedPose = poseEstimatorSubsystem.getEstimatedPose();
    double translationError = questPose.getTranslation().getDistance(estimatedPose.getTranslation());
    double rotationError = Math.abs(questPose.getRotation().minus(estimatedPose.getRotation()).getRadians());

    if (translationError > TELEPORT_TRANSLATION_METERS || rotationError > TELEPORT_ROTATION_RADIANS) {
      consecutiveTeleports++;
      if (consecutiveTeleports > 3 && healthState != QuestHealthState.UNHEALTHY) {
        healthState = QuestHealthState.UNHEALTHY;
        logStateTransition("-> UNHEALTHY", "Teleport gate exceeded 3 consecutive times");
      }
      reject(String.format("TELEPORT: trans=%.2fm, rot=%.1f deg", translationError, Math.toDegrees(rotationError)));
      return false;
    }

    // Also check implied velocity between consecutive Quest frames
    if (dt > MIN_DT_FOR_IMPLIED_VELOCITY && lastAcceptedQuestPose != null) {
      double impliedSpeed = questPose.getTranslation().getDistance(lastAcceptedQuestPose.getTranslation()) / dt;
      double impliedOmega = Math.abs(questPose.getRotation().minus(lastAcceptedQuestPose.getRotation()).getRadians()) / dt;
      double maxSpeed = MAX_PHYSICAL_SPEED_MPS * PHYSICAL_PLAUSIBILITY_MARGIN;
      double maxOmega = MAX_PHYSICAL_OMEGA_RAD_PER_SEC * PHYSICAL_PLAUSIBILITY_MARGIN;

      if (impliedSpeed > maxSpeed || impliedOmega > maxOmega) {
        consecutiveTeleports++;
        if (consecutiveTeleports > 3 && healthState != QuestHealthState.UNHEALTHY) {
          healthState = QuestHealthState.UNHEALTHY;
          logStateTransition("-> UNHEALTHY", "Implied velocity exceeded");
        }
        reject(String.format("IMPLIED VELOCITY: speed=%.2fm/s, omega=%.1frad/s", impliedSpeed, impliedOmega));
        return false;
      }
    }

    consecutiveTeleports = 0;
    return true;
  }

  // Compare how far Quest moved vs how far odometry moved in the same dt.
  // Persistent disagreement increments consecutiveDivergence and eventually degrades health.
  private void updateHealthState(Pose2d questPose, double dt) {
    if (lastAcceptedQuestPose == null || dt < MIN_DT_FOR_IMPLIED_VELOCITY) return;

    double questDeltaXY    = questPose.getTranslation().getDistance(lastAcceptedQuestPose.getTranslation());
    double questDeltaTheta = Math.abs(questPose.getRotation().minus(lastAcceptedQuestPose.getRotation()).getRadians());

    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double odomDeltaXY    = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) * dt;
    double odomDeltaTheta = Math.abs(speeds.omegaRadiansPerSecond) * dt;

    double divergence = Math.abs(
        (questDeltaXY + questDeltaTheta * DIVERGENCE_ANGULAR_WEIGHT)
        - (odomDeltaXY + odomDeltaTheta * DIVERGENCE_ANGULAR_WEIGHT));

    if (divergence > DIVERGENCE_THRESHOLD_METERS) {
      consecutiveDivergence++;
      if (consecutiveDivergence >= DIVERGENCE_PATIENCE_CYCLES && healthState == QuestHealthState.HEALTHY) {
        healthState = QuestHealthState.DEGRADED;
        logStateTransition("HEALTHY -> DEGRADED", "Persistent divergence");
      }
    } else {
      if (consecutiveDivergence > 0) consecutiveDivergence--;
      if (healthState == QuestHealthState.DEGRADED && consecutiveDivergence == 0) {
        healthState = QuestHealthState.HEALTHY;
        logStateTransition("DEGRADED -> HEALTHY", "Divergence resolved");
      }
    }
  }

  // Require N consecutive frames within tolerance of the seed pose before declaring HEALTHY
  private void startValidation(Pose2d initialQuestPose, double timestamp) {
    validationInProgress = true;
    validationStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    validationSeedPose = (validationMode == Constants.QuestNav.InitMode.COMP_SEED && expectedSeedPose != null)
        ? expectedSeedPose
        : initialQuestPose;
    validationPassStreak = 0;
    logStateTransition("Starting validation", validationMode + " mode");
  }

  private boolean checkValidation(Pose2d questPose) {
    if (validationSeedPose == null) return false;
    double error = questPose.getTranslation().getDistance(validationSeedPose.getTranslation());
    double tolerance = (validationMode == Constants.QuestNav.InitMode.COMP_SEED)
        ? COMP_VALIDATION_TOLERANCE_METERS
        : SHOP_STABILITY_TOLERANCE_METERS;
    if (error < tolerance) {
      validationPassStreak++;
      return validationPassStreak >= VALIDATION_REQUIRED_STREAK;
    }
    validationPassStreak = 0;
    return false;
  }

  // Scale up std devs (reduce trust) based on motion speed and health state.
  // Lower std devs = more trust. Higher std devs = the Kalman filter weighs this measurement less.
  private Matrix<N3, N1> getHealthAwareTrust(double age) {
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double linear  = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double angular = Math.abs(speeds.omegaRadiansPerSecond);

    double baseXY, baseTheta;
    if (healthState == QuestHealthState.UNVALIDATED) {
      baseXY    = QUESTNAV_STD_DEVS_INITIAL[0];
      baseTheta = QUESTNAV_STD_DEVS_INITIAL[2];
    } else if (linear < 0.05 && angular < 0.05) {
      baseXY    = QUESTNAV_STD_DEVS_STOPPED[0];
      baseTheta = QUESTNAV_STD_DEVS_STOPPED[2];
    } else {
      baseXY    = QUESTNAV_STD_DEVS[0] * ((linear > 1.0 || angular > 1.0) ? MOVING_TRUST_DEGRADATION_FACTOR : 1.0);
      baseTheta = QUESTNAV_STD_DEVS[2] * ((linear > 1.0 || angular > 1.0) ? MOVING_TRUST_DEGRADATION_FACTOR : 1.0);
    }

    double healthFactor = switch (healthState) {
      case HEALTHY     -> 1.0;
      case DEGRADED    -> DEGRADED_TRUST_FACTOR;
      case UNHEALTHY   -> UNHEALTHY_TRUST_FACTOR;
      case UNVALIDATED -> 1.0;
    };

    // Older measurements get slightly less trust (latency degrades accuracy)
    return VecBuilder.fill(
        baseXY    * healthFactor * (1.0 + age * 2.0),
        baseXY    * healthFactor * (1.0 + age * 2.0),
        baseTheta * healthFactor * (1.0 + age * 3.0));
  }

  // When UNHEALTHY and robot is stopped, collect Quest samples.
  // If Quest readings are self-consistent (low variance), odometry probably drifted — snap to Quest.
  private void checkUnhealthyRecovery(Pose2d questPose) {
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    boolean stopped = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < REACQUIRE_STOPPED_LINEAR_THRESHOLD
        && Math.abs(speeds.omegaRadiansPerSecond) < REACQUIRE_STOPPED_ANGULAR_THRESHOLD;

    if (!stopped) { recoveryBuffer.clear(); return; }

    recoveryBuffer.addLast(questPose);
    if (recoveryBuffer.size() > UNHEALTHY_RECOVERY_SAMPLES) recoveryBuffer.removeFirst();
    if (recoveryBuffer.size() < UNHEALTHY_RECOVERY_SAMPLES) return;

    // Compute max distance from centroid across all buffered samples
    double sumX = 0, sumY = 0;
    for (Pose2d p : recoveryBuffer) { sumX += p.getX(); sumY += p.getY(); }
    double cx = sumX / recoveryBuffer.size(), cy = sumY / recoveryBuffer.size();
    double maxDist = 0;
    for (Pose2d p : recoveryBuffer) maxDist = Math.max(maxDist, Math.hypot(p.getX() - cx, p.getY() - cy));

    if (maxDist < UNHEALTHY_RECOVERY_VARIANCE_METERS) {
      poseEstimatorSubsystem.resetPose(
          questPose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
      healthState = QuestHealthState.UNVALIDATED;
      consecutiveDivergence = 0;
      consecutiveTeleports = 0;
      recoveryBuffer.clear();
      SmartLogger.logConsole("[QuestNav Recovery] Snapped estimator to Quest: "
          + SmartLogger.formatPose(questPose), "QuestNav");
      logStateTransition("UNHEALTHY -> UNVALIDATED", "Quest stable recovery");
      Logger.recordOutput("PoseEstimator/QuestNav/Recovery/SnapPose", questPose);
    }
  }

  private boolean isMovingFast() {
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > MAX_LINEAR_SPEED_FOR_FUSION_MPS
        || Math.abs(speeds.omegaRadiansPerSecond) > MAX_ANGULAR_SPEED_FOR_FUSION_RAD_PER_SEC;
  }

  private boolean isTilted() {
    return Math.abs(driveSubsystem.getGyroPitchDegrees()) > TELEPORT_GATE_MAX_TILT_DEGREES
        || Math.abs(driveSubsystem.getGyroRollDegrees())  > TELEPORT_GATE_MAX_TILT_DEGREES;
  }

  // --- Public state notifications ---

  public void notifyEstimatorReset() {
    lastResetTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    lastAcceptedQuestTimestamp = -1.0;
    lastAcceptedQuestPose = null;
    consecutiveDivergence = 0;
    consecutiveTeleports = 0;
  }

  public void notifyTrackingLost() {
    healthState = QuestHealthState.UNVALIDATED;
    validationInProgress = false;
    logStateTransition("-> UNVALIDATED", "Tracking lost");
  }

  public void notifyTrackingRegained() {
    healthState = QuestHealthState.UNVALIDATED;
    validationInProgress = false;
    consecutiveDivergence = 0;
    consecutiveTeleports = 0;
    logStateTransition("-> UNVALIDATED", "Tracking regained — revalidation required");
  }

  public void setExpectedSeedPose(Pose2d pose) {
    this.expectedSeedPose = pose;
  }

  public void setValidationMode(Constants.QuestNav.InitMode mode) {
    this.validationMode = mode;
  }

  public QuestHealthState getHealthState() {
    return healthState;
  }

  // Called when a manual COMP_SEED happens — arms the post-seed frame latch
  public void onManualSeed(Pose2d seedPose) {
    double seedTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    lastAcceptedQuestPose = null;
    lastAcceptedQuestTimestamp = seedTime;
    healthState = QuestHealthState.UNVALIDATED;
    consecutiveDivergence = 0;
    consecutiveTeleports = 0;
    acceptNextValidFrame = true;
    manualSeedTimestamp = seedTime;
    validationInProgress = false;
    validationSeedPose = seedPose;
    SmartLogger.logConsole("[QuestNav] Manual seed received: " + SmartLogger.formatPose(seedPose), "QuestNav");
  }

  // Called on SHOP_RESUME — skip validation, grant a brief grace period to let odom catch up
  public void onShopResumeInit() {
    lastResetTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    healthState = QuestHealthState.HEALTHY;
    consecutiveDivergence = 0;
    consecutiveTeleports = 0;
    lastAcceptedQuestPose = null;
    lastAcceptedQuestTimestamp = -1.0;
    SmartLogger.logConsole("[QuestNav] SHOP_RESUME init — grace period started", "QuestNav");
  }

  // --- Telemetry helpers (all Logger.recordOutput calls live here, not in the algorithm above) ---

  // Logs periodic state at ~5Hz. Called at the top of processFrames every loop.
  private void logPeriodicState() {
    Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAccepted", false);
    if (logCounter % 10 != 0) return;
    Logger.recordOutput("PoseEstimator/QuestNav/HealthState", healthState.toString());
    Logger.recordOutput("PoseEstimator/QuestNav/ConsecutiveDivergence", consecutiveDivergence);
    Logger.recordOutput("PoseEstimator/QuestNav/ConsecutiveTeleports", consecutiveTeleports);
    Logger.recordOutput("PoseEstimator/QuestNav/ValidationStreak", validationPassStreak);
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    Logger.recordOutput("PoseEstimator/QuestNav/VelocityGate/LinearSpeed",
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    Logger.recordOutput("PoseEstimator/QuestNav/VelocityGate/AngularSpeed",
        Math.abs(speeds.omegaRadiansPerSecond));
  }

  // Logs once per accepted measurement.
  private void logAccepted(Pose2d questPose, double timestamp, long frameCount, double dt, double age) {
    Logger.recordOutput("PoseEstimator/QuestNavUsed", true);
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAccepted", true);
    Logger.recordOutput("PoseEstimator/QuestNav/LatestPose", questPose);
    Logger.recordOutput("PoseEstimator/QuestNav/Timestamp", timestamp);
    Logger.recordOutput("PoseEstimator/QuestNav/FrameCount", (double) frameCount);
    Logger.recordOutput("PoseEstimator/QuestNav/DT", dt);
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAge", age);
  }

  // Logs rejection reason so AKit always has a fresh value every loop
  private void reject(String reason) {
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAccepted", false);
    Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", reason);
  }

  // Only logs when the health state actually changes — avoids console spam
  private void logStateTransition(String transition, String reason) {
    if (healthState != lastLoggedHealthState) {
      SmartLogger.logConsole("[QuestNav Health] " + transition + " — " + reason, "QuestNav");
      lastLoggedHealthState = healthState;
    }
  }
}