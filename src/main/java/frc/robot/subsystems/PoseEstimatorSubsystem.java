package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Vision.*;
import static frc.robot.Constants.StartingPositions.*;
import static frc.robot.Constants.QuestNav.*;
import static frc.robot.Constants.Auto.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Kalman filter-based pose estimator - fuses wheel odometry, vision, and QuestNav SLAM
// Uses LAZY INITIALIZATION - collects data while disabled, initializes when enabled
// FIXED: Now checks TagVisionSubsystem for current detection status instead of circular init check
public class PoseEstimatorSubsystem extends SubsystemBase {
  public enum InitializationState {
    WAITING_FOR_VISION,
    VISION_INITIALIZED,
    FALLBACK_USED
  }

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final RobotState robotState;
  private final DriveSubsystem driveSubsystem;
  private final GyroSubsystem gyroSubsystem;
  private TagVisionSubsystem tagVisionSubsystem; // NEW: Set after construction to avoid circular dependency

  private InitializationState initState = InitializationState.WAITING_FOR_VISION;
  private final Timer visionWaitTimer = new Timer();

  private Pose2d lastPose = new Pose2d();
  private int visionUpdateCount = 0;

  private double lastQuestNavSyncTime = 0.0;
  private static final double QUESTNAV_SYNC_INTERVAL_SECONDS = 1.0;

  private double lastPoseValidationTime = 0.0;
  private static final double POSE_VALIDATION_INTERVAL_SECONDS = 10.0;

  private static final boolean VISION_UPDATES_ENABLED = true;

  private static final double LIMELIGHT_QUALITY = 1.0;
  private static final double RR_TAG_PV_QUALITY = 1.5;
  private static final double RL_TAG_PV_QUALITY = 2.0;

  private SendableChooser<Command> autoChooser;

  public PoseEstimatorSubsystem(
      DriveSubsystem driveSubsystem,
      RobotState robotState,
      GyroSubsystem gyroSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.robotState = robotState;
    this.gyroSubsystem = gyroSubsystem;
    this.kinematics = driveSubsystem.getKinematics();

    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        driveSubsystem.getGyroRotation(),
        driveSubsystem.getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(ODOMETRY_STD_DEVS[0], ODOMETRY_STD_DEVS[1], ODOMETRY_STD_DEVS[2]),
        VecBuilder.fill(VISION_STD_DEVS_SINGLE_TAG[0], VISION_STD_DEVS_SINGLE_TAG[1], VISION_STD_DEVS_SINGLE_TAG[2]));

    visionWaitTimer.start();
  }
  
  // NEW: Set TagVisionSubsystem reference (called by RobotContainer after both subsystems created)
  public void setTagVisionSubsystem(TagVisionSubsystem tagVisionSubsystem) {
    this.tagVisionSubsystem = tagVisionSubsystem;
  }

  public void setAutoChooser(SendableChooser<Command> autoChooser) {
    this.autoChooser = autoChooser;
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, int tagCount) {
    addVisionMeasurement(visionPose, timestampSeconds, tagCount, "unknown");
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, int tagCount, String cameraName) {
    if (!VISION_UPDATES_ENABLED) {
      Logger.recordOutput("PoseEstimator/VisionUpdatesDisabled", true);
      Logger.recordOutput("PoseEstimator/VisionMeasurementIgnored", visionPose);
      Logger.recordOutput("PoseEstimator/VisionTagCountIgnored", tagCount);
      return;
    }
    
    if (initState == InitializationState.WAITING_FOR_VISION) {
      initState = InitializationState.VISION_INITIALIZED;
      Logger.recordOutput("PoseEstimator/InitializedViaVision", true);
      System.out.println("=== POSE INITIALIZED VIA VISION ===");
      System.out.println("First vision pose: " + visionPose);
      System.out.println("Tags seen: " + tagCount);
      System.out.println("Time to initialize: " + visionWaitTimer.get() + "s");
      
      Pose3d pose3d = new Pose3d(
          visionPose.getX(),
          visionPose.getY(),
          0.0,
          new Rotation3d(0.0, 0.0, visionPose.getRotation().getRadians()));
      gyroSubsystem.setQuestNavPose(pose3d);
      lastQuestNavSyncTime = Timer.getFPGATimestamp();
    }

    Pose2d currentOdometry = poseEstimator.getEstimatedPosition();
    double correctionDistance = currentOdometry.getTranslation()
        .getDistance(visionPose.getTranslation());
    double correctionAngle = Math.abs(
        currentOdometry.getRotation().minus(visionPose.getRotation()).getDegrees());
    
    Logger.recordOutput("PoseEstimator/VisionCorrectionDistance", correctionDistance);
    Logger.recordOutput("PoseEstimator/VisionCorrectionAngle", correctionAngle);
    
    double currentTime = Timer.getFPGATimestamp();
    boolean shouldSync = (correctionDistance > 0.05 || correctionAngle > 2.0) &&
                         (currentTime - lastQuestNavSyncTime >= QUESTNAV_SYNC_INTERVAL_SECONDS);
    
    if (shouldSync) {
      Logger.recordOutput("PoseEstimator/SignificantVisionCorrection", true);
      
      Pose3d pose3d = new Pose3d(
          visionPose.getX(),
          visionPose.getY(),
          0.0,
          new Rotation3d(0.0, 0.0, visionPose.getRotation().getRadians()));
      gyroSubsystem.setQuestNavPose(pose3d);
      lastQuestNavSyncTime = currentTime;
    }

    double[] baseStdDevs = tagCount >= MIN_TAG_COUNT_FOR_MULTI 
        ? VISION_STD_DEVS_MULTI_TAG
        : VISION_STD_DEVS_SINGLE_TAG;

    double qualityMultiplier = getCameraQualityMultiplier(cameraName);
    double[] adjustedStdDevs = new double[] {
        baseStdDevs[0] * qualityMultiplier,
        baseStdDevs[1] * qualityMultiplier,
        baseStdDevs[2] * qualityMultiplier
    };

    poseEstimator.addVisionMeasurement(
        visionPose,
        timestampSeconds,
        VecBuilder.fill(adjustedStdDevs[0], adjustedStdDevs[1], adjustedStdDevs[2]));

    visionUpdateCount++;
    
    Logger.recordOutput("PoseEstimator/VisionUpdate", visionPose);
    Logger.recordOutput("PoseEstimator/VisionTagCount", tagCount);
    Logger.recordOutput("PoseEstimator/VisionCamera", cameraName);
    Logger.recordOutput("PoseEstimator/VisionQualityMultiplier", qualityMultiplier);
    Logger.recordOutput("PoseEstimator/VisionStdDevX", adjustedStdDevs[0]);
    Logger.recordOutput("PoseEstimator/TotalVisionUpdates", visionUpdateCount);
  }

  private double getCameraQualityMultiplier(String cameraName) {
    switch (cameraName) {
      case "limelight-front":
        return LIMELIGHT_QUALITY;
      case "RRTagPV":
        return RR_TAG_PV_QUALITY;
      case "RLTagPV":
        return RL_TAG_PV_QUALITY;
      default:
        return 2.0;
    }
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
    
    Pose3d pose3d = new Pose3d(
        pose.getX(),
        pose.getY(),
        0.0,
        new Rotation3d(0.0, 0.0, pose.getRotation().getRadians()));
    gyroSubsystem.setQuestNavPose(pose3d);
    
    if (initState == InitializationState.WAITING_FOR_VISION) {
      initState = InitializationState.VISION_INITIALIZED;
      Logger.recordOutput("PoseEstimator/InitializedManually", true);
    }
    
    Logger.recordOutput("PoseEstimator/PoseReset", pose);
    Logger.recordOutput("PoseEstimator/QuestNavSyncedOnReset", true);
  }

  public InitializationState getInitializationState() {
    return initState;
  }

  public boolean isInitialized() {
    return initState != InitializationState.WAITING_FOR_VISION;
  }

  @Override
  public void periodic() {
    if (robotState.isSysIdMode()) {
      Logger.recordOutput("PoseEstimator/SysIdMode", true);
      return;
    }
    
    poseEstimator.update(driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    
    if (robotState.getMode() == RobotState.Mode.DISABLED) {
      updateInitializationReadiness();
      periodicPoseValidation();
      return;
    }
    
    if (initState == InitializationState.WAITING_FOR_VISION) {
      attemptInitialization();
    }
    
    if (VISION_UPDATES_ENABLED && initState != InitializationState.WAITING_FOR_VISION) {
      if (robotState.getMode() != RobotState.Mode.AUTO) {
        processAllQuestNavFrames();
      } else {
        Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      }
    }
    
    Pose2d currentPose = getEstimatedPose();
    robotState.setRobotPose(currentPose);
    
    double poseChange = currentPose.getTranslation().getDistance(lastPose.getTranslation());
    Logger.recordOutput("PoseEstimator/PoseChangeMeters", poseChange);
    lastPose = currentPose;
    
    Logger.recordOutput("PoseEstimator/EstimatedPose", currentPose);
    Logger.recordOutput("PoseEstimator/InitState", initState.toString());
    Logger.recordOutput("PoseEstimator/WaitingForVisionTime", 
        initState == InitializationState.WAITING_FOR_VISION ? visionWaitTimer.get() : 0.0);
    Logger.recordOutput("PoseEstimator/TotalVisionUpdates", visionUpdateCount);
  }
  
  private void updateInitializationReadiness() {
    boolean hasMultiTagVision = checkForMultiTagVision();
    boolean hasSingleTagVision = checkForSingleTagVision();
    Pose2d questNavPose = gyroSubsystem.getQuestNavPose2d();
    boolean hasQuestNavPose = (questNavPose != null);
    
    if (hasMultiTagVision) {
      SmartDashboard.putString("Pose/InitStatus", "Multi-tag vision ready");
      SmartDashboard.putBoolean("Pose/ReadyToEnable", true);
    } else if (hasSingleTagVision) {
      SmartDashboard.putString("Pose/InitStatus", "Single-tag only");
      SmartDashboard.putBoolean("Pose/ReadyToEnable", true);
    } else if (hasQuestNavPose) {
      SmartDashboard.putString("Pose/InitStatus", "QuestNav only (might be stale)");
      SmartDashboard.putBoolean("Pose/ReadyToEnable", true);
    } else {
      SmartDashboard.putString("Pose/InitStatus", "MANUAL RESET REQUIRED");
      SmartDashboard.putBoolean("Pose/ReadyToEnable", false);
    }
    
    SmartDashboard.putBoolean("Vision/MultiTagReady", hasMultiTagVision);
    SmartDashboard.putBoolean("Vision/SingleTagReady", hasSingleTagVision);
    SmartDashboard.putBoolean("QuestNav/Ready", hasQuestNavPose);
    
    Logger.recordOutput("PoseEstimator/Readiness/MultiTag", hasMultiTagVision);
    Logger.recordOutput("PoseEstimator/Readiness/SingleTag", hasSingleTagVision);
    Logger.recordOutput("PoseEstimator/Readiness/QuestNav", hasQuestNavPose);
  }
  
  private void attemptInitialization() {
    if (checkForMultiTagVision()) {
      System.out.println("=== INITIALIZED FROM MULTI-TAG VISION ===");
      SmartDashboard.putString("Pose/InitMethod", "Multi-tag vision");
      return;
    }
    
    if (checkForSingleTagVision()) {
      System.out.println("=== INITIALIZED FROM SINGLE-TAG VISION ===");
      System.out.println("WARNING: Single tag - verify position!");
      SmartDashboard.putString("Pose/InitMethod", "Single-tag vision");
      return;
    }
    
    Pose2d questNavPose = gyroSubsystem.getQuestNavPose2d();
    if (questNavPose != null) {
      resetPose(questNavPose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
      initState = InitializationState.VISION_INITIALIZED;
      
      System.out.println("=== INITIALIZED FROM QUESTNAV ===");
      System.err.println("WARNING: QuestNav might be stale from previous run!");
      System.out.println("Pose: " + questNavPose);
      
      SmartDashboard.putString("Pose/InitMethod", "QuestNav (MIGHT BE STALE!)");
      Logger.recordOutput("PoseEstimator/InitializedViaQuestNav", true);
      return;
    }
    
    System.err.println("=== NO POSE AVAILABLE ===");
    System.err.println("Press START button to set position manually");
    System.err.println("Or move robot to see AprilTags");
    
    SmartDashboard.putString("Pose/InitMethod", "BLOCKED - MANUAL RESET REQUIRED");
    DriverStation.reportError("NO POSE - Press START to set position", false);
  }
  
  // FIXED: Check TagVisionSubsystem's CURRENT detection status
  private boolean checkForMultiTagVision() {
    if (tagVisionSubsystem == null) return false;
    return tagVisionSubsystem.hasMultiTagDetection();
  }
  
  // FIXED: Check TagVisionSubsystem's CURRENT detection status
  private boolean checkForSingleTagVision() {
    if (tagVisionSubsystem == null) return false;
    return tagVisionSubsystem.hasSingleTagDetection();
  }
  
  private void processAllQuestNavFrames() {
    PoseFrame[] frames = gyroSubsystem.getAllQuestNavFrames();
    
    if (frames == null || frames.length == 0) {
      Logger.recordOutput("PoseEstimator/QuestNavUsed", false);
      return;
    }
    
    int framesProcessed = 0;
    int framesRejected = 0;
    
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    double currentSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    
    for (PoseFrame frame : frames) {
      if (frame == null) continue;
      
      Pose3d questPose3d = frame.questPose3d();
      if (questPose3d == null) {
        Logger.recordOutput("PoseEstimator/QuestNav/RejectionReason", "Null Pose3d");
        framesRejected++;
        continue;
      }
      
      Pose2d questPose2d = new Pose2d(
        questPose3d.getX(),
        questPose3d.getY(),
        questPose3d.getRotation().toRotation2d());
      
      double timestamp = frame.dataTimestamp();
      
      if (!gateQuestNavMeasurement(questPose2d, timestamp, currentSpeed)) {
        framesRejected++;
        continue;
      }
      
      edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> stdDevs = 
        getSpeedAdaptiveQuestNavStdDevs(currentSpeed);
      
      poseEstimator.addVisionMeasurement(questPose2d, timestamp, stdDevs);
      framesProcessed++;
    }
    
    Logger.recordOutput("PoseEstimator/QuestNavFramesProcessed", framesProcessed);
    Logger.recordOutput("PoseEstimator/QuestNavFramesRejected", framesRejected);
    Logger.recordOutput("PoseEstimator/QuestNavUsed", framesProcessed > 0);
    Logger.recordOutput("PoseEstimator/CurrentSpeed", currentSpeed);
  }
  
  private boolean gateQuestNavMeasurement(Pose2d measurement, double timestamp, double currentSpeed) {
    Pose2d predictedPose = poseEstimator.getEstimatedPosition();
    
    double posError = measurement.getTranslation().getDistance(predictedPose.getTranslation());
    double rotError = Math.abs(measurement.getRotation().minus(predictedPose.getRotation()).getRadians());
    
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
  
  private edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> 
      getSpeedAdaptiveQuestNavStdDevs(double speedMPS) {
    
    double baseTrust = 0.02;
    double speedPenalty = 0.05 * speedMPS;
    double xyTrust = baseTrust + speedPenalty;
    double thetaTrust = baseTrust * 1.5 + (speedPenalty * 0.5);
    
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/XY", xyTrust);
    Logger.recordOutput("PoseEstimator/QuestNav/StdDev/Theta", thetaTrust);
    
    return edu.wpi.first.math.VecBuilder.fill(xyTrust, xyTrust, thetaTrust);
  }
  
  public void setInitializedViaVision() {
    if (initState == InitializationState.WAITING_FOR_VISION) {
      initState = InitializationState.VISION_INITIALIZED;
      Logger.recordOutput("PoseEstimator/InitializedViaMultiTagVision", true);
    }
  }

  private void periodicPoseValidation() {
    double currentTime = Timer.getFPGATimestamp();
    
    if (currentTime - lastPoseValidationTime < POSE_VALIDATION_INTERVAL_SECONDS) {
      return;
    }
    lastPoseValidationTime = currentTime;
    
    Pose2d currentPose = getEstimatedPose();
    Pose2d expectedPose = getExpectedAutoStartPose();
    
    if (expectedPose == null) {
      System.out.println("========== POSE VALIDATION (No Auto) ==========");
      System.out.println("Current pose: " + formatPose(currentPose));
      System.out.println("Auto: NONE SELECTED");
      System.out.println("Ready for manual placement");
      System.out.println("==============================================");
      
      SmartDashboard.putString("Pose/Validation", "No auto selected");
      SmartDashboard.putBoolean("Pose/AutoAligned", false);
      return;
    }
    
    double posError = currentPose.getTranslation().getDistance(expectedPose.getTranslation());
    double rotError = Math.abs(currentPose.getRotation().minus(expectedPose.getRotation()).getDegrees());
    
    boolean withinTolerance = 
        posError < STARTING_POSE_TOLERANCE_METERS && 
        rotError < STARTING_POSE_TOLERANCE_DEGREES;
    
    System.out.println("========== POSE VALIDATION ==========");
    System.out.println("Current:  " + formatPose(currentPose));
    System.out.println("Expected: " + formatPose(expectedPose));
    System.out.println("Position error: " + String.format("%.2fm", posError) + 
                       " (tolerance: " + STARTING_POSE_TOLERANCE_METERS + "m)");
    System.out.println("Rotation error: " + String.format("%.1f°", rotError) + 
                       " (tolerance: " + STARTING_POSE_TOLERANCE_DEGREES + "°)");
    
    if (withinTolerance) {
      System.out.println("Status: ALIGNED - Ready for auto");
    } else {
      System.err.println("Status: NOT ALIGNED - Adjust robot position!");
      if (posError >= STARTING_POSE_TOLERANCE_METERS) {
        System.err.println("  → Move robot " + String.format("%.2fm", posError) + " closer");
      }
      if (rotError >= STARTING_POSE_TOLERANCE_DEGREES) {
        System.err.println("  → Rotate robot " + String.format("%.1f°", rotError) + " more");
      }
    }
    System.out.println("=====================================");
    
    SmartDashboard.putString("Pose/Validation", withinTolerance ? "✓ ALIGNED" : "✗ NOT ALIGNED");
    SmartDashboard.putBoolean("Pose/AutoAligned", withinTolerance);
    SmartDashboard.putNumber("Pose/PosError", posError);
    SmartDashboard.putNumber("Pose/RotError", rotError);
    SmartDashboard.putString("Pose/Current", formatPose(currentPose));
    SmartDashboard.putString("Pose/Expected", formatPose(expectedPose));
    
    Logger.recordOutput("PoseValidation/WithinTolerance", withinTolerance);
    Logger.recordOutput("PoseValidation/PosError", posError);
    Logger.recordOutput("PoseValidation/RotError", rotError);
    Logger.recordOutput("PoseValidation/CurrentPose", currentPose);
    Logger.recordOutput("PoseValidation/ExpectedPose", expectedPose);
  }
  
  private Pose2d getExpectedAutoStartPose() {
    if (autoChooser == null) return null;
    
    try {
      Command selectedAuto = autoChooser.getSelected();
      if (selectedAuto == null) return null;
      
      String autoName = selectedAuto.getName();
      
      switch (autoName.toLowerCase()) {
        case "leftside1piece":
        case "leftside3piece":
          return new Pose2d(7.20, 0.45, Rotation2d.fromDegrees(180.0));
        case "rightside1piece":
          return new Pose2d(7.20, 5.50, Rotation2d.fromDegrees(180.0));
        default:
          Logger.recordOutput("PoseValidation/UnknownAuto", autoName);
          return null;
      }
    } catch (Exception e) {
      Logger.recordOutput("PoseValidation/GetPoseError", e.getMessage());
      return null;
    }
  }
  
  private String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", 
        pose.getX(), 
        pose.getY(), 
        pose.getRotation().getDegrees());
  }
}