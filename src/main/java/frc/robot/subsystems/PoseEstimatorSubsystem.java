package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.pose.PoseInitializer;
import frc.robot.subsystems.pose.QuestNavFusion;
import frc.robot.subsystems.pose.PoseValidator;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LimelightHelpers; // Ensure this is the correct package for LimelightHelpers
import frc.robot.Constants; // Ensure the Constants class is imported
import frc.robot.util.SmartLogger;

/**
 * Pose Estimator Subsystem - Fuses odometry + QuestNav for accurate robot localization
 * 
 * Refactored into helper classes:
 * - PoseInitializer: Handles startup/initialization logic
 * - QuestNavFusion: Processes QuestNav SLAM measurements
 * - PoseValidator: Validates auto alignment
 * 
 * This class orchestrates the helpers and owns the SwerveDrivePoseEstimator
 */
public class PoseEstimatorSubsystem extends SubsystemBase {
  
  // Core pose estimation
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  
  // Subsystems
  private final RobotState robotState;
  private final DriveSubsystem driveSubsystem;
  private final GyroSubsystem gyroSubsystem;
  private final QuestNavSubsystem questNavSubsystem;
  private TagVisionSubsystem tagVisionSubsystem;
  
  // Helper classes (single responsibility)
  private final PoseInitializer initializer;
  private final QuestNavFusion questNavFusion;
  private final PoseValidator validator;
  
  // State tracking
  private boolean hasEverBeenEnabled = false;
  private RobotState.Mode lastMode = RobotState.Mode.DISABLED;
  private Pose2d lastPose = new Pose2d();
  
  // Logging optimization
  private int logCounter = 0;
  private static final int LOG_SKIP_CYCLES = 4;

  private final Field2d field = new Field2d();

  private double m_lastUpdateTime = 0.0; // Generic measurement timestamp
  private double m_lastQuestNavFusionTime = 0.0; // When fusion occurred (FPGA "now")
  private double m_lastQuestNavMeasurementTimestamp = 0.0; // Actual Quest measurement timestamp used

  // Rate limiting for ForceUpdate warnings
  private double lastForceUpdateWarningTime = 0.0;
  private static final double FORCE_UPDATE_WARNING_INTERVAL = 0.5; // 500ms between warnings

  public PoseEstimatorSubsystem(
      DriveSubsystem driveSubsystem,
      RobotState robotState,
      GyroSubsystem gyroSubsystem,
      QuestNavSubsystem questNavSubsystem) {
    
    this.driveSubsystem = driveSubsystem;
    this.robotState = robotState;
    this.gyroSubsystem = gyroSubsystem;
    this.questNavSubsystem = questNavSubsystem;
    this.kinematics = driveSubsystem.getKinematics();

    // Create pose estimator
    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        driveSubsystem.getGyroRotation(),
        driveSubsystem.getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(ODOMETRY_STD_DEVS[0], ODOMETRY_STD_DEVS[1], ODOMETRY_STD_DEVS[2]),
        VecBuilder.fill(LIMELIGHT_MULTI_TAG_STD_DEVS[0], LIMELIGHT_MULTI_TAG_STD_DEVS[1], LIMELIGHT_MULTI_TAG_STD_DEVS[2]));

    // Create helper classes - NOTE: questNavFusion needs to be created BEFORE initializer
    this.questNavFusion = new QuestNavFusion(
        questNavSubsystem, 
        driveSubsystem, 
        poseEstimator,  // SwerveDrivePoseEstimator
        this);          // PoseEstimatorSubsystem (THIS!)
    this.initializer = new PoseInitializer(questNavSubsystem, questNavFusion); // CHANGED: Pass questNavFusion
    this.validator = new PoseValidator();
    
    // NEW: Publish field to SmartDashboard/Elastic
    SmartDashboard.putData("Field", field);
    
    SmartLogger.logConsole("PoseEstimatorSubsystem initialized (refactored with helpers)");
  }
  
  public void setTagVisionSubsystem(TagVisionSubsystem tagVisionSubsystem) {
    this.tagVisionSubsystem = tagVisionSubsystem;
  }

  public void setAutoChooser(SendableChooser<Command> autoChooser) {
    initializer.setAutoChooser(autoChooser);
    validator.setAutoChooser(autoChooser);
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, int tagCount) {
    addVisionMeasurement(visionPose, timestampSeconds, tagCount, "unknown");
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, int tagCount, String cameraName) {
    // LIMELIGHT ONLY - PhotonVision disabled for QuestNav-only testing
    
    // Reject PhotonVision measurements
    if (!cameraName.equals(LL_FRONT_NAME)) {
      Logger.recordOutput("PoseEstimator/PhotonVisionDisabled", true);
      Logger.recordOutput("PoseEstimator/VisionMeasurementIgnored", visionPose);
      Logger.recordOutput("PoseEstimator/VisionCameraIgnored", cameraName);
      return;
    }
    
    // Accept Limelight measurements with proper trust based on tag count
    Matrix<N3, N1> stdDevs;
    
    if (tagCount >= MIN_TAG_COUNT_FOR_MULTI) {
      // Multi-tag: High trust (Limelight factory calibrated)
      stdDevs = VecBuilder.fill(
          LIMELIGHT_MULTI_TAG_STD_DEVS[0],
          LIMELIGHT_MULTI_TAG_STD_DEVS[1],
          LIMELIGHT_MULTI_TAG_STD_DEVS[2]);
      
      Logger.recordOutput("PoseEstimator/VisionType", "Limelight_MultiTag");
    } else {
      // Single-tag: Lower trust
      stdDevs = VecBuilder.fill(
          LIMELIGHT_SINGLE_TAG_STD_DEVS[0],
          LIMELIGHT_SINGLE_TAG_STD_DEVS[1],
          LIMELIGHT_SINGLE_TAG_STD_DEVS[2]);
      
      Logger.recordOutput("PoseEstimator/VisionType", "Limelight_SingleTag");
    }
    
    // Add vision measurement with appropriate trust
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
    m_lastUpdateTime = Timer.getFPGATimestamp(); // Update timestamp whenever we add a measurement
    
    // Logging
    Logger.recordOutput("PoseEstimator/LimelightEnabled", true);
    Logger.recordOutput("PoseEstimator/PhotonVisionEnabled", false);
    Logger.recordOutput("PoseEstimator/VisionMeasurement", visionPose);
    Logger.recordOutput("PoseEstimator/VisionTagCount", tagCount);
    Logger.recordOutput("PoseEstimator/VisionCamera", cameraName);
    Logger.recordOutput("PoseEstimator/VisionStdDevXY", stdDevs.get(0, 0));
    Logger.recordOutput("PoseEstimator/VisionStdDevTheta", stdDevs.get(2, 0));
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
    questNavSubsystem.initialize(pose);
    
    if (!initializer.isInitialized()) {
      initializer.setInitState(PoseInitializer.InitializationState.VISION_INITIALIZED);
      Logger.recordOutput("PoseEstimator/InitializedManually", true);
    }
    
    Logger.recordOutput("PoseEstimator/PoseReset", pose);
    Logger.recordOutput("PoseEstimator/QuestNavSyncedOnReset", true);
  }

  public PoseInitializer.InitializationState getInitializationState() {
    return initializer.getInitState();
  }

  public boolean isInitialized() {
    return initializer.isInitialized();
  }

  @Override
  public void periodic() {
    if (robotState.isSysIdMode()) {
      Logger.recordOutput("PoseEstimator/SysIdMode", true);
      return;
    }
    
    logCounter++;
    
    // Update odometry (always runs)
    poseEstimator.update(driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    
    // DISABLED: Limelight orientation updates (uncalibrated)
    // updateLimelightOrientation();
    
    // Disabled mode: Check readiness and validate alignment
    if (robotState.getMode() == RobotState.Mode.DISABLED) {
      initializer.updateReadiness();
      validator.periodicValidation(getEstimatedPose());
    }
    
    // Attempt initialization if needed
    if (!initializer.isInitialized()) {
      Pose2d initPose = initializer.attemptInitialization();
      if (initPose != null) {
        resetPose(initPose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
        
        // Use CONSUME-ONCE semantics for initial QuestNav alignment
        // This prevents re-using the same frame multiple times
        java.util.Optional<QuestNavSubsystem.QuestMeasurement> questMeas = 
            questNavSubsystem.consumeLatestMeasurement();
        
        if (questMeas.isPresent()) {
          double measurementTime = questMeas.get().timestampFPGA;
          long sequence = questMeas.get().sequence;
          double age = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - measurementTime;
          
          // Only use if measurement is fresh (< 500ms old)
          if (age >= 0 && age < 0.5) {
            Matrix<N3, N1> initialStdDevs = questNavFusion.getInitialAlignmentStdDevs();
            
            // Add with RECEIVE-ALIGNED timestamp
            poseEstimator.addVisionMeasurement(questMeas.get().pose, measurementTime, initialStdDevs);
            
            // Track measurement timestamp
            m_lastQuestNavMeasurementTimestamp = measurementTime;
            m_lastQuestNavFusionTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            
            Logger.recordOutput("PoseEstimator/QuestNav/InitialAlignmentApplied", true);
            Logger.recordOutput("PoseEstimator/QuestNav/InitialMeasurementAge", age);
            Logger.recordOutput("PoseEstimator/QuestNav/InitialConsumedSequence", (double) sequence);
            Logger.recordOutput("PoseEstimator/QuestNav/InitialMeasurementTimestamp", measurementTime);
            SmartLogger.logConsole("QuestNav initial alignment applied with VERY HIGH TRUST (seq=" + 
                sequence + ", age=" + String.format("%.3fs)", age));
          } else {
            Logger.recordOutput("PoseEstimator/QuestNav/InitialAlignmentRejected", 
                "Stale: age=" + age);
            SmartLogger.logConsoleError("QuestNav initial alignment rejected: measurement too old (" + 
                String.format("%.3fs)", age));
          }
        } else {
          // No unconsumed measurement available
          Logger.recordOutput("PoseEstimator/QuestNav/InitialAlignmentSkipped", 
              "No unconsumed measurement");
          SmartLogger.logConsole("QuestNav initial alignment skipped: no unconsumed measurement available");
        }
        
        // Set operator perspective based on scenario
        // Use getRobotPose() for display (doesn't consume)
        setOperatorPerspectiveBasedOnScenario(initPose, questNavSubsystem.getRobotPose().orElse(null));
      }
    }
    
    // Process QuestNav measurements (if initialized)
    if (initializer.isInitialized()) {
      questNavFusion.processFrames();
    }
    
    // Update robot state
    Pose2d currentPose = getEstimatedPose();
    robotState.setRobotPose(currentPose);
    
    // ===== FIELD VISUALIZATION =====
    
    // 1. Robot's estimated pose (fused odometry + QuestNav)
    field.setRobotPose(currentPose);
    
    // 2. QuestNav pose (green robot - raw SLAM estimate)
    questNavSubsystem.getRobotPose().ifPresent(questPose -> {
      field.getObject("QuestNav").setPose(questPose);
    });
    
    // 3. Target pose (if navigating)
    if (robotState.getNavigationPhase() != RobotState.NavigationPhase.NONE) {
      // You'd get this from your SmartDriveToPosition command
      // For now, just show it exists
      field.getObject("Target").setPoses(); // Clear when not navigating
    }
    
    // 4. Trajectory preview (optional - shows PathPlanner path)
    // This requires PathPlanner integration - we can add later
    
    // Logging
    double poseChange = currentPose.getTranslation().getDistance(lastPose.getTranslation());
    lastPose = currentPose;
    
    Logger.recordOutput("PoseEstimator/EstimatedPose", currentPose);
    Logger.recordOutput("PoseEstimator/InitState", initializer.getInitState().toString());
    
    if (logCounter % (LOG_SKIP_CYCLES + 1) == 0) {
      Logger.recordOutput("PoseEstimator/PoseChangeMeters", poseChange);
      Logger.recordOutput("PoseEstimator/WaitingForVisionTime", initializer.getWaitTime());
    }
    
    // Handle mode changes
    RobotState.Mode currentMode = robotState.getMode();
    if (currentMode != lastMode) {
      onModeChange(lastMode, currentMode);
      lastMode = currentMode;
    }
  }
  
  // NEW: Get time since last pose update (for QuestNav settle)
  public double getTimeSinceLastUpdate() {
    return Timer.getFPGATimestamp() - m_lastUpdateTime;
  }

  // NEW: Getter for last Quest measurement timestamp (for debugging)
  public double getLastQuestNavMeasurementTimestamp() {
    return m_lastQuestNavMeasurementTimestamp;
  }

  // NEW: Getter for QuestNav fusion time
  public double getTimeSinceLastQuestNavFusion() {
    return Timer.getFPGATimestamp() - m_lastQuestNavFusionTime;
  }

  // UPDATED: Track measurement timestamp when fusion occurs
  public void notifyQuestNavFusionOccurred(double measurementTimestamp) {
    m_lastQuestNavFusionTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    m_lastQuestNavMeasurementTimestamp = measurementTimestamp;
    
    // Also update generic "last update" time so it reflects all fusion sources
    m_lastUpdateTime = m_lastQuestNavFusionTime;
    
    Logger.recordOutput("PoseEstimator/QuestNav/LastFusionTime", m_lastQuestNavFusionTime);
    Logger.recordOutput("PoseEstimator/QuestNav/LastMeasurementTimestamp", m_lastQuestNavMeasurementTimestamp);
    
    // Calculate and log how old the measurement was when fused
    double measurementAge = m_lastQuestNavFusionTime - m_lastQuestNavMeasurementTimestamp;
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAgeAtFusion", measurementAge);
  }
  
  private void onModeChange(RobotState.Mode from, RobotState.Mode to) {
    if (to == RobotState.Mode.ENABLED_AUTO) {
      SmartLogger.logConsole("=== AUTO ENABLED ===");
      hasEverBeenEnabled = true;
    }
    
    if (from == RobotState.Mode.ENABLED_AUTO && to == RobotState.Mode.ENABLED_TELEOP) {
      SmartLogger.logConsole("=== AUTO → TELEOP TRANSITION ===");
      SmartLogger.logConsole("KEEPING pose from auto - no reset!");
      
      Pose2d currentPose = getEstimatedPose();
      SmartLogger.logConsole("Current pose: " + formatPose(currentPose));
      SmartLogger.logConsole("Current heading: " + currentPose.getRotation().getDegrees() + "°");
      
      driveSubsystem.setOperatorPerspectiveForward(currentPose.getRotation());
      
      Logger.recordOutput("PoseEstimator/AutoToTeleopTransition", true);
      Logger.recordOutput("PoseEstimator/TransitionPose", currentPose);
    }
    
    if (to == RobotState.Mode.ENABLED_TELEOP && !hasEverBeenEnabled) {
      SmartLogger.logConsole("=== TELEOP ENABLED (First Enable) ===");
      hasEverBeenEnabled = true;
      
      if (!initializer.isInitialized()) {
        SmartLogger.logConsoleError("WARNING: Enabled in teleop without pose!");
        SmartLogger.logConsoleError("Robot needs QuestNav or manual pose set");
      }
    }
  }
  
  public void setInitializedViaVision() {
    if (!initializer.isInitialized()) {
      initializer.setInitState(PoseInitializer.InitializationState.VISION_INITIALIZED);
      Logger.recordOutput("PoseEstimator/InitializedViaMultiTagVision", true);
    }
  }
  
  private String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", 
        pose.getX(), 
        pose.getY(), 
        pose.getRotation().getDegrees());
  }
  
  /**
   * Set operator perspective based on initialization scenario
   */
  private void setOperatorPerspectiveBasedOnScenario(Pose2d initPose, Pose2d questNavPose) {
    boolean isAutoOrFMS = DriverStation.isAutonomousEnabled() || DriverStation.isFMSAttached();
    
    if (isAutoOrFMS) {
      // AUTO/FMS: Trust robot is facing downfield, let CTRE handle it
      SmartLogger.logConsole("=== OPERATOR PERSPECTIVE (AUTO/FMS) ===");
      SmartLogger.logConsole("Mode: Competition / Practice Auto");
      SmartLogger.logConsole("Action: Using alliance-based perspective (CTRE default)");
      
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        String allianceName = alliance.get() == Alliance.Red ? "Red" : "Blue";
        double expectedPerspective = alliance.get() == Alliance.Red ? 180.0 : 0.0;
        SmartLogger.logConsole("Alliance: " + allianceName);
        SmartLogger.logConsole("Expected downfield: " + expectedPerspective + " deg");
      }
      SmartLogger.logConsole("======================================");
      
      Logger.recordOutput("PoseEstimator/OperatorPerspective/Scenario", "Auto/FMS");
      
    } else {
      // TELEOP PRACTICE: Use QuestNav heading to calculate perspective
      SmartLogger.logConsole("=== OPERATOR PERSPECTIVE (TELEOP PRACTICE) ===");
      SmartLogger.logConsole("Mode: Teleop practice (random field position)");
      
      if (questNavPose != null) {
        setOperatorPerspectiveFromPose(questNavPose);
        Logger.recordOutput("PoseEstimator/OperatorPerspective/Scenario", "Teleop");
      } else {
        SmartLogger.logConsoleError("WARNING: QuestNav unavailable - using CTRE default");
        Logger.recordOutput("PoseEstimator/OperatorPerspective/Scenario", "Teleop (QuestNav unavailable)");
      }
    }
  }
  
  /**
   * Calculate operator perspective from QuestNav pose (Teleop scenario)
   * Snaps to 5° increments to prevent micro-drift
   */
  private void setOperatorPerspectiveFromPose(Pose2d robotPose) {
    var alliance = DriverStation.getAlliance();
    Rotation2d downfieldDirection = alliance.isPresent() && alliance.get() == Alliance.Red
        ? Rotation2d.fromDegrees(180)
        : Rotation2d.fromDegrees(0);
    
    
    Rotation2d robotHeading = robotPose.getRotation();
    Rotation2d operatorPerspective = downfieldDirection.minus(robotHeading);
    
    // SNAP to nearest 5° to prevent micro-drift
    double perspectiveDeg = operatorPerspective.getDegrees();
    double snappedDeg = Math.round(perspectiveDeg / 5.0) * 5.0;
    Rotation2d snappedPerspective = Rotation2d.fromDegrees(snappedDeg);
    
    driveSubsystem.setOperatorPerspectiveForward(snappedPerspective);
    
    SmartLogger.logConsole("Robot heading (QuestNav): " + robotHeading.getDegrees() + " deg");
    SmartLogger.logConsole("Downfield direction: " + downfieldDirection.getDegrees() + " deg");
    SmartLogger.logConsole("Calculated perspective: " + perspectiveDeg + " deg");
    SmartLogger.logConsole("Snapped perspective: " + snappedDeg + " deg (nearest 5 deg)");
    SmartLogger.logConsole("============================================");
    
    Logger.recordOutput("PoseEstimator/RobotHeading", robotHeading.getDegrees());
    Logger.recordOutput("PoseEstimator/DownfieldDirection", downfieldDirection.getDegrees());
    Logger.recordOutput("PoseEstimator/OperatorPerspective/Calculated", perspectiveDeg);
    Logger.recordOutput("PoseEstimator/OperatorPerspective/Snapped", snappedDeg);
    Logger.recordOutput("PoseEstimator/OperatorPerspective/Applied", true);
  }
  
  /**
   * 4) FIXED: Force-accept using peek + acknowledge (no frame burning)
   * 
   * Uses PEEK semantics so rejected measurements are still available for fusion.
   * Rate-limits console warnings to prevent spam.
   */
  public boolean forceAcceptQuestNavPose() {
    if (!questNavSubsystem.isTracking()) {
      double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      if (currentTime - lastForceUpdateWarningTime > FORCE_UPDATE_WARNING_INTERVAL) {
        SmartLogger.logConsoleError("[ForceUpdate] QuestNav not tracking - cannot force update!");
        lastForceUpdateWarningTime = currentTime;
      }
      return false;
    }
    
    // PEEK measurement (non-consuming)
    var questMeas = questNavSubsystem.peekLatestMeasurement();
    
    if (!questMeas.isPresent()) {
      double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      if (currentTime - lastForceUpdateWarningTime > FORCE_UPDATE_WARNING_INTERVAL) {
        SmartLogger.logConsoleError("[ForceUpdate] No unconsumed QuestNav measurement available!");
        lastForceUpdateWarningTime = currentTime;
      }
      return false;
    }
    
    Pose2d forcedPose = questMeas.get().pose;
    double timestamp = questMeas.get().timestampFPGA;
    long sequence = questMeas.get().sequence;
    
    // Reject stale measurements
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double measurementAge = currentTime - timestamp;
    
    if (measurementAge < 0 || measurementAge > 0.25) {
      SmartLogger.logConsoleError("[ForceUpdate] Rejected stale measurement (age: " + 
          String.format("%.3fs)", measurementAge));
      Logger.recordOutput("PoseEstimator/ForceAccept/Rejected", "Stale: age=" + measurementAge);
      return false; // Do NOT acknowledge - preserve for fusion
    }
    
    // ACKNOWLEDGE only after accepting
    if (!questNavSubsystem.acknowledgeMeasurement(sequence)) {
      SmartLogger.logConsoleError("[ForceUpdate] Failed to acknowledge measurement (already consumed)");
      return false;
    }
    
    // Use VERY HIGH TRUST standard deviations (1cm XY, 1° theta)
    var veryHighTrust = VecBuilder.fill(0.01, 0.01, Math.toRadians(1.0));
    
    // Force-add measurement
    poseEstimator.addVisionMeasurement(forcedPose, timestamp, veryHighTrust);
    
    m_lastUpdateTime = currentTime;
    m_lastQuestNavMeasurementTimestamp = timestamp;
    m_lastQuestNavFusionTime = currentTime;
    
    SmartLogger.logConsole("========== FORCED POSE UPDATE ==========");
    SmartLogger.logConsole("QuestNav pose: " + formatPose(forcedPose));
    SmartLogger.logConsole("Timestamp (FPGA receive): " + String.format("%.3f", timestamp));
    SmartLogger.logConsole("Measurement age: " + String.format("%.3fs", measurementAge));
    SmartLogger.logConsole("Sequence: " + sequence);
    SmartLogger.logConsole("Trust level: 1cm XY, 1° theta (VERY HIGH)");
    SmartLogger.logConsole("========================================");
    
    Logger.recordOutput("PoseEstimator/ForceAccept/Success", true);
    Logger.recordOutput("PoseEstimator/ForceAccept/MeasurementAge", measurementAge);
    Logger.recordOutput("PoseEstimator/ForceAccept/Sequence", (double) sequence);
    
    return true;
  }
    
  
  /**
   * DISABLED: Updates Limelight's robot orientation for MegaTag2 pose disambiguation.
   * Re-enable after Limelight mount is calibrated.
   * 
   * Called every cycle to provide yaw + turn rate from odometry.
   */
  private void updateLimelightOrientation() {
    Pose2d currentPose = getEstimatedPose();
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    
    double yawDegrees = currentPose.getRotation().getDegrees();
    double turnRateDegPerSec = Math.toDegrees(speeds.omegaRadiansPerSecond);
    
    // Feed to Limelight for MegaTag2 (yaw, yawRate, pitch, pitchRate, roll, rollRate)
    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.LL_FRONT_NAME,
        yawDegrees,
        turnRateDegPerSec,
        0.0, 0.0,  // pitch, pitchRate (unused for swerve)
        0.0, 0.0); // roll, rollRate (unused for swerve)
    
    // Log for debugging
    Logger.recordOutput("Limelight/YawDegrees", yawDegrees);
    Logger.recordOutput("Limelight/TurnRateDegPerSec", turnRateDegPerSec);
  }
}