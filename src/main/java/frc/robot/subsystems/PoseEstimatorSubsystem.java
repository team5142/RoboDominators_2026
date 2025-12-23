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
  private double m_lastQuestNavFusionTime = 0.0; // NEW: QuestNav fusion timestamp

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
    
    // Disabled mode: Check readiness and validate alignment
    if (robotState.getMode() == RobotState.Mode.DISABLED) {
      initializer.updateReadiness();
      validator.periodicValidation(getEstimatedPose());
    }
    
    // Attempt initialization if needed
    if (!initializer.isInitialized()) {
      Pose2d initPose = initializer.attemptInitialization();
      if (initPose != null) {
        // NEW: Use VERY HIGH TRUST for initial QuestNav alignment
        // This ensures the pose estimator starts with accurate position from QuestNav
        Matrix<N3, N1> initialStdDevs = questNavFusion.getInitialAlignmentStdDevs();
        
        resetPose(initPose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
        
        // CRITICAL: Immediately add a QuestNav measurement with VERY HIGH TRUST
        java.util.Optional<Pose2d> questPose = questNavSubsystem.getRobotPose();
        if (questPose.isPresent()) {
          poseEstimator.addVisionMeasurement(
              questPose.get(),
              edu.wpi.first.wpilibj.Timer.getFPGATimestamp(),
              initialStdDevs); // VERY HIGH TRUST (1cm XY, 1 deg theta)
          
          Logger.recordOutput("PoseEstimator/QuestNav/InitialAlignmentApplied", true);
          SmartLogger.logConsole("QuestNav initial alignment applied with VERY HIGH TRUST");
        }
        
        // NEW: SCENARIO-BASED OPERATOR PERSPECTIVE (THIS WAS MISSING!)
        setOperatorPerspectiveBasedOnScenario(initPose, questPose.orElse(null));
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

  // NEW: Getter for QuestNav fusion time
  public double getTimeSinceLastQuestNavFusion() {
    return Timer.getFPGATimestamp() - m_lastQuestNavFusionTime;
  }

  // NEW: Called by QuestNavFusion when it processes frames
  public void notifyQuestNavFusionOccurred() {
    m_lastQuestNavFusionTime = Timer.getFPGATimestamp();
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
   * Force-accept the next QuestNav pose with very high trust.
   * Bypasses normal Kalman filtering - use only when robot is stopped and QuestNav is tracking!
   * 
   * @return true if QuestNav pose was force-accepted, false if QuestNav unavailable
   */
  public boolean forceAcceptQuestNavPose() {
    // Check if QuestNav is available and tracking
    if (!questNavSubsystem.isTracking()) {
      SmartLogger.logConsoleError("[ForceUpdate] QuestNav not tracking - cannot force update!");
      SmartLogger.logReplay("PoseEstimator/ForceUpdate/Failed", "Not tracking");
      return false;
    }
    
    // Get current QuestNav pose
    var questPose = questNavSubsystem.getRobotPose();
    if (!questPose.isPresent()) {
      SmartLogger.logConsoleError("[ForceUpdate] QuestNav has no pose available!");
      SmartLogger.logReplay("PoseEstimator/ForceUpdate/Failed", "No pose");
      return false;
    }
    
    Pose2d forcedPose = questPose.get();
    
    // Use VERY HIGH TRUST standard deviations (1cm XY, 1° theta)
    var veryHighTrust = VecBuilder.fill(0.01, 0.01, Math.toRadians(1.0));
    
    // Force-add measurement with high trust (essentially resets pose to QuestNav)
    poseEstimator.addVisionMeasurement(
        forcedPose,
        edu.wpi.first.wpilibj.Timer.getFPGATimestamp(),
        veryHighTrust);
    
    m_lastUpdateTime = Timer.getFPGATimestamp(); // Update timestamp
    
    // NEW: Notify fusion timestamp immediately (don't wait for processFrames())
    notifyQuestNavFusionOccurred();
    
    SmartLogger.logConsole("========== FORCED POSE UPDATE ==========");
    SmartLogger.logConsole("QuestNav pose: " + formatPose(forcedPose));
    SmartLogger.logConsole("Trust level: 1cm XY, 1° theta (VERY HIGH)");
    SmartLogger.logConsole("Kalman filter bypassed - pose locked to QuestNav");
    SmartLogger.logConsole("========================================");
    
    SmartLogger.logReplay("PoseEstimator/ForceUpdate/Success", true);
    SmartLogger.logReplay("PoseEstimator/ForceUpdate/Pose", forcedPose);
    SmartLogger.logReplay("PoseEstimator/ForceUpdate/Trust", "VERY HIGH (1cm, 1deg)");
    
    // NEW: Flush old frames so they don't get processed after force-update
    questNavSubsystem.getAllUnreadFrames(); // Discard old queue
    
    return true;
  }
    
  
  /**
   * Updates Limelight's robot orientation for MegaTag2 pose disambiguation.
   * Called every cycle to provide yaw + turn rate from odometry.
   * 
   * Sister team's critical insight: Feeding yaw+turnRate dramatically improves
   * AprilTag pose accuracy during rotation.
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