package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.SmartLogger;
import edu.wpi.first.wpilibj.DriverStation;

// Fuses odometry + QuestNav - Two modes: COMP_SEED (match) vs SHOP_RESUME (practice)
public class PoseEstimatorSubsystem extends SubsystemBase {
  
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  
  private final RobotState robotState;
  private final DriveSubsystem driveSubsystem;
  private final QuestNavSubsystem questNavSubsystem;
  private TagVisionSubsystem tagVisionSubsystem;
  
  private final PoseInitializer initializer;
  private final QuestNavFusion questNavFusion;
  private final PoseValidator validator;
  
  private boolean hasEverBeenEnabled = false;
  private RobotState.Mode lastMode = RobotState.Mode.DISABLED;
  private Pose2d lastPose = new Pose2d();
  
  private int logCounter = 0;
  private static final int LOG_SKIP_CYCLES = 4;

  private final Field2d field = new Field2d();

  private double m_lastUpdateTime = 0.0;
  private double m_lastQuestNavFusionTime = 0.0;
  private double m_lastQuestNavMeasurementTimestamp = 0.0;

  private boolean lastTrackingState = false;
  
  private Constants.QuestNav.InitMode currentInitMode = null;

  private int initAttempts = 0; // NEW: Track retry attempts
  private static final int MAX_INIT_ATTEMPTS = 250; // 5 seconds @ 50Hz

  public PoseEstimatorSubsystem(
      DriveSubsystem driveSubsystem,
      RobotState robotState,
      QuestNavSubsystem questNavSubsystem) {
    
    this.driveSubsystem = driveSubsystem;
    this.robotState = robotState;
    this.questNavSubsystem = questNavSubsystem;
    this.kinematics = driveSubsystem.getKinematics();

    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        driveSubsystem.getGyroRotation(),
        driveSubsystem.getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(ODOMETRY_STD_DEVS[0], ODOMETRY_STD_DEVS[1], ODOMETRY_STD_DEVS[2]),
        VecBuilder.fill(LIMELIGHT_MULTI_TAG_STD_DEVS[0], LIMELIGHT_MULTI_TAG_STD_DEVS[1], LIMELIGHT_MULTI_TAG_STD_DEVS[2]));

    this.questNavFusion = new QuestNavFusion(questNavSubsystem, driveSubsystem, poseEstimator, this);
    this.initializer = new PoseInitializer(questNavSubsystem);
    this.validator = new PoseValidator();
    
    SmartDashboard.putData("Field", field);
    SmartLogger.logConsole("PoseEstimatorSubsystem initialized");
  }
  
  public void setTagVisionSubsystem(TagVisionSubsystem tagVisionSubsystem) {
    this.tagVisionSubsystem = tagVisionSubsystem;
  }

  public void setAutoChooser(SendableChooser<Command> autoChooser) {
    initializer.setAutoChooser(autoChooser);
    validator.setAutoChooser(autoChooser);
    validator.setPoseInitializer(initializer);
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, int tagCount) {
    addVisionMeasurement(visionPose, timestampSeconds, tagCount, "unknown");
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, int tagCount, String cameraName) {
    // Limelight only (PhotonVision cameras disabled for QuestNav-only testing)
    if (!cameraName.equals(Constants.Vision.LL_FRONT_NAME)) {
      Logger.recordOutput("PoseEstimator/VisionIgnored", true);
      Logger.recordOutput("PoseEstimator/VisionIgnoreReason", "WrongCamera: " + cameraName);
      Logger.recordOutput("PoseEstimator/VisionMeasurementIgnored", visionPose);
      Logger.recordOutput("PoseEstimator/VisionCameraIgnored", cameraName);
      return;
    }
    
    Matrix<N3, N1> stdDevs;
    
    if (tagCount >= MIN_TAG_COUNT_FOR_MULTI) {
      stdDevs = VecBuilder.fill(
          LIMELIGHT_MULTI_TAG_STD_DEVS[0],
          LIMELIGHT_MULTI_TAG_STD_DEVS[1],
          LIMELIGHT_MULTI_TAG_STD_DEVS[2]);
      Logger.recordOutput("PoseEstimator/VisionType", "Limelight_MultiTag");
    } else {
      stdDevs = VecBuilder.fill(
          LIMELIGHT_SINGLE_TAG_STD_DEVS[0],
          LIMELIGHT_SINGLE_TAG_STD_DEVS[1],
          LIMELIGHT_SINGLE_TAG_STD_DEVS[2]);
      Logger.recordOutput("PoseEstimator/VisionType", "Limelight_SingleTag");
    }
    
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
    m_lastUpdateTime = Timer.getFPGATimestamp();
    
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
    questNavFusion.notifyEstimatorReset();
    
    if (!initializer.isInitialized()) {
      initializer.setInitState(PoseInitializer.InitializationState.INITIALIZED);
      Logger.recordOutput("PoseEstimator/InitializedManually", true);
    }
    
    Logger.recordOutput("PoseEstimator/PoseReset", pose);
  }

  /**
   * SHOP-ONLY CALIBRATION TOOL 
   * 
   * DO NOT call this in match code! PoseInitializer handles automatic COMP_SEED.
   * 
   * This method should ONLY be called by SetStartingPoseCommand (which enforces safety checks).
   * 
   * IMPORTANT: Call gyro.setHeading() BEFORE calling this method!
   * The gyroAngleNow parameter must be the angle AFTER the gyro has been reset.
   * 
   * This method:
   * 1. Resets pose estimator to known field pose (using confirmed gyro angle)
   * 2. Seeds QuestNav to match (COMP_SEED mode)
   * 3. Configures validation against seed
   * 4. Marks system as initialized
   * 
   * @param pose Known field-aligned pose (e.g., from SetStartingPoseCommand)
   * @param gyroAngleNow Current gyro angle (AFTER gyro.setHeading() has been called)
   * @throws IllegalStateException if QuestNav not tracking (defensive check)
   */
  public void manualCompSeed(Pose2d pose, Rotation2d gyroAngleNow) {
    // Defensive check: QuestNav must be tracking (SetStartingPoseCommand should prevent this)
    if (!questNavSubsystem.isTracking()) {
      SmartLogger.logConsoleError("[manualCompSeed] REJECTED: QuestNav not tracking!");
      Logger.recordOutput("PoseEstimator/ManualCompSeed/Failed", "QuestNav not tracking");
      return;
    }
    
    // Defensive check: Null pose guard
    if (pose == null || gyroAngleNow == null) {
      SmartLogger.logConsoleError("[manualCompSeed] REJECTED: Null pose or gyro angle!");
      Logger.recordOutput("PoseEstimator/ManualCompSeed/Failed", "Null parameters");
      return;
    }
    
    SmartLogger.logConsole("=== MANUAL COMP_SEED ===");
    SmartLogger.logConsole("Target pose: " + SmartLogger.formatPose(pose));
    SmartLogger.logConsole("Gyro angle: " + String.format("%.1f deg", gyroAngleNow.getDegrees()));
    
    // Diagnostic: Warn if gyro angle doesn't match pose rotation (suggests timing issue)
    double angleDiff = Math.abs(gyroAngleNow.minus(pose.getRotation()).getDegrees());
    if (angleDiff > 5.0) {
      SmartLogger.logConsoleError(String.format(
          "[manualCompSeed] WARNING: Gyro angle (%.1f deg) differs from pose rotation (%.1f deg) by %.1f deg",
          gyroAngleNow.getDegrees(), pose.getRotation().getDegrees(), angleDiff));
      Logger.recordOutput("PoseEstimator/ManualCompSeed/AngleMismatch", angleDiff);
    }
    
    // Step 1: Reset pose estimator (use confirmed gyro angle - not pose rotation!)
    poseEstimator.resetPosition(
        gyroAngleNow,  // CRITICAL: Use gyro angle AFTER it's been set
        driveSubsystem.getModulePositions(),
        pose);
    SmartLogger.logConsole("✓ Pose estimator reset");
    
    // Step 2: Seed QuestNav to match field frame
    questNavSubsystem.seedToPose(pose);
    SmartLogger.logConsole("✓ QuestNav seeded");
    
    // Step 3: Configure QuestNavFusion for COMP_SEED validation
    questNavFusion.setValidationMode(Constants.QuestNav.InitMode.COMP_SEED);
    questNavFusion.onManualSeed(pose); // NEW: Reset acceptance baseline + enable fast-track
    currentInitMode = Constants.QuestNav.InitMode.COMP_SEED;
    SmartLogger.logConsole("Validation configured: COMP_SEED");
    
    // Mark as initialized (prevent PoseInitializer from re-initing)
    if (!initializer.isInitialized()) {
      initializer.setInitState(PoseInitializer.InitializationState.INITIALIZED);
    }
    SmartLogger.logConsole("Initialization state set");
    
    // Consolidated logging (avoid duplicates)
    Logger.recordOutput("PoseEstimator/ManualCompSeed/Success", true);
    Logger.recordOutput("PoseEstimator/ManualCompSeed/Pose", pose);
    Logger.recordOutput("PoseEstimator/ManualCompSeed/GyroAngle", gyroAngleNow.getDegrees());
    Logger.recordOutput("PoseEstimator/InitMode", "COMP_SEED"); // Single source
    Logger.recordOutput("QuestNav/Seeded", true); // Single source
    
    // Verify
    Pose2d actualPose = getEstimatedPose();
    double actualGyro = driveSubsystem.getGyroRotation().getDegrees();
    SmartLogger.logConsole("Actual pose: " + SmartLogger.formatPose(actualPose));
    SmartLogger.logConsole("Actual gyro: " + String.format("%.1f deg", actualGyro));
    SmartLogger.logConsole("========================");
    
    Logger.recordOutput("PoseEstimator/ManualCompSeed/VerifyPose", actualPose);
    Logger.recordOutput("PoseEstimator/ManualCompSeed/VerifyGyro", actualGyro);
  }

  public PoseInitializer.InitializationState getInitializationState() {
    return initializer.getInitState();
  }

  public boolean isInitialized() {
    return initializer.isInitialized();
  }

  public PoseInitializer getPoseInitializer() {
    return initializer;
  }

  @Override
  public void periodic() {
    if (robotState.isSysIdMode()) {
      Logger.recordOutput("PoseEstimator/SysIdMode", true);
      return;
    }
    
    logCounter++;
    poseEstimator.update(driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    
    if (robotState.getMode() == RobotState.Mode.DISABLED) {
      // 5Hz is enough for dashboard readiness display (not every 20ms loop)
      if (logCounter % 10 == 0) {
        initializer.updateReadiness();
      }
      // validator.periodicValidation(getEstimatedPose()); // Comment out (saves 50-100ms)
    }
    
    if (!initializer.isInitialized()) {
      initAttempts++; // NEW: Increment counter
      
      PoseInitializer.InitResult initResult = initializer.attemptInitialization();
      
      if (initResult != null && initResult.pose != null) {
        // SUCCESS: Reset counter
        initAttempts = 0;
        
        resetPose(initResult.pose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
        
        if (initResult.shouldSeedQuest) {
          // COMP_SEED mode (match/competition)
          questNavSubsystem.seedToPose(initResult.pose);
          questNavFusion.setExpectedSeedPose(initResult.pose);
          questNavFusion.setValidationMode(Constants.QuestNav.InitMode.COMP_SEED);
          questNavFusion.onManualSeed(initResult.pose);
          currentInitMode = Constants.QuestNav.InitMode.COMP_SEED;

          // Set driver perspective: always downfield (0 on blue, 180 on red)
          driveSubsystem.setOperatorPerspectiveForward(getDriverDownfieldAngle());

          SmartLogger.logConsole("COMP_SEED: Quest seeded to " + SmartLogger.formatPose(initResult.pose));
          Logger.recordOutput("PoseEstimator/InitMode", "COMP_SEED");
          Logger.recordOutput("QuestNav/Seeded", true);
        } else {
          // SHOP_RESUME mode (practice/testing)
          questNavSubsystem.enterResumeMode();
          questNavFusion.setExpectedSeedPose(null);
          questNavFusion.setValidationMode(Constants.QuestNav.InitMode.SHOP_RESUME);
          questNavFusion.onShopResumeInit();
          currentInitMode = Constants.QuestNav.InitMode.SHOP_RESUME;

          // In shop mode, set perspective to the robot's current physical heading
          // so "forward on stick" always means the direction the robot is facing on boot
          driveSubsystem.setOperatorPerspectiveForward(initResult.pose.getRotation());

          SmartLogger.logConsole("SHOP_RESUME: Using Quest's existing tracking (not seeded)");
          Logger.recordOutput("PoseEstimator/InitMode", "SHOP_RESUME");
          Logger.recordOutput("QuestNav/Seeded", false);
        }
        
        Logger.recordOutput("PoseEstimator/InitReason", initResult.reason);
        SmartLogger.logConsole("Init reason: " + initResult.reason);
        
      } else if (initAttempts >= MAX_INIT_ATTEMPTS) {
        // TIMEOUT: Log diagnostic info
        SmartLogger.logConsoleError("========== INITIALIZATION TIMEOUT ==========");
        SmartLogger.logConsoleError("Waited 2 seconds, no valid pose found!");
        SmartLogger.logConsoleError("Diagnostics:");
        SmartLogger.logConsoleError("  FMS attached: " + DriverStation.isFMSAttached());
        SmartLogger.logConsoleError("  Quest tracking: " + questNavSubsystem.isTracking());
        SmartLogger.logConsoleError("  Quest has pose: " + questNavSubsystem.getRobotPose().isPresent());
        
        if (questNavSubsystem.getRobotPose().isPresent()) {
          Pose2d questPose = questNavSubsystem.getRobotPose().get();
          SmartLogger.logConsoleError("  Quest pose: " + SmartLogger.formatPose(questPose));
        }
        
        SmartLogger.logConsoleError("============================================");
        
        // Reset counter to avoid spam
        initAttempts = 0;
      }
    }
    
    if (initializer.isInitialized()) {
      questNavFusion.processFrames(); // Will skip if paused
    }
    
    Pose2d currentPose = getEstimatedPose();
    robotState.setRobotPose(currentPose);
    
    field.setRobotPose(currentPose);
    questNavSubsystem.getRobotPose().ifPresent(questPose -> {
      field.getObject("QuestNav").setPose(questPose);
    });
    
    if (robotState.getNavigationPhase() != RobotState.NavigationPhase.NONE) {
      field.getObject("Target").setPoses();
    }
    
    double poseChange = currentPose.getTranslation().getDistance(lastPose.getTranslation());
    lastPose = currentPose;
    
    Logger.recordOutput("PoseEstimator/EstimatedPose", currentPose);
    Logger.recordOutput("PoseEstimator/InitState", initializer.getInitState().toString());
    Logger.recordOutput("PoseEstimator/CurrentInitMode", currentInitMode != null ? currentInitMode.toString() : "NONE");
    
    if (logCounter % (LOG_SKIP_CYCLES + 1) == 0) {
      Logger.recordOutput("PoseEstimator/PoseChangeMeters", poseChange);
      Logger.recordOutput("PoseEstimator/InitWaitTime", initializer.getWaitTime());
    }
    
    RobotState.Mode currentMode = robotState.getMode();
    if (currentMode != lastMode) {
      onModeChange(lastMode, currentMode);
      lastMode = currentMode;
    }
    
    boolean currentTracking = questNavSubsystem.isTracking();
    if (!currentTracking && lastTrackingState) {
      questNavFusion.notifyTrackingLost();
      SmartLogger.logConsoleError("QuestNav tracking lost - entering UNVALIDATED state");
    } else if (currentTracking && !lastTrackingState) {
      questNavFusion.notifyTrackingRegained();
      SmartLogger.logConsole("QuestNav tracking regained - starting revalidation");
    }
    lastTrackingState = currentTracking;
  }
  
  public double getTimeSinceLastUpdate() {
    return Timer.getFPGATimestamp() - m_lastUpdateTime;
  }

  public double getLastQuestNavMeasurementTimestamp() {
    return m_lastQuestNavMeasurementTimestamp;
  }

  public double getTimeSinceLastQuestNavFusion() {
    return Timer.getFPGATimestamp() - m_lastQuestNavFusionTime;
  }

  public void notifyQuestNavFusionOccurred(double measurementTimestamp) {
    m_lastQuestNavFusionTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    m_lastQuestNavMeasurementTimestamp = measurementTimestamp;
    m_lastUpdateTime = m_lastQuestNavFusionTime;
    
    Logger.recordOutput("PoseEstimator/QuestNav/LastFusionTime", m_lastQuestNavFusionTime);
    Logger.recordOutput("PoseEstimator/QuestNav/LastMeasurementTimestamp", m_lastQuestNavMeasurementTimestamp);
    
    double measurementAge = m_lastQuestNavFusionTime - m_lastQuestNavMeasurementTimestamp;
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAgeAtFusion", measurementAge);
  }
  
  private void onModeChange(RobotState.Mode from, RobotState.Mode to) {
    if (to == RobotState.Mode.ENABLED_AUTO) {
      SmartLogger.logConsole("=== AUTO ENABLED ===");
      hasEverBeenEnabled = true;
    }
    
    if (from == RobotState.Mode.ENABLED_AUTO && to == RobotState.Mode.ENABLED_TELEOP) {
      SmartLogger.logConsole("=== AUTO TO TELEOP TRANSITION ===");
      SmartLogger.logConsole("Keeping pose from auto");
      
      Pose2d currentPose = getEstimatedPose();
      SmartLogger.logConsole("Current pose: " + SmartLogger.formatPose(currentPose));
      SmartLogger.logConsole("Current heading: " + String.format("%.1f deg", currentPose.getRotation().getDegrees()));
      
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
      initializer.setInitState(PoseInitializer.InitializationState.INITIALIZED);
      Logger.recordOutput("PoseEstimator/InitializedViaMultiTagVision", true);
    }
  }
  
  public boolean forceAcceptQuestNavPose() {
    return questNavFusion.forceAcceptMeasurement();
  }

  // Returns the downfield direction from the driver's perspective.
  // Blue driver faces +X (0 deg), Red driver faces -X (180 deg).
  private Rotation2d getDriverDownfieldAngle() {
    boolean isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
    return Rotation2d.fromDegrees(isRed ? 180.0 : 0.0);
  }
}