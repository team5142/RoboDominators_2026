package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private double lastUpdateTime = 0.0;
  private double lastQuestNavFusionTime = 0.0;
  private double lastQuestNavMeasurementTimestamp = 0.0;

  private boolean lastTrackingState = false;
  
  private Constants.QuestNav.InitMode currentInitMode = null;

  private int initAttempts = 0;
  private static final int MAX_INIT_ATTEMPTS = 250; // 5 seconds @ 50Hz

  // Tracks the auto name that was last seeded so we can detect chooser changes while disabled.
  private String lastSeededAutoName = null;

  public PoseEstimatorSubsystem(
      DriveSubsystem driveSubsystem,
      RobotState robotState,
      QuestNavSubsystem questNavSubsystem) {
    
    this.driveSubsystem = driveSubsystem;
    this.robotState = robotState;
    this.questNavSubsystem = questNavSubsystem;

    poseEstimator = new SwerveDrivePoseEstimator(
        driveSubsystem.getKinematics(),
        driveSubsystem.getGyroRotation(),
        driveSubsystem.getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(ODOMETRY_STD_DEVS[0], ODOMETRY_STD_DEVS[1], ODOMETRY_STD_DEVS[2]),
        VecBuilder.fill(LIMELIGHT_MULTI_TAG_STD_DEVS[0], LIMELIGHT_MULTI_TAG_STD_DEVS[1], LIMELIGHT_MULTI_TAG_STD_DEVS[2]));

    this.questNavFusion = new QuestNavFusion(questNavSubsystem, driveSubsystem, poseEstimator, this);
    this.initializer = new PoseInitializer(questNavSubsystem);
    this.validator = new PoseValidator();
    
    SmartDashboard.putData("Field", field);
    SmartLogger.logConsole("PoseEstimatorSubsystem initialized", "Pose");
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
      // No logging here - this fires every loop for every ignored camera and bloats logs
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
    lastUpdateTime = Timer.getFPGATimestamp();
    
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

  // SHOP-ONLY: Seeds robot pose and QuestNav to a known field position.
  // Only call from SetStartingPoseCommand - call gyro.setHeading() BEFORE calling this.
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
    
    SmartLogger.logConsole("=== MANUAL COMP_SEED ===", "Pose");
    SmartLogger.logConsole("Target pose: " + SmartLogger.formatPose(pose), "Pose");
    SmartLogger.logConsole("Gyro angle: " + String.format("%.1f deg", gyroAngleNow.getDegrees()), "Pose");
    
    // Diagnostic: Warn if gyro angle doesn't match pose rotation (suggests timing issue)
    double angleDiff = Math.abs(gyroAngleNow.minus(pose.getRotation()).getDegrees());
    if (angleDiff > 5.0) {
      SmartLogger.logConsoleError(String.format(
          "[manualCompSeed] WARNING: Gyro angle (%.1f deg) differs from pose rotation (%.1f deg) by %.1f deg",
          gyroAngleNow.getDegrees(), pose.getRotation().getDegrees(), angleDiff));
      Logger.recordOutput("PoseEstimator/ManualCompSeed/AngleMismatch", angleDiff);
    }
    
    // Use confirmed gyro angle (not pose rotation) - must match what gyro.setHeading() set
    poseEstimator.resetPosition(
        gyroAngleNow,
        driveSubsystem.getModulePositions(),
        pose);
    SmartLogger.logConsole("Pose estimator reset", "Pose");
    
    // Seed QuestNav to match field frame
    questNavSubsystem.seedToPose(pose);
    SmartLogger.logConsole("QuestNav seeded", "Pose");
    
    // Configure QuestNavFusion for COMP_SEED validation
    questNavFusion.setValidationMode(Constants.QuestNav.InitMode.COMP_SEED);
    questNavFusion.onManualSeed(pose);
    currentInitMode = Constants.QuestNav.InitMode.COMP_SEED;
    SmartLogger.logConsole("Validation configured: COMP_SEED", "Pose");
    
    // Mark as initialized (prevent PoseInitializer from re-initing)
    if (!initializer.isInitialized()) {
      initializer.setInitState(PoseInitializer.InitializationState.INITIALIZED);
    }
    SmartLogger.logConsole("Initialization state set", "Pose");
    
    // Consolidated logging (avoid duplicates)
    Logger.recordOutput("PoseEstimator/ManualCompSeed/Success", true);
    Logger.recordOutput("PoseEstimator/ManualCompSeed/Pose", pose);
    Logger.recordOutput("PoseEstimator/ManualCompSeed/GyroAngle", gyroAngleNow.getDegrees());
    Logger.recordOutput("PoseEstimator/InitMode", "COMP_SEED"); // Single source
    Logger.recordOutput("QuestNav/Seeded", true); // Single source
    
    // Verify
    Pose2d actualPose = getEstimatedPose();
    double actualGyro = driveSubsystem.getGyroRotation().getDegrees();
    SmartLogger.logConsole("Actual pose: " + SmartLogger.formatPose(actualPose), "Pose");
    SmartLogger.logConsole("Actual gyro: " + String.format("%.1f deg", actualGyro), "Pose");
    SmartLogger.logConsole("========================", "Pose");
    
    Logger.recordOutput("PoseEstimator/ManualCompSeed/VerifyPose", actualPose);
    Logger.recordOutput("PoseEstimator/ManualCompSeed/VerifyGyro", actualGyro);
  }

  public PoseInitializer.InitializationState getInitializationState() {
    return initializer.getInitState();
  }

  public boolean isInitialized() {
    return initializer.isInitialized();
  }

  // Called at teleopInit to re-seed gyro and field perspective from auto end pose.
  // Without this, field orientation is stale if auto ended at a different heading.
  public void onTeleopInit() {
    Pose2d currentPose = getEstimatedPose();
    Rotation2d heading = currentPose.getRotation();
    driveSubsystem.setGyroHeading(heading);
    driveSubsystem.setOperatorPerspectiveForward(getDriverDownfieldAngle());
    Logger.recordOutput("Transition/TeleopInitPose", currentPose);
    Logger.recordOutput("Transition/TeleopInitHeadingDeg", heading.getDegrees());
    SmartLogger.logConsole("Teleop init: gyro seeded to " + String.format("%.1f", heading.getDegrees())
        + " deg, perspective=" + String.format("%.1f", getDriverDownfieldAngle().getDegrees()) + " deg", "Pose");
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
      // 5Hz is enough for dashboard readiness display and auto-change detection
      if (logCounter % 10 == 0) {
        initializer.updateReadiness();

        // If the auto chooser selection changes while disabled, reset init so the pose
        // estimator and Quest re-seed to the new auto's start position immediately.
        // This lets the driver see the correct robot position on the field in Elastic
        // before enabling, and change autos freely with live position updates.
        if (initializer.isInitialized()) {
          String currentAutoName = initializer.getSelectedAutoName();
          if (currentAutoName != null && !currentAutoName.equals(lastSeededAutoName)) {
            SmartLogger.logConsole("Auto changed: '" + lastSeededAutoName + "' -> '" + currentAutoName + "' - re-seeding", "Pose");
            Logger.recordOutput("PoseEstimator/AutoChangedReseed", currentAutoName);
            initializer.setInitState(PoseInitializer.InitializationState.WAITING);
            initAttempts = 0;
          }
        }
      }
    }
    
    if (!initializer.isInitialized()) {
      initAttempts++;
      
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

          SmartLogger.logConsole("COMP_SEED: Quest seeded to " + SmartLogger.formatPose(initResult.pose), "Pose");
          Logger.recordOutput("PoseEstimator/InitMode", "COMP_SEED");
          Logger.recordOutput("QuestNav/Seeded", true);
          lastSeededAutoName = initializer.getSelectedAutoName();
          Logger.recordOutput("Auto/SeededAutoName", lastSeededAutoName != null ? lastSeededAutoName : "");
        } else {
          // SHOP_RESUME mode (practice/testing)
          questNavSubsystem.enterResumeMode();
          questNavFusion.setExpectedSeedPose(null);
          questNavFusion.setValidationMode(Constants.QuestNav.InitMode.SHOP_RESUME);
          questNavFusion.onShopResumeInit();
          currentInitMode = Constants.QuestNav.InitMode.SHOP_RESUME;

          // Seed the Pigeon to Quest's reported heading so CTRE field-centric math is correct.
          // Quest SLAM heading = actual physical robot heading (unanchored but self-consistent).
          // After seeding, perspective = allianceDownfield so forward always points toward Red wall.
          Rotation2d questHeading = initResult.pose.getRotation();
          // Cache once - reads DriverStation.getAlliance() which is a network call
          Rotation2d downfield = getDriverDownfieldAngle();
          driveSubsystem.setGyroHeading(questHeading);
          driveSubsystem.setOperatorPerspectiveForward(downfield);
          resetPose(initResult.pose, questHeading, driveSubsystem.getModulePositions());

          SmartLogger.logConsole("SHOP_RESUME: questHeading=" + String.format("%.1f", questHeading.getDegrees())
              + " perspective=" + String.format("%.1f", downfield.getDegrees()), "Pose");
          Logger.recordOutput("PoseEstimator/InitMode", "SHOP_RESUME");
          Logger.recordOutput("QuestNav/Seeded", false);
          // SHOP_RESUME has no specific auto target - clear so auto changes can still re-seed
          lastSeededAutoName = null;
        }
        
        Logger.recordOutput("PoseEstimator/InitReason", initResult.reason);
        SmartLogger.logConsole("Init reason: " + initResult.reason, "Pose");
        
      } else if (initAttempts >= MAX_INIT_ATTEMPTS) {
        // TIMEOUT: Log diagnostic info
        SmartLogger.logConsoleError("========== INITIALIZATION TIMEOUT ==========");
        SmartLogger.logConsoleError("Waited 5 seconds, no valid pose found!");
        SmartLogger.logConsoleError("Diagnostics:");
        SmartLogger.logConsoleError("  FMS attached: " + DriverStation.isFMSAttached());
        SmartLogger.logConsoleError("  Quest tracking: " + questNavSubsystem.isTracking());
        SmartLogger.logConsoleError("  Quest has pose: " + questNavSubsystem.getRobotPose().isPresent());
        
        if (questNavSubsystem.getRobotPose().isPresent()) {
          Pose2d questPose = questNavSubsystem.getRobotPose().get();
          SmartLogger.logConsoleError("  Quest pose: " + SmartLogger.formatPose(questPose));
        }
        
        SmartLogger.logConsoleError("============================================");
        
        // Reset counter to restart 5s countdown (will log again if still not initialized)
        initAttempts = 0;
      }
    }
    
    if (initializer.isInitialized()) {
      questNavFusion.processFrames(); // Will skip if paused
    }
    
    Pose2d currentPose = getEstimatedPose();
    robotState.setRobotPose(currentPose);
    
    // Update field pose every loop (required for Elastic Field widget)
    field.setRobotPose(currentPose);
    // QuestNav overlay and Target only updated when needed - reduces Field2d serialization cost
    if (logCounter % 5 == 0) {
      questNavSubsystem.getRobotPose().ifPresent(questPose -> {
        field.getObject("QuestNav").setPose(questPose);
      });

      // Show the auto start target while disabled so the driver can confirm the robot
      // is in the right spot before enabling. Marker disappears once enabled.
      if (robotState.getMode() == RobotState.Mode.DISABLED) {
        String autoName = initializer.getSelectedAutoName();
        Pose2d autoTarget = autoName != null ? initializer.getStartPoseForAutoName(autoName) : null;
        if (autoTarget != null) {
          field.getObject("AutoTarget").setPose(autoTarget);
        } else {
          field.getObject("AutoTarget").setPoses(); // clear marker
        }
      } else {
        field.getObject("AutoTarget").setPoses(); // clear once enabled
      }

      // Clear navigation target marker when not navigating (throttled same as other field objects)
      if (robotState.getNavigationPhase() == RobotState.NavigationPhase.NONE) {
        field.getObject("Target").setPoses();
      }
    }
    
    double poseChange = currentPose.getTranslation().getDistance(lastPose.getTranslation());
    lastPose = currentPose;
    
    Logger.recordOutput("PoseEstimator/EstimatedPose", currentPose);
    Logger.recordOutput("PoseEstimator/InitState", initializer.getInitState().toString());
    Logger.recordOutput("PoseEstimator/CurrentInitMode", currentInitMode != null ? currentInitMode.toString() : "NONE");

    // Publish flat pose values for the HTML dashboard (throttled to 10Hz)
    if (logCounter % 5 == 0) {
      SmartDashboard.putNumber("Robot/PoseX", currentPose.getX());
      SmartDashboard.putNumber("Robot/PoseY", currentPose.getY());
      SmartDashboard.putNumber("Robot/Heading", currentPose.getRotation().getDegrees());
    }

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
      SmartLogger.logConsole("QuestNav tracking regained - starting revalidation", "Pose");
    }
    lastTrackingState = currentTracking;
  }
  
  public double getTimeSinceLastUpdate() {
    return Timer.getFPGATimestamp() - lastUpdateTime;
  }

  public double getLastQuestNavMeasurementTimestamp() {
    return lastQuestNavMeasurementTimestamp;
  }

  public double getTimeSinceLastQuestNavFusion() {
    return Timer.getFPGATimestamp() - lastQuestNavFusionTime;
  }

  public void notifyQuestNavFusionOccurred(double measurementTimestamp) {
    lastQuestNavFusionTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    lastQuestNavMeasurementTimestamp = measurementTimestamp;
    lastUpdateTime = lastQuestNavFusionTime;
    
    Logger.recordOutput("PoseEstimator/QuestNav/LastFusionTime", lastQuestNavFusionTime);
    Logger.recordOutput("PoseEstimator/QuestNav/LastMeasurementTimestamp", lastQuestNavMeasurementTimestamp);
    
    double measurementAge = lastQuestNavFusionTime - lastQuestNavMeasurementTimestamp;
    Logger.recordOutput("PoseEstimator/QuestNav/MeasurementAgeAtFusion", measurementAge);
  }
  
  private void onModeChange(RobotState.Mode from, RobotState.Mode to) {
    if (to == RobotState.Mode.ENABLED_AUTO) {
      SmartLogger.logConsole("=== AUTO ENABLED ===", "Pose");
      hasEverBeenEnabled = true;
    }
    
    if (from == RobotState.Mode.ENABLED_AUTO && to == RobotState.Mode.ENABLED_TELEOP) {
      SmartLogger.logConsole("=== AUTO TO TELEOP TRANSITION ===", "Pose");
      SmartLogger.logConsole("Keeping pose from auto", "Pose");
      
      Pose2d currentPose = getEstimatedPose();
      SmartLogger.logConsole("Current pose: " + SmartLogger.formatPose(currentPose), "Pose");
      SmartLogger.logConsole("Current heading: " + String.format("%.1f deg", currentPose.getRotation().getDegrees()), "Pose");
      
      driveSubsystem.setOperatorPerspectiveForward(currentPose.getRotation());
      
      Logger.recordOutput("PoseEstimator/AutoToTeleopTransition", true);
      Logger.recordOutput("PoseEstimator/TransitionPose", currentPose);
    }
    
    if (to == RobotState.Mode.ENABLED_TELEOP && !hasEverBeenEnabled) {
      SmartLogger.logConsole("=== TELEOP ENABLED (First Enable) ===", "Pose");
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
