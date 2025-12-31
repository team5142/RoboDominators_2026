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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.SmartLogger;

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

  private double lastForceUpdateWarningTime = 0.0;
  private static final double FORCE_UPDATE_WARNING_INTERVAL = 0.5;

  private boolean lastTrackingState = false;
  
  private Constants.QuestNav.InitMode currentInitMode = null;

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
        driveSubsystem.getGyroRotation(), // Yaw from Pigeon (via DriveSubsystem)
        driveSubsystem.getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(ODOMETRY_STD_DEVS[0], ODOMETRY_STD_DEVS[1], ODOMETRY_STD_DEVS[2]),
        VecBuilder.fill(LIMELIGHT_MULTI_TAG_STD_DEVS[0], LIMELIGHT_MULTI_TAG_STD_DEVS[1], LIMELIGHT_MULTI_TAG_STD_DEVS[2]));

    this.questNavFusion = new QuestNavFusion(questNavSubsystem, driveSubsystem, poseEstimator, this);
    this.initializer = new PoseInitializer(questNavSubsystem); // FIXED: Removed questNavFusion param
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
      initializer.setInitState(PoseInitializer.InitializationState.INITIALIZED); // FIXED: Was VISION_INITIALIZED
      Logger.recordOutput("PoseEstimator/InitializedManually", true);
    }
    
    Logger.recordOutput("PoseEstimator/PoseReset", pose);
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
    poseEstimator.update(driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    
    if (robotState.getMode() == RobotState.Mode.DISABLED) {
      initializer.updateReadiness();
      validator.periodicValidation(getEstimatedPose());
    }
    
    if (!initializer.isInitialized()) {
      PoseInitializer.InitResult initResult = initializer.attemptInitialization();
      
      if (initResult != null && initResult.pose != null) {
        resetPose(initResult.pose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
        
        if (initResult.shouldSeedQuest) {
          questNavSubsystem.seedToPose(initResult.pose);
          questNavFusion.setExpectedSeedPose(initResult.pose);
          questNavFusion.setValidationMode(Constants.QuestNav.InitMode.COMP_SEED);
          currentInitMode = Constants.QuestNav.InitMode.COMP_SEED;
          
          SmartLogger.logConsole("COMP_SEED: Quest seeded to " + formatPose(initResult.pose));
          Logger.recordOutput("PoseEstimator/InitMode", "COMP_SEED");
          Logger.recordOutput("PoseEstimator/QuestSeeded", true);
        } else {
          questNavSubsystem.enterResumeMode();
          questNavFusion.setExpectedSeedPose(null);
          questNavFusion.setValidationMode(Constants.QuestNav.InitMode.SHOP_RESUME);
          currentInitMode = Constants.QuestNav.InitMode.SHOP_RESUME;
          
          SmartLogger.logConsole("SHOP_RESUME: Using Quest's existing tracking (not seeded)");
          Logger.recordOutput("PoseEstimator/InitMode", "SHOP_RESUME");
          Logger.recordOutput("PoseEstimator/QuestSeeded", false);
        }
        
        Logger.recordOutput("PoseEstimator/InitReason", initResult.reason);
        SmartLogger.logConsole("Init reason: " + initResult.reason);
      }
    }
    
    // Quest fusion runs even while disabled (maintains pose continuity if robot is moved)
    if (initializer.isInitialized()) {
      questNavFusion.processFrames();
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
      Logger.recordOutput("PoseEstimator/WaitingForVisionTime", initializer.getWaitTime());
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
    
    // HealthState already logged by QuestNavFusion.processFrames()
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
      initializer.setInitState(PoseInitializer.InitializationState.INITIALIZED); // FIXED: Was VISION_INITIALIZED
      Logger.recordOutput("PoseEstimator/InitializedViaMultiTagVision", true);
    }
  }
  
  private String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", 
        pose.getX(), 
        pose.getY(), 
        pose.getRotation().getDegrees());
  }
  
  public boolean forceAcceptQuestNavPose() {
    return questNavFusion.forceAcceptMeasurement();
  }
}