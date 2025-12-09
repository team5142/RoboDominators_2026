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
import frc.robot.RobotState;
import frc.robot.subsystems.pose.PoseInitializer;
import frc.robot.subsystems.pose.QuestNavFusion;
import frc.robot.subsystems.pose.PoseValidator;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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
    this.questNavFusion = new QuestNavFusion(questNavSubsystem, driveSubsystem, poseEstimator);
    this.initializer = new PoseInitializer(questNavSubsystem, questNavFusion); // CHANGED: Pass questNavFusion
    this.validator = new PoseValidator();
    
    System.out.println("PoseEstimatorSubsystem initialized (refactored with helpers)");
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
    // Vision updates disabled for QuestNav-only testing
    Logger.recordOutput("PoseEstimator/VisionUpdatesDisabled", true);
    Logger.recordOutput("PoseEstimator/VisionMeasurementIgnored", visionPose);
    Logger.recordOutput("PoseEstimator/VisionTagCountIgnored", tagCount);
    Logger.recordOutput("PoseEstimator/VisionCameraIgnored", cameraName);
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
        // This locks in the initial position before auto starts
        java.util.Optional<Pose2d> questPose = questNavSubsystem.getRobotPose();
        if (questPose.isPresent()) {
          poseEstimator.addVisionMeasurement(
              questPose.get(),
              edu.wpi.first.wpilibj.Timer.getFPGATimestamp(),
              initialStdDevs); // VERY HIGH TRUST (1cm XY, 1 deg theta)
          
          Logger.recordOutput("PoseEstimator/QuestNav/InitialAlignmentApplied", true);
          System.out.println("QuestNav initial alignment applied with VERY HIGH TRUST");
        }
      }
    }
    
    // Process QuestNav measurements (if initialized)
    if (initializer.isInitialized()) {
      questNavFusion.processFrames();
    }
    
    // Update robot state
    Pose2d currentPose = getEstimatedPose();
    robotState.setRobotPose(currentPose);
    
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
  
  private void onModeChange(RobotState.Mode from, RobotState.Mode to) {
    if (to == RobotState.Mode.ENABLED_AUTO) {
      System.out.println("=== AUTO ENABLED ===");
      hasEverBeenEnabled = true;
    }
    
    if (from == RobotState.Mode.ENABLED_AUTO && to == RobotState.Mode.ENABLED_TELEOP) {
      System.out.println("=== AUTO → TELEOP TRANSITION ===");
      System.out.println("KEEPING pose from auto - no reset!");
      
      Pose2d currentPose = getEstimatedPose();
      System.out.println("Current pose: " + formatPose(currentPose));
      System.out.println("Current heading: " + currentPose.getRotation().getDegrees() + "°");
      
      driveSubsystem.setOperatorPerspectiveForward(currentPose.getRotation());
      
      Logger.recordOutput("PoseEstimator/AutoToTeleopTransition", true);
      Logger.recordOutput("PoseEstimator/TransitionPose", currentPose);
    }
    
    if (to == RobotState.Mode.ENABLED_TELEOP && !hasEverBeenEnabled) {
      System.out.println("=== TELEOP ENABLED (First Enable) ===");
      hasEverBeenEnabled = true;
      
      if (!initializer.isInitialized()) {
        System.err.println("WARNING: Enabled in teleop without pose!");
        System.err.println("Robot needs QuestNav or manual pose set");
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
}