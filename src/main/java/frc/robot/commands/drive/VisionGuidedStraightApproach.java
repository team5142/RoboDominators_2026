package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;
import frc.robot.subsystems.GyroSubsystem; 
import frc.robot.subsystems.QuestNavSubsystem; // ADD THIS
import org.littletonrobotics.junction.Logger;

/**
 * Vision-guided straight approach to target with advanced features:
 * 
 * VISION FALLBACK STRATEGY:
 *   1. Multi-tag (2+) vision - HIGHEST trust
 *   2. Single-tag vision - MEDIUM trust  
 *   3. Odometry only - LOW trust (fallback)
 * 
 * SAFETY FEATURES:
 *   - Driver override detection (any joystick input cancels)
 *   - Stuck detection (no movement for 2s → abort)
 *   - Vision loss handling (continue with odometry)
 *   - Emergency abort (timeout safety)
 * 
 * LOGGING:
 *   - Complete telemetry to SmartDashboard
 *   - AdvantageScope logging for post-match analysis
 *   - Real-time progress updates
 */
public class VisionGuidedStraightApproach extends Command {
  
  // Subsystems
  private final DriveSubsystem driveSubsystem;
  private final PoseEstimatorSubsystem poseEstimator;
  private final TagVisionSubsystem tagVision;
  private final RobotState robotState;
  private final XboxController driverController;
  private final QuestNavSubsystem questNavSubsystem; // CHANGED: Use QuestNav directly
  
  // Target
  private final Pose2d targetPose;
  
  // PID controllers for precision driving
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  
  // Vision quality tracking
  private enum VisionQuality {
    MULTI_TAG,    // 2+ tags (best)
    SINGLE_TAG,   // 1 tag (acceptable)
    ODOMETRY_ONLY // No vision (fallback)
  }
  private VisionQuality currentVisionQuality = VisionQuality.ODOMETRY_ONLY;
  private int consecutiveVisionLosses = 0;
  private static final int MAX_VISION_LOSSES = 10; // 0.2s at 20ms loop
  
  // Stuck detection
  private final Timer stuckTimer = new Timer();
  private Pose2d lastPose;
  private static final double STUCK_TIMEOUT_SECONDS = 2.0;
  private static final double STUCK_DISTANCE_THRESHOLD = 0.05; // 5cm in 2s
  
  // Tolerances
  private static final double POSITION_TOLERANCE_METERS = 0.05; // 5cm
  private static final double ROTATION_TOLERANCE_DEGREES = 0.5; // 0.5°
  
  // NEW: Settling parameters
  private static final double SETTLING_TIME_SECONDS = 0.3; // Hold position for 300ms
  private final Timer settlingTimer = new Timer();
  private boolean isSettling = false;

  // NEW: Vision requirement for finishing
  private static final int MIN_VISION_SAMPLES_FOR_FINISH = 3; // Require 3 consecutive vision updates
  private int consecutiveVisionSamples = 0;

  // Driver override detection
  private static final double JOYSTICK_OVERRIDE_THRESHOLD = 0.15;
  
  // Speed limits
  private static final double MAX_SPEED_MPS = 1.2; // CHANGED: 1.2 m/s (~4 ft/s) - good balance
  private static final double MAX_ROTATION_RAD_PER_SEC = Math.PI * 0.5;
  
  /**
   * Create vision-guided straight approach to target
   */
  public VisionGuidedStraightApproach(
      Pose2d target,
      PoseEstimatorSubsystem poseEstimator,
      TagVisionSubsystem tagVision,
      RobotState robotState,
      XboxController driverController,
      DriveSubsystem driveSubsystem,
      QuestNavSubsystem questNavSubsystem) { // CHANGED: Accept QuestNavSubsystem instead
    
    this.targetPose = target;
    this.poseEstimator = poseEstimator;
    this.tagVision = tagVision;
    this.robotState = robotState;
    this.driverController = driverController;
    this.driveSubsystem = driveSubsystem;
    this.questNavSubsystem = questNavSubsystem; // CHANGED
    
    // PID controllers - AGGRESSIVE for fast precision approach
    // With P=2.5 and 1m error, we get 2.5 m/s command (clamped to 1.2 m/s max)
    // This means we run at FULL SPEED until very close to target!
    xController = new PIDController(1, 0.0, 1);  
    yController = new PIDController(1, 0.0, 1);  
    thetaController = new PIDController(1.5, 0.0, 1.5); // CHANGED: 5.0 (was 3.0) - more aggressive rotation
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Set tolerances
    xController.setTolerance(POSITION_TOLERANCE_METERS);
    yController.setTolerance(POSITION_TOLERANCE_METERS);
    thetaController.setTolerance(Math.toRadians(ROTATION_TOLERANCE_DEGREES));
    
    addRequirements(driveSubsystem);
  }
  
  @Override
  public void initialize() {
    System.out.println("=== VISION PRECISION APPROACH ===");
    System.out.println("Target: " + formatPose(targetPose));
    
    lastPose = poseEstimator.getEstimatedPose();
    stuckTimer.restart();
    settlingTimer.restart(); // NEW
    consecutiveVisionLosses = 0;
    isSettling = false; // NEW
    
    robotState.setNavigationPhase(RobotState.NavigationPhase.PRECISION);
    
    SmartDashboard.putString("Precision/Status", "Active");
    SmartDashboard.putNumber("Precision/Progress", 0.0);
    
    Logger.recordOutput("Precision/Started", true);
    Logger.recordOutput("Precision/Target", targetPose);
  }
  
  @Override
  public void execute() {
    // ===== DRIVER OVERRIDE CHECK =====
    if (driverWantsControl()) {
      System.out.println("Driver override detected - canceling precision approach");
      robotState.setNavigationPhase(RobotState.NavigationPhase.DRIVER_OVERRIDE);
      cancel();
      return;
    }
    
    // ===== VISION QUALITY ASSESSMENT =====
    updateVisionQuality();
    
    // ===== STUCK DETECTION =====
    Pose2d currentPose = poseEstimator.getEstimatedPose();
    double movementDistance = currentPose.getTranslation()
        .getDistance(lastPose.getTranslation());
    
    // NEW: Check if we're close to target BEFORE declaring stuck
    double distanceToTarget = currentPose.getTranslation()
        .getDistance(targetPose.getTranslation());
    
    // Only check for stuck if we're still far from target (>10cm)
    if (distanceToTarget > 0.10) {
      if (movementDistance < STUCK_DISTANCE_THRESHOLD) {
        if (stuckTimer.hasElapsed(STUCK_TIMEOUT_SECONDS)) {
          System.err.println("Robot stuck - aborting precision approach!");
          robotState.setNavigationPhase(RobotState.NavigationPhase.STUCK);
          DriverStation.reportError("Robot stuck during precision approach", false);
          cancel();
          return;
        }
      } else {
        stuckTimer.restart(); // Making progress - reset timer
      }
    } else {
      // Close to target - don't check for stuck (robot is settling)
      stuckTimer.restart();
    }
    
    lastPose = currentPose;
    
    // ===== CALCULATE CONTROL OUTPUTS =====
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double thetaError = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
    
    // Normalize theta error to [-π, π]
    while (thetaError > Math.PI) thetaError -= 2 * Math.PI;
    while (thetaError < -Math.PI) thetaError += 2 * Math.PI;
    
    // PID control
    double xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());
    double yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());
    double thetaVelocity = thetaController.calculate(
        currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians());
    
    // Clamp speeds
    xVelocity = Math.max(-MAX_SPEED_MPS, Math.min(MAX_SPEED_MPS, xVelocity));
    yVelocity = Math.max(-MAX_SPEED_MPS, Math.min(MAX_SPEED_MPS, yVelocity));
    thetaVelocity = Math.max(-MAX_ROTATION_RAD_PER_SEC, 
                             Math.min(MAX_ROTATION_RAD_PER_SEC, thetaVelocity));
    
    // Field-relative drive
    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity);
    driveSubsystem.driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds,
            currentPose.getRotation()));
    
    // ===== PROGRESS TRACKING =====
    double distanceRemaining = currentPose.getTranslation()
        .getDistance(targetPose.getTranslation());
    double progress = Math.max(0.0, 1.0 - (distanceRemaining / 2.0)); // Assume started ~2m away
    
    SmartDashboard.putNumber("Precision/Progress", progress);
    SmartDashboard.putNumber("Precision/DistanceRemaining", distanceRemaining);
    SmartDashboard.putNumber("Precision/XError", xError);
    SmartDashboard.putNumber("Precision/YError", yError);
    SmartDashboard.putNumber("Precision/ThetaError", Math.toDegrees(thetaError));
    SmartDashboard.putString("Precision/VisionQuality", currentVisionQuality.toString());
    
    Logger.recordOutput("Precision/Progress", progress);
    Logger.recordOutput("Precision/DistanceRemaining", distanceRemaining);
    Logger.recordOutput("Precision/XError", xError);
    Logger.recordOutput("Precision/YError", yError);
    Logger.recordOutput("Precision/ThetaError", thetaError);
    Logger.recordOutput("Precision/VisionQuality", currentVisionQuality.toString());
    Logger.recordOutput("Precision/XVelocity", xVelocity);
    Logger.recordOutput("Precision/YVelocity", yVelocity);
    Logger.recordOutput("Precision/ThetaVelocity", thetaVelocity);
  }
  
  /**
   * Update vision quality and handle fallback
   */
  private void updateVisionQuality() {
    if (tagVision.hasMultiTagDetection()) {
      // BEST: Multi-tag vision
      currentVisionQuality = VisionQuality.MULTI_TAG;
      consecutiveVisionLosses = 0;
      
    } else if (tagVision.hasSingleTagDetection()) {
      // ACCEPTABLE: Single-tag vision
      currentVisionQuality = VisionQuality.SINGLE_TAG;
      consecutiveVisionLosses = 0;
      
    } else {
      // VISION LOST: Increment counter
      consecutiveVisionLosses++;
      
      if (consecutiveVisionLosses > MAX_VISION_LOSSES) {
        // Switch to odometry-only mode
        if (currentVisionQuality != VisionQuality.ODOMETRY_ONLY) {
          System.err.println("Vision lost - falling back to odometry only");
          currentVisionQuality = VisionQuality.ODOMETRY_ONLY;
          robotState.setNavigationPhase(RobotState.NavigationPhase.VISION_LOST);
          
          SmartDashboard.putString("Precision/Warning", "Vision lost - odometry only");
          Logger.recordOutput("Precision/VisionLost", true);
        }
      }
    }
  }
  
  /**
   * Check if driver wants to take control
   */
  private boolean driverWantsControl() {
    double xInput = Math.abs(driverController.getLeftY());
    double yInput = Math.abs(driverController.getLeftX());
    double rotInput = Math.abs(driverController.getRightX());
    
    boolean override = xInput > JOYSTICK_OVERRIDE_THRESHOLD ||
                      yInput > JOYSTICK_OVERRIDE_THRESHOLD ||
                      rotInput > JOYSTICK_OVERRIDE_THRESHOLD;
    
    if (override) {
      Logger.recordOutput("Precision/DriverOverride", true);
      SmartDashboard.putString("Precision/Status", "Driver override");
    }
    
    return override;
  }
  
  @Override
  public boolean isFinished() {
    // Check if at target within tolerance
    Pose2d currentPose = poseEstimator.getEstimatedPose();
    
    double distanceError = currentPose.getTranslation()
        .getDistance(targetPose.getTranslation());
    double rotationError = Math.abs(
        currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());
    
    boolean atTarget = distanceError < POSITION_TOLERANCE_METERS &&
                      rotationError < ROTATION_TOLERANCE_DEGREES;
    
    // CHANGED: Check if we have ACTIVE vision OR QuestNav
    boolean hasActiveVision = (currentVisionQuality == VisionQuality.MULTI_TAG) ||
                              (currentVisionQuality == VisionQuality.SINGLE_TAG);
    
    boolean hasQuestNav = questNavSubsystem.isCalibrated() && 
                          questNavSubsystem.getRobotPose().isPresent(); // FIXED
    
    boolean hasPoseConfirmation = hasActiveVision || hasQuestNav;
    
    if (atTarget) {
      // CHANGED: Require vision OR QuestNav before finishing
      if (!hasPoseConfirmation) {
        // At target per odometry, but NO EXTERNAL POSE SOURCE - don't trust it!
        if (!isSettling) {
          System.err.println("WARNING: At target per odometry, but NO VISION/QUESTNAV!");
          System.err.println("Continuing to drive until pose confirmed...");
        }
        isSettling = false;
        consecutiveVisionSamples = 0;
        Logger.recordOutput("Precision/WaitingForPoseConfirmation", true);
        return false; // Keep driving until we get confirmation
      }
      
      // Count consecutive pose-confirmed samples
      consecutiveVisionSamples++;
      
      if (!isSettling) {
        // Just reached target WITH confirmation - start settling timer
        isSettling = true;
        settlingTimer.restart();
        
        String confirmationSource = hasActiveVision ? "Vision" : "QuestNav";
        System.out.println("Pose confirmed by: " + confirmationSource);
        
        Logger.recordOutput("Precision/Settling", true);
        Logger.recordOutput("Precision/VisionSamples", consecutiveVisionSamples);
        Logger.recordOutput("Precision/ConfirmationSource", confirmationSource);
        
      } else if (settlingTimer.hasElapsed(SETTLING_TIME_SECONDS) && 
                 consecutiveVisionSamples >= MIN_VISION_SAMPLES_FOR_FINISH) {
        // Held position stable for required time WITH confirmation - DONE!
        String confirmationSource = hasActiveVision ? currentVisionQuality.toString() : "QuestNav";
        
        System.out.println("=== PRECISION COMPLETE ===");
        System.out.println("Final error: " + String.format("%.2fcm, %.1f°", 
            distanceError * 100, rotationError));
        System.out.println("Pose source: " + confirmationSource);
        System.out.println("Confirmation samples: " + consecutiveVisionSamples);
        
        Logger.recordOutput("Precision/FinalConfirmationSource", confirmationSource);
        
        return true;
      }
      
      // Still settling - keep holding position
      return false;
      
    } else {
      // Drifted outside tolerance - restart settling
      if (isSettling) {
        isSettling = false;
        consecutiveVisionSamples = 0;
        Logger.recordOutput("Precision/SettlingLost", true);
      }
      return false;
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    // Stop robot
    driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    
    Pose2d finalPose = poseEstimator.getEstimatedPose();
    double finalError = finalPose.getTranslation()
        .getDistance(targetPose.getTranslation());
    
    if (interrupted) {
      System.err.println("Precision approach interrupted!");
      SmartDashboard.putString("Precision/Status", "Interrupted");
    } else {
      SmartDashboard.putString("Precision/Status", "Complete");
    }
    
    SmartDashboard.putNumber("Precision/FinalError", finalError);
    
    Logger.recordOutput("Precision/Interrupted", interrupted);
    Logger.recordOutput("Precision/Complete", !interrupted);
    Logger.recordOutput("Precision/FinalError", finalError);
    Logger.recordOutput("Precision/FinalPose", finalPose);
    
    // Reset navigation phase (will be set by parent command if needed)
    if (robotState.getNavigationPhase() == RobotState.NavigationPhase.PRECISION ||
        robotState.getNavigationPhase() == RobotState.NavigationPhase.VISION_LOST) {
      robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
    }
  }
  
  private String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }
}

