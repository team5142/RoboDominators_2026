package frc.robot.subsystems.pose;

import static frc.robot.Constants.StartingPositions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.QuestNavSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Handles pose estimation initialization logic
 * 
 * Responsibilities:
 * - Determine readiness to enable (do we have a pose?)
 * - Initialize from auto starting pose
 * - Initialize from QuestNav in teleop
 * - Provide user feedback on initialization status
 */
public class PoseInitializer {
  
  public enum InitializationState {
    WAITING_FOR_VISION,
    VISION_INITIALIZED,
    FALLBACK_USED
  }
  
  private final QuestNavSubsystem questNavSubsystem;
  private final Timer visionWaitTimer = new Timer();
  
  private InitializationState initState = InitializationState.WAITING_FOR_VISION;
  private SendableChooser<Command> autoChooser;
  private boolean initializedFromAuto = false;
  
  public PoseInitializer(QuestNavSubsystem questNavSubsystem) {
    this.questNavSubsystem = questNavSubsystem;
    visionWaitTimer.start();
  }
  
  public void setAutoChooser(SendableChooser<Command> autoChooser) {
    this.autoChooser = autoChooser;
  }
  
  /**
   * Update readiness indicators on SmartDashboard
   * Called every cycle while disabled
   */
  public void updateReadiness() {
    Pose2d questNavPose = questNavSubsystem.getRobotPose().orElse(null);
    boolean hasQuestNavPose = (questNavPose != null);
    
    if (hasQuestNavPose) {
      SmartDashboard.putString("Pose/InitStatus", "QuestNav ONLY (Vision disabled for testing)");
      SmartDashboard.putBoolean("Pose/ReadyToEnable", true);
    } else {
      SmartDashboard.putString("Pose/InitStatus", "MANUAL RESET REQUIRED (Vision disabled)");
      SmartDashboard.putBoolean("Pose/ReadyToEnable", false);
    }
    
    SmartDashboard.putBoolean("Vision/MultiTagReady", false);
    SmartDashboard.putBoolean("Vision/SingleTagReady", false);
    SmartDashboard.putBoolean("QuestNav/Ready", hasQuestNavPose);
    
    Logger.recordOutput("PoseEstimator/Readiness/VisionDisabled", true);
    Logger.recordOutput("PoseEstimator/Readiness/QuestNav", hasQuestNavPose);
  }
  
  /**
   * Attempt to initialize pose estimate
   * Returns pose to initialize to, or null if can't initialize
   */
  public Pose2d attemptInitialization() {
    // AUTO MODE: Use starting pose from selected auto
    if (DriverStation.isAutonomousEnabled() || 
        (DriverStation.isDisabled() && DriverStation.isFMSAttached())) {
      
      Pose2d autoStartPose = getExpectedAutoStartPose();
      
      if (autoStartPose != null) {
        initState = InitializationState.VISION_INITIALIZED;
        initializedFromAuto = true;
        
        System.out.println("=== INITIALIZED FROM AUTO STARTING POSE ===");
        System.out.println("Auto: " + (autoChooser != null ? autoChooser.getSelected().getName() : "Unknown"));
        System.out.println("Starting pose: " + autoStartPose);
        System.out.println("QuestNav will track from here");
        
        SmartDashboard.putString("Pose/InitMethod", "Auto Starting Pose (QuestNav-only)");
        Logger.recordOutput("PoseEstimator/InitializedFromAuto", true);
        
        return autoStartPose;
      }
    }
    
    // TELEOP MODE: Use QuestNav current pose
    if (DriverStation.isTeleopEnabled() || 
        (DriverStation.isDisabled() && !DriverStation.isFMSAttached())) {
      
      Pose2d questNavPose = questNavSubsystem.getRobotPose().orElse(null);
      if (questNavPose != null) {
        initState = InitializationState.VISION_INITIALIZED;
        
        System.out.println("=== INITIALIZED FROM QUESTNAV (TELEOP START) ===");
        System.out.println("TESTING: Vision disabled - QuestNav-only mode");
        System.out.println("Pose: " + questNavPose);
        
        SmartDashboard.putString("Pose/InitMethod", "QuestNav ONLY (Vision disabled)");
        Logger.recordOutput("PoseEstimator/InitializedViaQuestNav", true);
        
        return questNavPose;
      }
    }
    
    // FAILED: No pose source available
    SmartDashboard.putString("Pose/InitMethod", "BLOCKED - MANUAL RESET REQUIRED (QuestNav-only mode)");
    DriverStation.reportError("NO POSE - Press START to set position (Vision disabled)", false);
    
    return null;
  }
  
  /**
   * Get expected starting pose for currently selected auto
   */
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
  
  public InitializationState getInitState() {
    return initState;
  }
  
  public void setInitState(InitializationState state) {
    this.initState = state;
  }
  
  public boolean isInitialized() {
    return initState != InitializationState.WAITING_FOR_VISION;
  }
  
  public boolean wasInitializedFromAuto() {
    return initializedFromAuto;
  }
  
  public double getWaitTime() {
    return initState == InitializationState.WAITING_FOR_VISION ? visionWaitTimer.get() : 0.0;
  }
}
