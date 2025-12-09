package frc.robot.subsystems.pose;

import static frc.robot.Constants.StartingPositions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.QuestNavSubsystem;
import org.littletonrobotics.junction.Logger;

public class PoseInitializer {
  
  public enum InitializationState {
    WAITING_FOR_VISION,
    VISION_INITIALIZED,
    FALLBACK_USED
  }
  
  private final QuestNavSubsystem questNavSubsystem;
  private final QuestNavFusion questNavFusion;
  private final Timer visionWaitTimer = new Timer();
  
  private InitializationState initState = InitializationState.WAITING_FOR_VISION;
  private SendableChooser<Command> autoChooser;
  private boolean initializedFromAuto = false;
  private boolean noPoseWarningShown = false;

  private static final double FIELD_LENGTH_METERS = Units.feetToMeters(54.0);
  private static final double FIELD_WIDTH_METERS = Units.feetToMeters(27.0);
  private static final double FIELD_MARGIN_METERS = 0.3;
  
  public PoseInitializer(QuestNavSubsystem questNavSubsystem, QuestNavFusion questNavFusion) {
    this.questNavSubsystem = questNavSubsystem;
    this.questNavFusion = questNavFusion;
    visionWaitTimer.start();
  }
  
  public void setAutoChooser(SendableChooser<Command> autoChooser) {
    this.autoChooser = autoChooser;
  }
  
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
  
  public Pose2d attemptInitialization() {
    if (DriverStation.isAutonomousEnabled() || 
        (DriverStation.isDisabled() && DriverStation.isFMSAttached())) {
      
      Pose2d autoStartPose = getExpectedAutoStartPose();
      
      if (autoStartPose != null && isWithinField(autoStartPose.getTranslation())) {
        initState = InitializationState.VISION_INITIALIZED;
        initializedFromAuto = true;
        
        System.out.println("=== INITIALIZED FROM AUTO STARTING POSE ===");
        System.out.println("Auto: " + (autoChooser != null ? autoChooser.getSelected().getName() : "Unknown"));
        System.out.println("Starting pose: " + autoStartPose);
        System.out.println("QuestNav will track from here with INITIAL HIGH TRUST");
        
        SmartDashboard.putString("Pose/InitMethod", "Auto Start + QuestNav Initial Alignment");
        Logger.recordOutput("PoseEstimator/InitializedFromAuto", true);
        Logger.recordOutput("PoseEstimator/QuestNav/InitialAlignmentUsed", true);
        
        return autoStartPose;
      }
    }
    
    if (DriverStation.isTeleopEnabled() || 
        (DriverStation.isDisabled() && !DriverStation.isFMSAttached())) {
      
      Pose2d questNavPose = questNavSubsystem.getRobotPose().orElse(null);
      if (questNavPose != null && isWithinField(questNavPose.getTranslation())) {
        initState = InitializationState.VISION_INITIALIZED;
        
        System.out.println("=== INITIALIZED FROM QUESTNAV (TELEOP START) ===");
        System.out.println("TESTING: Vision disabled - QuestNav-only mode with INITIAL HIGH TRUST");
        System.out.println("Pose: " + questNavPose);
        
        SmartDashboard.putString("Pose/InitMethod", "QuestNav Initial Alignment (Vision disabled)");
        Logger.recordOutput("PoseEstimator/InitializedViaQuestNav", true);
        Logger.recordOutput("PoseEstimator/QuestNav/InitialAlignmentUsed", true);
        
        return questNavPose;
      }
    }
    
    SmartDashboard.putString("Pose/InitMethod", "BLOCKED - MANUAL RESET REQUIRED (QuestNav-only mode)");
    
    if (!noPoseWarningShown) {
      Logger.recordOutput("PoseEstimator/NoPoseWarningShown", true);
      noPoseWarningShown = true;
    }
    
    return null;
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
          Logger.recordOutput("PoseInitializer/UnknownAuto", autoName);
          return null;
      }
    } catch (Exception e) {
      Logger.recordOutput("PoseInitializer/GetPoseError", e.getMessage());
      return null;
    }
  }
  
  private boolean isWithinField(Translation2d point) {
    return point.getX() > FIELD_MARGIN_METERS &&
           point.getX() < FIELD_LENGTH_METERS - FIELD_MARGIN_METERS &&
           point.getY() > FIELD_MARGIN_METERS &&
           point.getY() < FIELD_WIDTH_METERS - FIELD_MARGIN_METERS;
  }
  
  public boolean isInitialized() {
    return initState == InitializationState.VISION_INITIALIZED || 
           initState == InitializationState.FALLBACK_USED;
  }
  
  public InitializationState getInitState() {
    return initState;
  }
  
  public void setInitState(InitializationState state) {
    this.initState = state;
  }
  
  public double getWaitTime() {
    return visionWaitTimer.get();
  }
}