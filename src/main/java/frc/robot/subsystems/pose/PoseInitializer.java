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
import frc.robot.util.SmartLogger;
import org.littletonrobotics.junction.Logger;

// Determines initialization mode: COMP_SEED (seed Quest to known start) vs SHOP_RESUME (use Quest's existing tracking)
public class PoseInitializer {
  
  public enum InitializationState {
    WAITING,        // Not yet initialized
    INITIALIZED,    // Successfully initialized (either mode)
    FALLBACK_USED   // Fallback used (not currently implemented)
  }
  
  public static class InitResult {
    public final Pose2d pose;
    public final boolean shouldSeedQuest;
    public final String reason;
    
    public InitResult(Pose2d pose, boolean shouldSeedQuest, String reason) {
      this.pose = pose;
      this.shouldSeedQuest = shouldSeedQuest;
      this.reason = reason;
    }
  }
  
  private final QuestNavSubsystem questNavSubsystem;
  private final Timer initWaitTimer = new Timer();
  
  private InitializationState initState = InitializationState.WAITING;
  private SendableChooser<Command> autoChooser;
  private boolean noPoseWarningShown = false;

  private static final double FIELD_LENGTH_METERS = Units.feetToMeters(54.0);
  private static final double FIELD_WIDTH_METERS = Units.feetToMeters(27.0);
  private static final double FIELD_MARGIN_METERS = 0.3;
  private static final double MAX_SANE_POSE_MAGNITUDE = 100.0; // Sanity check for unanchored poses
  
  public PoseInitializer(QuestNavSubsystem questNavSubsystem) {
    this.questNavSubsystem = questNavSubsystem;
    initWaitTimer.start();
  }
  
  public void setAutoChooser(SendableChooser<Command> autoChooser) {
    this.autoChooser = autoChooser;
  }
  
  public void updateReadiness() {
    Pose2d questNavPose = questNavSubsystem.getRobotPose().orElse(null);
    boolean hasQuestNavPose = (questNavPose != null);
    boolean isFMSAttached = DriverStation.isFMSAttached();
    
    if (hasQuestNavPose) {
      if (isFMSAttached) {
        SmartDashboard.putString("Pose/InitStatus", "QuestNav ready - FIELD ALIGNED");
        SmartDashboard.putBoolean("Pose/ReadyToEnable", true);
        SmartDashboard.putBoolean("Pose/FieldAligned", true);
        Logger.recordOutput("PoseEstimator/Readiness/FieldAligned", true);
      } else {
        SmartDashboard.putString("Pose/InitStatus", "QuestNav ready - TELEOP ONLY (not field-aligned)");
        SmartDashboard.putBoolean("Pose/ReadyToEnable", true);
        SmartDashboard.putBoolean("Pose/FieldAligned", false);
        Logger.recordOutput("PoseEstimator/Readiness/FieldAligned", false);
      }
    } else {
      SmartDashboard.putString("Pose/InitStatus", "MANUAL RESET REQUIRED");
      SmartDashboard.putBoolean("Pose/ReadyToEnable", false);
      SmartDashboard.putBoolean("Pose/FieldAligned", false);
      Logger.recordOutput("PoseEstimator/Readiness/FieldAligned", false);
    }
    
    SmartDashboard.putBoolean("Vision/MultiTagReady", false);
    SmartDashboard.putBoolean("Vision/SingleTagReady", false);
    SmartDashboard.putBoolean("QuestNav/Ready", hasQuestNavPose);
    
    Logger.recordOutput("PoseEstimator/Readiness/VisionDisabled", true);
    Logger.recordOutput("PoseEstimator/Readiness/QuestNav", hasQuestNavPose);
  }
  
  public InitResult attemptInitialization() {
    // === COMP_SEED MODE: Disabled + FMS attached (prevents mid-auto re-init) ===
    if (DriverStation.isDisabled() && DriverStation.isFMSAttached()) {
      
      Pose2d autoStartPose = getExpectedAutoStartPose();
      
      if (autoStartPose != null && isWithinField(autoStartPose.getTranslation())) {
        initState = InitializationState.INITIALIZED;
        
        double waitTime = initWaitTimer.get();
        Logger.recordOutput("PoseEstimator/InitWaitSeconds", waitTime);
        
        SmartLogger.logConsole("=== COMP_SEED MODE ===");
        SmartLogger.logConsole("Auto: " + (autoChooser != null ? autoChooser.getSelected().getName() : "Unknown"));
        SmartLogger.logConsole("Starting pose: " + formatPose(autoStartPose));
        SmartLogger.logConsole("Quest will be SEEDED to this pose");
        SmartLogger.logConsole("Wait time: " + String.format("%.2fs", waitTime));
        SmartLogger.logConsole("====================");
        
        Logger.recordOutput("PoseEstimator/InitializedFromAuto", true);
        Logger.recordOutput("PoseEstimator/InitMode", "COMP_SEED");
        
        return new InitResult(autoStartPose, true, "COMP_SEED: Auto start pose from chooser");
      }
    }
    
    // === SHOP_RESUME MODE: Teleop or disabled without FMS ===
    // Pose is UNANCHORED - skip field bounds check, just sanity-check
    if (!DriverStation.isFMSAttached()) {
      
      Pose2d questNavPose = questNavSubsystem.getRobotPose().orElse(null);
      if (questNavPose != null && isSanePose(questNavPose)) {
        initState = InitializationState.INITIALIZED;
        
        double waitTime = initWaitTimer.get();
        Logger.recordOutput("PoseEstimator/InitWaitSeconds", waitTime);
        
        SmartLogger.logConsole("=== SHOP_RESUME MODE ===");
        SmartLogger.logConsole("Using Quest's existing tracking (NOT SEEDED)");
        SmartLogger.logConsole("Pose: " + formatPose(questNavPose));
        SmartLogger.logConsole("Wait time: " + String.format("%.2fs", waitTime));
        SmartLogger.logConsoleError("WARNING: Pose is in Quest's own frame (NOT field-aligned!)");
        SmartLogger.logConsoleError("Do NOT use for autonomous - teleop practice only!");
        SmartLogger.logConsole("====================");
        
        Logger.recordOutput("PoseEstimator/InitializedViaQuestNav", true);
        Logger.recordOutput("PoseEstimator/InitMode", "SHOP_RESUME");
        Logger.recordOutput("PoseEstimator/UnanchoredFrame", true);
        
        return new InitResult(questNavPose, false, "SHOP_RESUME: Quest existing tracking (UNANCHORED - teleop only)");
      }
    }
    
    SmartDashboard.putString("Pose/InitMethod", "BLOCKED - Quest not tracking");
    
    if (!noPoseWarningShown) {
      SmartLogger.logConsoleError("Cannot initialize: Quest not tracking");
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
  
  // Field bounds check (only for COMP_SEED mode - known field frame)
  private boolean isWithinField(Translation2d point) {
    return point.getX() > FIELD_MARGIN_METERS &&
           point.getX() < FIELD_LENGTH_METERS - FIELD_MARGIN_METERS &&
           point.getY() > FIELD_MARGIN_METERS &&
           point.getY() < FIELD_WIDTH_METERS - FIELD_MARGIN_METERS;
  }
  
  // Sanity check for unanchored poses (SHOP_RESUME mode)
  // Rejects NaN/Inf and absurdly large values (>100m suggests Quest error)
  private boolean isSanePose(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getRotation().getRadians();
    
    // Check for NaN/Inf
    if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(theta)) {
      Logger.recordOutput("PoseInitializer/InsanePose", "NaN/Inf detected");
      return false;
    }
    
    // Check for absurdly large values (Quest bug/corruption)
    double magnitude = Math.hypot(x, y);
    if (magnitude > MAX_SANE_POSE_MAGNITUDE) {
      Logger.recordOutput("PoseInitializer/InsanePose", 
          String.format("Magnitude too large: %.2fm", magnitude));
      return false;
    }
    
    return true;
  }
  
  public boolean isInitialized() {
    return initState == InitializationState.INITIALIZED || 
           initState == InitializationState.FALLBACK_USED;
  }
  
  public InitializationState getInitState() {
    return initState;
  }
  
  public void setInitState(InitializationState state) {
    this.initState = state;
  }
  
  public double getWaitTime() {
    return initWaitTimer.get();
  }
  
  private String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", 
        pose.getX(), 
        pose.getY(), 
        pose.getRotation().getDegrees());
  }
}