package frc.robot.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.util.SmartLogger;
import org.littletonrobotics.junction.Logger;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

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

  // Cache last JSON parse result - avoids file I/O on every periodic loop
  private String cachedAutoName = null;
  private Pose2d cachedAutoStartPose = null;

  private static final double FIELD_LENGTH_METERS = Constants.Field.FIELD_LENGTH_METERS;
  private static final double FIELD_WIDTH_METERS = Constants.Field.FIELD_WIDTH_METERS;
  private static final double FIELD_MARGIN_METERS = 0.3;
  private static final double MAX_SANE_POSE_MAGNITUDE = 100.0; // Sanity check for unanchored poses
  
  public PoseInitializer(QuestNavSubsystem questNavSubsystem) {
    this.questNavSubsystem = questNavSubsystem;
    initWaitTimer.start();
  }
  
  public void setAutoChooser(SendableChooser<Command> autoChooser) {
    this.autoChooser = autoChooser;
  }

  // Returns the currently selected auto command's name, or null if nothing is selected.
  public String getSelectedAutoName() {
    if (autoChooser == null) return null;
    Command selected = autoChooser.getSelected();
    return selected != null ? selected.getName() : null;
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
    if (DriverStation.isDisabled()) {
      Pose2d autoStartPose = getExpectedAutoStartPose();
      if (autoStartPose != null && isWithinField(autoStartPose.getTranslation())) {
        initState = InitializationState.INITIALIZED;

        double waitTime = initWaitTimer.get();
        Logger.recordOutput("PoseEstimator/InitWaitSeconds", waitTime);

        Logger.recordOutput("PoseEstimator/InitializedFromAuto", true);
        Logger.recordOutput("PoseEstimator/InitMode",
            DriverStation.isFMSAttached() ? "COMP_SEED" : "PRACTICE_AUTO_SEED");

        return new InitResult(
            autoStartPose,
            true,
            DriverStation.isFMSAttached()
                ? "COMP_SEED: Auto start pose from chooser"
                : "PRACTICE_AUTO_SEED: Auto start pose from chooser");
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
        
        SmartLogger.logConsole("SHOP_RESUME: Quest tracking unanchored, pose: " + SmartLogger.formatPose(questNavPose));
        SmartLogger.logConsoleError("WARNING: Not field-aligned - teleop practice only!");
        
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
  
  public Pose2d getStartPoseForAutoName(String autoName) {
    if (autoName == null || autoName.isEmpty()) return null;

    // Return cached result if same auto and same alliance is requested again (avoids file I/O every loop)
    boolean isRedNow = DriverStation.getAlliance()
        .map(a -> a == DriverStation.Alliance.Red).orElse(false);
    String cacheKey = autoName + (isRedNow ? "_red" : "_blue");
    if (cacheKey.equals(cachedAutoName)) return cachedAutoStartPose;

    try {
      // Read the .auto file to find first path name
      File autoFile = new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto");
      if (!autoFile.exists()) {
        Logger.recordOutput("PoseInitializer/UnknownAuto", autoName);
        cachedAutoName = cacheKey;
        cachedAutoStartPose = null;
        return null;
      }

      JSONParser parser = new JSONParser();
      JSONObject autoJson = (JSONObject) parser.parse(new BufferedReader(new FileReader(autoFile)));
      String firstPathName = findFirstPathName(autoJson);
      if (firstPathName == null) {
        Logger.recordOutput("PoseInitializer/AutoNoPath", autoName);
        cachedAutoName = cacheKey;
        cachedAutoStartPose = null;
        return null;
      }

      // Read the .path file and extract first waypoint anchor
      File pathFile = new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + firstPathName + ".path");
      if (!pathFile.exists()) {
        Logger.recordOutput("PoseInitializer/PathNotFound", firstPathName);
        cachedAutoName = cacheKey;
        cachedAutoStartPose = null;
        return null;
      }

      JSONObject pathJson = (JSONObject) parser.parse(new BufferedReader(new FileReader(pathFile)));
      JSONArray waypoints = (JSONArray) pathJson.get("waypoints");
      if (waypoints == null || waypoints.isEmpty()) return null;

      JSONObject firstWaypoint = (JSONObject) waypoints.get(0);
      JSONObject anchor = (JSONObject) firstWaypoint.get("anchor");
      if (anchor == null) return null;

      double x = ((Number) anchor.get("x")).doubleValue();
      double y = ((Number) anchor.get("y")).doubleValue();

      // Rotation from ideal heading in path file (null = 0)
      Object idealRotation = pathJson.get("idealStartingState");
      double rotDeg = 0.0;
      if (idealRotation instanceof JSONObject) {
        Object rot = ((JSONObject) idealRotation).get("rotation");
        if (rot instanceof Number) rotDeg = ((Number) rot).doubleValue();
      }

      Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(rotDeg));

      // Paths are authored on blue side. Rotate 180 deg around field center for red.
      if (isRedNow) {
        pose = new Pose2d(
            FIELD_LENGTH_METERS - x,
            FIELD_WIDTH_METERS - y,
            Rotation2d.fromDegrees(rotDeg + 180.0));
      }

      Logger.recordOutput("PoseInitializer/AutoStartPose", pose);
      Logger.recordOutput("PoseInitializer/AutoStartPoseFlipped", isRedNow);
      cachedAutoName = cacheKey;
      cachedAutoStartPose = pose;
      return pose;
    } catch (Exception e) {
      Logger.recordOutput("PoseInitializer/AutoStartPoseError", e.getMessage());
      cachedAutoName = cacheKey;
      cachedAutoStartPose = null;
      return null;
    }
  }

  // Recursively finds the first path command name in an auto command tree
  private String findFirstPathName(JSONObject command) {
    if (command == null) return null;
    String type = (String) command.get("type");
    JSONObject data = (JSONObject) command.get("data");
    if ("path".equals(type) && data != null) {
      return (String) data.get("pathName");
    }
    if (data != null) {
      JSONArray commands = (JSONArray) data.get("commands");
      if (commands != null) {
        for (Object cmd : commands) {
          String found = findFirstPathName((JSONObject) cmd);
          if (found != null) return found;
        }
      }
    }
    // Top-level auto file has command at root
    JSONObject rootCommand = (JSONObject) command.get("command");
    if (rootCommand != null) return findFirstPathName(rootCommand);
    return null;
  }

  private Pose2d getExpectedAutoStartPose() {
    if (autoChooser == null) return null;

    try {
      Command selectedAuto = autoChooser.getSelected();
      if (selectedAuto == null) return null;

      String autoName = selectedAuto.getName();
      Pose2d pose = getStartPoseForAutoName(autoName);

      if (pose == null) {
        Logger.recordOutput("PoseInitializer/UnknownAuto", autoName);
      }

      return pose;
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
}