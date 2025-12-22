package frc.robot;

import frc.robot.util.SmartLogger;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.LinkedList;
import java.util.Queue;

// Global robot state tracker - coordinates subsystem states and robot intents
// Single source of truth for robot mode, navigation phase, and mechanism states (2026+)
public class RobotState {
  
  // Game state (FMS-driven)
  public enum Mode {
    DISABLED, 
    ENABLED_TELEOP, 
    ENABLED_AUTO, 
    TEST
  }
  private Mode mode = Mode.DISABLED;
  private boolean enabled = false;
  private boolean sysIdMode = false;
  
  // Robot intent (high-level actions)
  public enum RobotIntent {
    IDLE, 
    NAVIGATING
    // TODO 2026: Add INTAKE_FLOOR, SCORE_HIGH, CLIMB, etc.
  }
  private RobotIntent currentIntent = RobotIntent.IDLE;
  private final Queue<RobotIntent> intentQueue = new LinkedList<>();
  
  // Navigation state (active - used by SmartDrive)
  public enum NavigationPhase {
    NONE,           // Not navigating
    FAST_APPROACH,  // PathPlanner pathfinding
    PRECISION_PATH, // AutoPilot precision
    LOCKED          // Navigation complete, wheels locked
  }
  private NavigationPhase navigationPhase = NavigationPhase.NONE;
  
  // Field position
  private Pose2d robotPose = new Pose2d();
  
  // PUBLIC API
  public void requestIntent(RobotIntent intent) {
    currentIntent = intent;
    SmartLogger.logReplay("RobotState/Intent", intent.toString());
  }
  
  public RobotIntent getCurrentIntent() { return currentIntent; }
  
  public void setNavigationPhase(NavigationPhase navPhase) {
    this.navigationPhase = navPhase;
    SmartLogger.logReplay("RobotState/NavigationPhase", navPhase.toString());
  }
  
  public NavigationPhase getNavigationPhase() { return navigationPhase; }
  
  public void setRobotPose(Pose2d pose) { this.robotPose = pose; }
  public Pose2d getRobotPose() { return robotPose; }
  
  public void setMode(Mode mode) {
    this.mode = mode;
    SmartLogger.logReplay("RobotState/Mode", mode.toString());
    SmartLogger.logReplay("RobotState/Enabled", enabled);
  }
  
  public Mode getMode() { return mode; }
  
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    SmartLogger.logReplay("RobotState/Enabled", enabled);
  }
  
  public boolean isEnabled() { return enabled; }
  
  public void setSysIdMode(boolean sysIdMode) {
    this.sysIdMode = sysIdMode;
    if (sysIdMode) {
      SmartLogger.logConsole("SysId mode enabled - vision updates disabled", "SysId Mode");
    }
  }
  
  public boolean isSysIdMode() { return sysIdMode; }
}