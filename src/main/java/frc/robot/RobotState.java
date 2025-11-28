package frc.robot;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;

// Global robot state tracker - shared across all subsystems to coordinate behavior
// Use this to check what mode the robot is in or get the current field position
public class RobotState {
  // Robot operating modes - set by Robot.java based on driver station
  public enum Mode {
    DISABLED, // Robot is powered but not moving (safe to work on)
    TELEOP,   // Driver control with joysticks
    AUTO,     // Running autonomous routine
    TEST      // Test mode for manual subsystem testing
  }

  // Driver assistance modes - used by vision and auto-alignment features
  public enum AssistMode {
    NONE,                   // Manual control only
    ALIGN_TO_TARGET,        // Auto-rotate to face AprilTag
    AUTO_SCORE,             // Full auto-score sequence
    FOLLOW_PREPLANNED_PATH  // Following a PathPlanner path
  }

  private Mode mode = Mode.DISABLED; // Start disabled for safety
  private AssistMode assistMode = AssistMode.NONE; // No assist by default
  private boolean enabled = false; // Tracks if robot is enabled (auto/teleop/test)
  private boolean sysIdMode = false; // Special mode for system characterization (disables vision)
  private Pose2d robotPose = new Pose2d(); // Current field position (updated by PoseEstimator)

  // Update robot's field position - called by PoseEstimatorSubsystem
  public void setRobotPose(Pose2d pose) {
    this.robotPose = pose;
  }

  // Get current field position (X, Y, rotation)
  public Pose2d getRobotPose() {
    return robotPose;
  }

  // Set robot mode (called by Robot.java on mode transitions)
  public void setMode(Mode mode) {
    this.mode = mode;
    log(); // Log to AdvantageScope for analysis
  }

  public Mode getMode() {
    return mode;
  }

  // Set driver assistance mode (used by vision alignment commands)
  public void setAssistMode(AssistMode assistMode) {
    this.assistMode = assistMode;
    log();
  }

  public AssistMode getAssistMode() {
    return assistMode;
  }

  // Set enabled state (true = auto/teleop/test, false = disabled)
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    log();
  }

  public boolean isEnabled() {
    return enabled;
  }

  // SysID mode check - when true, vision updates are disabled for clean characterization
  public boolean isSysIdMode() {
    return sysIdMode;
  }

  // Enable/disable SysID mode - prevents vision from interfering with motor tests
  public void setSysIdMode(boolean sysIdMode) {
    this.sysIdMode = sysIdMode;
    if (sysIdMode) {
      System.out.println("=== SYSID MODE ENABLED ===");
      System.out.println("Vision updates DISABLED");
      System.out.println("Run SysID routines now");
    } else {
      System.out.println("=== SYSID MODE DISABLED ===");
    }
  }

  // Log state changes to AdvantageScope for debugging
  private void log() {
    Logger.recordOutput("RobotState/Mode", mode.toString());
    Logger.recordOutput("RobotState/AssistMode", assistMode.toString());
    Logger.recordOutput("RobotState/Enabled", enabled);
  }
}
