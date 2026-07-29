package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.SmartLogger;

// Global robot state - single source of truth for what every mechanism is doing.
// Subsystems write their state here each loop so other subsystems can read it
// without needing a direct reference to each other.
public class RobotState {

  // FMS-driven mode - do not modify
  public enum Mode { DISABLED, ENABLED_TELEOP, ENABLED_AUTO, TEST }
  private Mode mode = Mode.DISABLED;
  private boolean enabled = false;

  private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

  // Navigation phase - do not modify
  public enum NavigationPhase {
    NONE,
    FAST_APPROACH,
    PRECISION_PATH,
    LOCKED
  }
  private NavigationPhase navigationPhase = NavigationPhase.NONE;

  // Field position - written by PoseEstimatorSubsystem
  private Pose2d robotPose = new Pose2d();

  // Spindexer spin direction - provided as a reference for how enums work in RobotState.
  // Notice: enum declared, field declared with a default value, getter and setter below.
  public enum SpindexerState {
    STOPPED,
    FORWARD,
    REVERSE
  }
  private SpindexerState spindexerState = SpindexerState.STOPPED;

  /*
   * TASK 19a - Add IntakePosition Enum and State
   * -----------------------------------------------------------------------
   * The intake arm has several positions it can be in. We track this so
   * other subsystems (like auto commands) can read the arm state without
   * directly calling IntakeSubsystem.
   *
   * An enum is a fixed set of named values. Example of a simple enum:
   *   public enum Color { RED, GREEN, BLUE }
   *
   * The intake arm needs these positions:
   *   HOMING        - moving inward to find the limit switch on first enable
   *   HOMING_FAILED - stalled before reaching the limit switch
   *   RETRACTED     - fully in, limit switch triggered, encoder zeroed
   *   EXTENDING     - currently moving outward
   *   EXTENDED      - reached full extension
   *   RETRACTING    - currently moving inward
   *
   * Steps:
   *   1. Declare the IntakePosition enum with those six values.
   *   2. Declare a private IntakePosition field with a default of RETRACTED.
   *   3. Write a setter: check if the value changed, update the field,
   *      call SmartLogger.logReplay("RobotState/Intake/Position", pos.toString())
   *   4. Write a getter that returns the current IntakePosition.
   *
   * Look at the SpindexerState block above - the pattern is identical.
   * Write it out from scratch rather than copying.
   *
   * When done: compile and move to Task 19b in RobotContainer.java (or continue to Task 20a here).
   * -----------------------------------------------------------------------
   */

  /*
   * TASK 20a - Add IntakeRollerState Enum and State
   * -----------------------------------------------------------------------
   * The intake rollers (separate from the arm) can spin in, spin out, or stop.
   *
   * Values needed: STOPPED, INTAKING, REVERSING
   *
   * Same pattern as Task 19a - enum, field, setter with log, getter.
   * Log key: "RobotState/Intake/RollerState"
   *
   * When done: compile and move to Task 20b in DriveWithJoysticks.java (or continue to Task 21a here).
   * -----------------------------------------------------------------------
   */

  /*
   * TASK 21a - Add SingulatorState Enum and State
   * -----------------------------------------------------------------------
   * The Singulator feeds balls one at a time. It can be paused, feeding, or reversing.
   *
   * Values needed: PAUSED, FEEDING, REVERSING
   *
   * Same pattern again. You should be able to write this one without looking
   * at the previous tasks.
   * Log key: "RobotState/SingulatorState"
   *
   * Also add a boolean field for the beam break sensor:
   *   - private boolean singulatorBeamBreak = false;
   *   - setter: setSingulatorBeamBreak(boolean value) - logs to "RobotState/SingulatorBeamBreak"
   *   - getter: getSingulatorBeamBreak() returns boolean
   *
   * When done: compile, then move to Task 21b in DriveWithJoysticks.java (or continue to Task 22a in IntakeSubsystem.java).
   * -----------------------------------------------------------------------
   */

  // ---- Mode ---- (do not modify)

  public void setMode(Mode mode) {
    if (this.mode == mode) return;
    this.mode = mode;
    SmartLogger.logReplay("RobotState/Mode", mode.toString());
  }
  public Mode getMode() { return mode; }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    SmartLogger.logReplay("RobotState/Enabled", enabled);
  }
  public boolean isEnabled() { return enabled; }

  // ---- Alliance ---- (do not modify)

  public void setAlliance(DriverStation.Alliance alliance) {
    if (this.alliance == alliance) return;
    this.alliance = alliance;
    SmartLogger.logReplay("RobotState/Alliance", alliance.toString());
  }
  public DriverStation.Alliance getAlliance() { return alliance; }

  // ---- Navigation ---- (do not modify)

  public void setNavigationPhase(NavigationPhase phase) {
    if (this.navigationPhase == phase) return;
    this.navigationPhase = phase;
    SmartLogger.logReplay("RobotState/NavigationPhase", phase.toString());
  }
  public NavigationPhase getNavigationPhase() { return navigationPhase; }

  // ---- Pose ---- (do not modify)

  public void setRobotPose(Pose2d pose) { this.robotPose = pose; }
  public Pose2d getRobotPose() { return robotPose; }

  // ---- Spindexer ---- (provided as reference)

  public void setSpindexerState(SpindexerState state) {
    if (this.spindexerState == state) return;
    this.spindexerState = state;
    SmartLogger.logReplay("RobotState/SpindexerState", state.toString());
  }
  public SpindexerState getSpindexerState() { return spindexerState; }
}
