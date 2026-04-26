package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.SmartLogger;

// Global robot state - single source of truth for mechanism states and robot mode.
// Subsystems write their state here each loop so other subsystems can read it without
// creating direct dependencies between each other.
public class RobotState {

  // FMS-driven mode
  public enum Mode { DISABLED, ENABLED_TELEOP, ENABLED_AUTO, TEST }
  private Mode mode = Mode.DISABLED;
  private boolean enabled = false;

  private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

  // Navigation phase - written by SmartDriveToPosition, read by PoseEstimatorSubsystem
  public enum NavigationPhase {
    NONE,           // not navigating
    FAST_APPROACH,  // PathPlanner pathfinding
    PRECISION_PATH, // AutoPilot precision
    LOCKED          // navigation complete, wheels locked
  }
  private NavigationPhase navigationPhase = NavigationPhase.NONE;

  // Field position - written by PoseEstimatorSubsystem
  private Pose2d robotPose = new Pose2d();

  // Intake arm position
  public enum IntakePosition {
    HOMING,        // moving to find limit switch on first enable
    HOMING_FAILED, // stalled before finding limit switch
    RETRACTED,     // fully in, limit switch triggered, encoder zeroed
    EXTENDING,     // moving outward
    EXTENDED,      // at full extension
    RETRACTING     // moving inward
  }

  // Intake roller direction
  public enum IntakeRollerState { STOPPED, INTAKING, REVERSING }

  // Spindexer spin direction
  public enum SpindexerState {
    STOPPED,
    FORWARD,  // feeding toward singulator
    REVERSE   // unjam pulse
  }

  // Singulator feed state
  public enum SingulatorState {
    PAUSED,    // motor stopped
    FEEDING,   // running toward flywheels
    REVERSING  // clearing a jam
  }

  // --- Mechanism state fields ---
  // Default RETRACTED so extend/retract work even if homing is skipped
  private IntakePosition intakePosition = IntakePosition.RETRACTED;
  private IntakeRollerState intakeRollerState = IntakeRollerState.STOPPED;
  private boolean intakeLimitSwitch = false;

  private SpindexerState spindexerState = SpindexerState.STOPPED;

  private SingulatorState singulatorState = SingulatorState.PAUSED;
  private boolean singulatorBeamBreak = false;

  // ---- Mode ----

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

  // ---- Alliance ----

  public void setAlliance(DriverStation.Alliance alliance) {
    if (this.alliance == alliance) return;
    this.alliance = alliance;
    SmartLogger.logReplay("RobotState/Alliance", alliance.toString());
  }
  public DriverStation.Alliance getAlliance() { return alliance; }

  // ---- Navigation ----

  public void setNavigationPhase(NavigationPhase phase) {
    if (this.navigationPhase == phase) return;
    this.navigationPhase = phase;
    SmartLogger.logReplay("RobotState/NavigationPhase", phase.toString());
  }
  public NavigationPhase getNavigationPhase() { return navigationPhase; }

  // ---- Pose ----

  public void setRobotPose(Pose2d pose) { this.robotPose = pose; }
  public Pose2d getRobotPose() { return robotPose; }

  // ---- Intake ----

  public void setIntakePosition(IntakePosition pos) {
    if (intakePosition == pos) return;
    intakePosition = pos;
    SmartLogger.logReplay("RobotState/Intake/Position", pos.toString());
  }
  public IntakePosition getIntakePosition() { return intakePosition; }

  public void setIntakeRollerState(IntakeRollerState state) {
    if (intakeRollerState == state) return;
    intakeRollerState = state;
    SmartLogger.logReplay("RobotState/Intake/RollerState", state.toString());
  }
  public IntakeRollerState getIntakeRollerState() { return intakeRollerState; }

  public void setIntakeLimitSwitch(boolean pressed) {
    if (intakeLimitSwitch == pressed) return;
    intakeLimitSwitch = pressed;
    SmartLogger.logReplay("RobotState/Intake/LimitSwitch", pressed);
  }
  public boolean isIntakeFullyRetracted() { return intakeLimitSwitch; }

  // ---- Spindexer ----

  public void setSpindexerState(SpindexerState state) {
    if (this.spindexerState == state) return;
    this.spindexerState = state;
    SmartLogger.logReplay("RobotState/SpindexerState", state.toString());
  }
  public SpindexerState getSpindexerState() { return spindexerState; }

  // ---- Singulator ----

  public void setSingulatorState(SingulatorState state) {
    if (this.singulatorState == state) return;
    this.singulatorState = state;
    SmartLogger.logReplay("RobotState/SingulatorState", state.toString());
  }
  public SingulatorState getSingulatorState() { return singulatorState; }

  // True when a ball is present at the singulator beam break sensor
  public void setSingulatorBeamBreak(boolean value) {
    if (this.singulatorBeamBreak == value) return;
    this.singulatorBeamBreak = value;
    SmartLogger.logReplay("RobotState/SingulatorBeamBreak", value);
  }
  public boolean getSingulatorBeamBreak() { return singulatorBeamBreak; }
}