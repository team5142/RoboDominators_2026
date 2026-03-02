package frc.robot;

import frc.robot.util.MatchPhaseTracker;
import frc.robot.util.SmartLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

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
  private boolean operatorDriveLockout = false;
  private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

  // Match phase and hub active tracking
  private final MatchPhaseTracker matchPhaseTracker = new MatchPhaseTracker();
  
  // Robot intent (high-level actions)
  public enum RobotIntent {
    IDLE, 
    NAVIGATING
    // TODO 2026: Add INTAKE_FLOOR, SCORE_HIGH, CLIMB, etc.
  }
  private RobotIntent currentIntent = RobotIntent.IDLE;
  
  // Navigation state (active - used by SmartDrive)
  public enum NavigationPhase {
    NONE,           // Not navigating
    FAST_APPROACH,  // PathPlanner pathfinding
    PRECISION_PATH, // AutoPilot precision
    LOCKED          // Navigation complete, wheels locked
  }
  private NavigationPhase navigationPhase = NavigationPhase.NONE;

  // Mechanism states
  public enum TurretState {
    IDLE,
    ACTIVE
  }

  // Position of the intake arm (extension axis)
  public enum IntakePosition {
    HOMING,         // actively moving up to find limit switches on first enable
    HOMING_FAILED,  // homing stalled without finding both switches — arm is blocked
    RETRACTED,      // fully in, both limit switches triggered, encoder zeroed
    EXTENDING,      // moving outward
    EXTENDED,       // at full extension target rotation
    RETRACTING      // moving inward toward limit switches
  }

  // Spin direction of the intake rollers
  public enum IntakeRollerState {
    STOPPED,
    INTAKING,
    REVERSING
  }

  public enum ClimberState {
    IDLE,
    ACTIVE
  }

  // Spin state of the spindexer cone
  public enum SpindexerState {
    STOPPED,
    FORWARD,  // feeding balls toward singulator groove
    REVERSE   // agitator/unjam pulse
  }

  // State of the singulator feed motor
  public enum SingulatorState {
    PAUSED,    // holding — do not feed
    FEEDING,   // running toward flywheels
    REVERSING  // reverse to clear a jam
  }

  private TurretState turretState = TurretState.IDLE;
  private IntakePosition intakePosition = IntakePosition.HOMING;
  private IntakeRollerState intakeRollerState = IntakeRollerState.STOPPED;
  private ClimberState climberState = ClimberState.IDLE;
  private SpindexerState spindexerState = SpindexerState.STOPPED;
  private SingulatorState singulatorState = SingulatorState.PAUSED;
  private boolean singulatorBeamBreak = false;
  private int ballsFedCount = 0;
  private boolean intakeLimitSwitch = false;

  private boolean turretHoodBeamBreakRaw = false;
  private boolean turretHallLeftRaw = false;
  private boolean turretHallRightRaw = false;

  private double turretHoodAbsolutePositionRotations = 0.0;
  private double turretRotationAbsolutePositionRotations = 0.0;

  private double turretFlywheelPercent = 0.0;
  private double turretHoodPercent = 0.0;
  private double turretRotationPercent = 0.0;
  private double intakePercent = 0.0;
  private double intakeExtensionPercent = 0.0;
  private double climberPullPercent = 0.0;
  private double climberRotationPercent = 0.0;
  
  // Field position
  private Pose2d robotPose = new Pose2d();
  
  // PUBLIC API
  public void requestIntent(RobotIntent intent) {
    if (currentIntent == intent) {
      return;
    }
    currentIntent = intent;
    SmartLogger.logReplay("RobotState/Intent", intent.toString());
  }
  
  public RobotIntent getCurrentIntent() { return currentIntent; }
  
  public void setNavigationPhase(NavigationPhase navPhase) {
    if (this.navigationPhase == navPhase) {
      return;
    }
    this.navigationPhase = navPhase;
    SmartLogger.logReplay("RobotState/NavigationPhase", navPhase.toString());
  }
  
  public NavigationPhase getNavigationPhase() { return navigationPhase; }
  
  public void setRobotPose(Pose2d pose) { this.robotPose = pose; }
  public Pose2d getRobotPose() { return robotPose; }
  
  public void setMode(Mode mode) {
    if (this.mode == mode) {
      return;
    }
    this.mode = mode;
    SmartLogger.logReplay("RobotState/Mode", mode.toString());
  }
  
  public Mode getMode() { return mode; }

  public void setAlliance(DriverStation.Alliance alliance) {
    if (this.alliance == alliance) {
      return;
    }
    this.alliance = alliance;
    SmartLogger.logReplay("RobotState/Alliance", alliance.toString());
  }

  public DriverStation.Alliance getAlliance() { return alliance; }
  
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    SmartLogger.logReplay("RobotState/Enabled", enabled);
  }
  
  public boolean isEnabled() { return enabled; }
  
  public void setSysIdMode(boolean sysIdMode) {
    this.sysIdMode = sysIdMode;
    SmartLogger.logConsole("SysId mode " + (sysIdMode ? "enabled - vision updates disabled" : "disabled"), "SysId Mode");
  }
  
  public boolean isSysIdMode() { return sysIdMode; }
  
  public boolean isOperatorDriveLockout() {
    return operatorDriveLockout;
  }

  public void setOperatorDriveLockout(boolean operatorDriveLockout) {
    if (this.operatorDriveLockout == operatorDriveLockout) {
      return;
    }
    this.operatorDriveLockout = operatorDriveLockout;
    SmartLogger.logReplay("RobotState/OperatorDriveLockout", operatorDriveLockout);
  }

  public void setTurretState(TurretState turretState) {
    if (this.turretState == turretState) {
      return;
    }
    this.turretState = turretState;
    SmartLogger.logReplay("RobotState/TurretState", turretState.toString());
  }

  public TurretState getTurretState() { return turretState; }

  public void setIntakePosition(IntakePosition pos) {
    if (intakePosition == pos) return;
    intakePosition = pos;
    SmartLogger.logReplay("RobotState/Intake/Position", pos.toString());
  }

  public IntakePosition getIntakePosition() { return intakePosition; }

  public void setIntakeRollerState(IntakeRollerState rollerState) {
    if (intakeRollerState == rollerState) return;
    intakeRollerState = rollerState;
    SmartLogger.logReplay("RobotState/Intake/RollerState", rollerState.toString());
  }

  public IntakeRollerState getIntakeRollerState() { return intakeRollerState; }

  public void setIntakeLimitSwitch(boolean pressed) {
    if (intakeLimitSwitch == pressed) return;
    intakeLimitSwitch = pressed;
    SmartLogger.logReplay("RobotState/Intake/LimitSwitch", pressed);
  }

  public boolean isIntakeFullyRetracted() { return intakeLimitSwitch; }

  public void setSpindexerState(SpindexerState state) {
    if (this.spindexerState == state) return;
    this.spindexerState = state;
    SmartLogger.logReplay("RobotState/SpindexerState", state.toString());
  }
  public SpindexerState getSpindexerState() { return spindexerState; }

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

  // Incremented by SingulatorSubsystem each time a ball passes through; never resets during a match
  public void incrementBallsFed() {
    ballsFedCount++;
    SmartLogger.logReplay("RobotState/BallsFedCount", ballsFedCount);
  }
  public int getBallsFedCount() { return ballsFedCount; }

  public void setClimberState(ClimberState climberState) {
    if (this.climberState == climberState) {
      return;
    }
    this.climberState = climberState;
    SmartLogger.logReplay("RobotState/ClimberState", climberState.toString());
  }

  public ClimberState getClimberState() { return climberState; }

  public void setTurretHoodLimitSwitchRaw(boolean limitRaw) {
    if (turretHoodBeamBreakRaw == limitRaw) return;
    turretHoodBeamBreakRaw = limitRaw;
    SmartLogger.logReplay("RobotState/Turret/HoodLimitSwitchRaw", limitRaw);
  }

  public boolean getTurretHoodLimitSwitchRaw() { return turretHoodBeamBreakRaw; }

  public void setTurretHallLeftRaw(boolean hallRaw) {
    if (turretHallLeftRaw == hallRaw) {
      return;
    }
    turretHallLeftRaw = hallRaw;
    SmartLogger.logReplay("RobotState/Turret/HallLeftRaw", hallRaw);
  }

  public boolean getTurretHallLeftRaw() { return turretHallLeftRaw; }

  public void setTurretHallRightRaw(boolean hallRaw) {
    if (turretHallRightRaw == hallRaw) {
      return;
    }
    turretHallRightRaw = hallRaw;
    SmartLogger.logReplay("RobotState/Turret/HallRightRaw", hallRaw);
  }

  public boolean getTurretHallRightRaw() { return turretHallRightRaw; }

  public void setTurretHoodMotorPositionRotations(double rotations) {
    if (Math.abs(turretHoodAbsolutePositionRotations - rotations) < 0.0001) return;
    turretHoodAbsolutePositionRotations = rotations;
    SmartLogger.logReplay("RobotState/Turret/HoodMotorRot", rotations);
  }

  public double getTurretHoodMotorPositionRotations() { return turretHoodAbsolutePositionRotations; }

  public void setTurretRotationAbsolutePositionRotations(double rotations) {
    if (Math.abs(turretRotationAbsolutePositionRotations - rotations) < 0.0001) return;
    turretRotationAbsolutePositionRotations = rotations;
    SmartLogger.logReplay("RobotState/Turret/RotationAbsRot", rotations);
  }

  public double getTurretRotationAbsolutePositionRotations() { return turretRotationAbsolutePositionRotations; }

  public void setTurretFlywheelPercent(double percent) {
    if (Math.abs(turretFlywheelPercent - percent) < 0.001) return;
    turretFlywheelPercent = percent;
    SmartLogger.logReplay("RobotState/Turret/FlywheelPercent", percent);
  }

  public double getTurretFlywheelPercent() { return turretFlywheelPercent; }

  public void setTurretHoodPercent(double percent) {
    if (Math.abs(turretHoodPercent - percent) < 0.001) return;
    turretHoodPercent = percent;
    SmartLogger.logReplay("RobotState/Turret/HoodPercent", percent);
  }

  public double getTurretHoodPercent() { return turretHoodPercent; }

  public void setTurretRotationPercent(double percent) {
    if (Math.abs(turretRotationPercent - percent) < 0.001) return;
    turretRotationPercent = percent;
    SmartLogger.logReplay("RobotState/Turret/RotationPercent", percent);
  }

  public double getTurretRotationPercent() { return turretRotationPercent; }

  public void setIntakePercent(double percent) {
    if (Math.abs(intakePercent - percent) < 0.001) return;
    intakePercent = percent;
    SmartLogger.logReplay("RobotState/Intake/Percent", percent);
  }

  public double getIntakePercent() { return intakePercent; }

  public void setIntakeExtensionPercent(double percent) {
    if (Math.abs(intakeExtensionPercent - percent) < 0.001) return;
    intakeExtensionPercent = percent;
    SmartLogger.logReplay("RobotState/Intake/ExtensionPercent", percent);
  }

  public double getIntakeExtensionPercent() { return intakeExtensionPercent; }

  public void setClimberPullPercent(double percent) {
    if (Math.abs(climberPullPercent - percent) < 0.001) return;
    climberPullPercent = percent;
    SmartLogger.logReplay("RobotState/Climber/PullPercent", percent);
  }

  public double getClimberPullPercent() { return climberPullPercent; }

  public void setClimberRotationPercent(double percent) {
    if (Math.abs(climberRotationPercent - percent) < 0.001) return;
    climberRotationPercent = percent;
    SmartLogger.logReplay("RobotState/Climber/RotationPercent", percent);
  }

  public double getClimberRotationPercent() { return climberRotationPercent; }

  // Updates match phase state - call once per teleop periodic loop.
  public void updateMatchPhase() {
    matchPhaseTracker.update();
  }

  // True if our alliance hub is currently active for scoring.
  public boolean isHubActive() {
    return matchPhaseTracker.isHubActive();
  }

  // True if our hub will be active within leadSeconds from now.
  // Use to start flywheel spin-up before the window opens.
  public boolean isHubActiveIn(double leadSeconds) {
    return matchPhaseTracker.isHubActiveIn(leadSeconds);
  }

  // True if we should be shooting right now, accounting for lead/stop-early times.
  // leadSeconds: start spinning up this many seconds before a window opens.
  // stopEarlySeconds: stop feeding balls this many seconds before a window closes (flight time).
  public boolean shouldShoot(double leadSeconds, double stopEarlySeconds) {
    return matchPhaseTracker.shouldShoot(leadSeconds, stopEarlySeconds);
  }

  public MatchPhaseTracker.GamePhase getGamePhase() {
    return matchPhaseTracker.getPhase();
  }

  public String getPhaseName()            { return matchPhaseTracker.getPhaseName(); }
  public int getShiftNumber()             { return matchPhaseTracker.getShiftNumber(); }
  public double getSecondsUntilPhaseEnd() { return matchPhaseTracker.getSecondsUntilPhaseEnd(); }
  public boolean hasMatchGameData()       { return matchPhaseTracker.hasGameData(); }
}