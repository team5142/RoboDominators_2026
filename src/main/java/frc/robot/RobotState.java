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
  // When true: QuestNav is unreliable — pose-dependent commands are blocked, turret uses fixed preset.
  private boolean questNavEmergencyMode = false;
  private boolean turretPhase1Fallback  = false;
  // When true: auto shoot mode active — flywheels, singulator, and spindexer run automatically.
  private boolean autoShootMode = false;
  // When true: auto shoot is temporarily paused — ball stays staged, shooting is blocked.
  private boolean autoShootPaused = false;
  // Field zone suppression: bump, net, or tower shadow (set by TurretTargetSelector).
  private boolean fieldZoneSuppressed = false;
  // Deadzone suppression: target is behind the robot in the turret blind spot (set by TurretSubsystem).
  private boolean deadzoneSuppressed = false;
  // Flywheel warm-up state — readable by auto commands and the default turret command.
  private boolean flywheelOn = false;
  // Sequenced shooting mode — fires one ball every 4s automatically while flywheels are on.
  private boolean sequencedShootingMode = false;
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
    RETRACTING,     // moving inward toward limit switches
    AGITATING,      // partial retract to 6.5 rot, then re-extend to target
    BUMP_LIFTING    // tiny retract during bump traversal to clear slope, gravity returns it
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

  // Which field zone the robot is currently in — set by TurretTargetSelector each loop.
  public enum ShootingZone {
    ALLIANCE,   // our side — shoot into hub when hub is active
    NEUTRAL,    // mid-field — pass back during first 15s of opponent period
    OPPONENT    // opponent side — never shoot
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
  // Default to RETRACTED so extend/retract work when homing is disabled (ENABLE_INTAKE_HOMING = false).
  private IntakePosition intakePosition = IntakePosition.RETRACTED;
  private ShootingZone shootingZone = ShootingZone.ALLIANCE;
  private IntakeRollerState intakeRollerState = IntakeRollerState.STOPPED;
  private ClimberState climberState = ClimberState.IDLE;
  private SpindexerState spindexerState = SpindexerState.STOPPED;
  private SingulatorState singulatorState = SingulatorState.PAUSED;
  private boolean singulatorBeamBreak = false;
  private boolean deadZoneBeamBreak = false;
  private int ballsFedCount = 0;

  // Segmented ball counters — reset on autonomousInit or via resetBallCounters()
  private int ballsShotAuto     = 0;
  private int ballsShotTeleop   = 0;
  private int ballsShotEndgame  = 0;
  private int ballsPassed       = 0;
  private boolean intakeLimitSwitch = false;

  private boolean turretHoodBeamBreakRaw = false;
  private boolean turretHallCCWRaw = false;

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

  public boolean isQuestNavEmergencyMode() { return questNavEmergencyMode; }
  public void setQuestNavEmergencyMode(boolean active) {
    questNavEmergencyMode = active;
    SmartLogger.logConsole("QuestNav emergency mode " + (active ? "ACTIVE — pose commands blocked" : "cleared"), "Emergency");
  }

  public boolean isTurretPhase1Fallback() { return turretPhase1Fallback; }
  public void setTurretPhase1Fallback(boolean active) {
    turretPhase1Fallback = active;
    SmartLogger.logConsole("Turret Phase1 fallback " + (active ? "ACTIVE — turret locked forward" : "cleared"), "Emergency");
    SmartLogger.logReplay("RobotState/TurretPhase1Fallback", active);
  }

  public boolean isFlywheelOn() { return flywheelOn; }
  public void setFlywheelOn(boolean on) { flywheelOn = on; }

  public boolean isSequencedShootingMode() { return sequencedShootingMode; }
  public void setSequencedShootingMode(boolean on) { sequencedShootingMode = on; }

  public boolean isAutoShootMode() { return autoShootMode; }
  public void setAutoShootMode(boolean active) {
    autoShootMode = active;
    SmartLogger.logConsole("Auto shoot mode " + (active ? "ACTIVE" : "OFF"), "AutoShoot");
  }

  public boolean isAutoShootPaused() { return autoShootPaused; }
  public void setAutoShootPaused(boolean paused) { autoShootPaused = paused; }

  public boolean isShotSuppressed() { return fieldZoneSuppressed || deadzoneSuppressed; }

  public ShootingZone getShootingZone() { return shootingZone; }
  public void setShootingZone(ShootingZone zone) {
    if (shootingZone == zone) return;
    shootingZone = zone;
    SmartLogger.logReplay("RobotState/ShootingZone", zone.toString());
  }

  public void setFieldZoneSuppressed(boolean value) {
    if (fieldZoneSuppressed == value) return;
    fieldZoneSuppressed = value;
    SmartLogger.logReplay("RobotState/FieldZoneSuppressed", value);
    SmartLogger.logReplay("RobotState/ShotSuppressed", isShotSuppressed());
  }

  public void setDeadzoneSuppressed(boolean value) {
    if (deadzoneSuppressed == value) return;
    deadzoneSuppressed = value;
    SmartLogger.logReplay("RobotState/DeadzoneSuppressed", value);
    SmartLogger.logReplay("RobotState/ShotSuppressed", isShotSuppressed());
  }

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

  // True when a ball is stuck in the dead zone above the singulator, below the flywheels
  public void setDeadZoneBeamBreak(boolean value) {
    if (this.deadZoneBeamBreak == value) return;
    this.deadZoneBeamBreak = value;
    SmartLogger.logReplay("RobotState/DeadZoneBeamBreak", value);
  }
  public boolean getDeadZoneBeamBreak() { return deadZoneBeamBreak; }

  // Incremented when a ball exits toward the flywheels; decremented when one is pulled back.
  // Routes to the correct segment based on current zone and phase.
  public void incrementBallsFed() {
    ballsFedCount++;
    SmartLogger.logReplay("RobotState/BallsFedCount", ballsFedCount);
    if (shootingZone == ShootingZone.NEUTRAL) {
      ballsPassed++;
    } else {
      MatchPhaseTracker.GamePhase phase = getGamePhase();
      if (phase == MatchPhaseTracker.GamePhase.AUTO) {
        ballsShotAuto++;
      } else if (phase == MatchPhaseTracker.GamePhase.END_GAME) {
        ballsShotEndgame++;
      } else {
        ballsShotTeleop++;
      }
    }
  }

  public void decrementBallsFed() {
    if (ballsFedCount > 0) ballsFedCount--;
    SmartLogger.logReplay("RobotState/BallsFedCount", ballsFedCount);
    if (shootingZone == ShootingZone.NEUTRAL) {
      if (ballsPassed > 0) ballsPassed--;
    } else {
      MatchPhaseTracker.GamePhase phase = getGamePhase();
      if (phase == MatchPhaseTracker.GamePhase.AUTO) {
        if (ballsShotAuto > 0) ballsShotAuto--;
      } else if (phase == MatchPhaseTracker.GamePhase.END_GAME) {
        if (ballsShotEndgame > 0) ballsShotEndgame--;
      } else {
        if (ballsShotTeleop > 0) ballsShotTeleop--;
      }
    }
  }

  public int getBallsFedCount()    { return ballsFedCount; }
  public int getBallsShotAuto()    { return ballsShotAuto; }
  public int getBallsShotTeleop()  { return ballsShotTeleop; }
  public int getBallsShotEndgame() { return ballsShotEndgame; }
  public int getBallsPassed()      { return ballsPassed; }
  public int getBallsShotTotal()   { return ballsShotAuto + ballsShotTeleop + ballsShotEndgame; }

  public void resetBallCounters() {
    ballsFedCount    = 0;
    ballsShotAuto    = 0;
    ballsShotTeleop  = 0;
    ballsShotEndgame = 0;
    ballsPassed      = 0;
    SmartLogger.logConsole("Ball counters reset", "BallCount");
  }

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

  public void setTurretHallCCWRaw(boolean hallRaw) {
    if (turretHallCCWRaw == hallRaw) {
      return;
    }
    turretHallCCWRaw = hallRaw;
    SmartLogger.logReplay("RobotState/Turret/HallCCWRaw", hallRaw);
  }

  public boolean getTurretHallCCWRaw() { return turretHallCCWRaw; }

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

  // Zone-aware shoot gate — checks match phase and current robot zone.
  // Returns true when shooting/passing is allowed. Always true in plain teleop (no match time).
  public boolean shouldShootInZone() {
    return matchPhaseTracker.shouldShootInZone(shootingZone);
  }

  public MatchPhaseTracker.GamePhase getGamePhase() {
    return matchPhaseTracker.getPhase();
  }

  public String getPhaseName()            { return matchPhaseTracker.getPhaseName(); }
  public int getShiftNumber()             { return matchPhaseTracker.getShiftNumber(); }
  public double getSecondsUntilPhaseEnd() { return matchPhaseTracker.getSecondsUntilPhaseEnd(); }
  public boolean hasMatchGameData()       { return matchPhaseTracker.hasGameData(); }
}