package frc.robot;

import frc.robot.util.MatchPhaseTracker;
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
  private boolean operatorDriveLockout = false;
  private edu.wpi.first.wpilibj.DriverStation.Alliance alliance = edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

  // Match phase and hub active tracking
  private final MatchPhaseTracker matchPhaseTracker = new MatchPhaseTracker();
  
  // Robot intent (high-level actions)
  public enum RobotIntent {
    IDLE, 
    NAVIGATING
    // TODO 2026: Add INTAKE_FLOOR, SCORE_HIGH, CLIMB, etc.
  }
  private RobotIntent currentIntent = RobotIntent.IDLE;
  @SuppressWarnings("unused")
  private final Queue<RobotIntent> intentQueue = new LinkedList<>();
  
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

  public enum IntakeState {
    IDLE,
    ACTIVE
  }

  public enum ClimberState {
    IDLE,
    ACTIVE
  }

  private TurretState turretState = TurretState.IDLE;
  private IntakeState intakeState = IntakeState.IDLE;
  private ClimberState climberState = ClimberState.IDLE;

  private boolean turretSingulatorBeamBreakRaw = false;
  private boolean turretHoodBeamBreakRaw = false;
  private boolean turretHallLeftRaw = false;
  private boolean turretHallRightRaw = false;

  private double turretHoodAbsolutePositionRotations = 0.0;
  private double turretRotationAbsolutePositionRotations = 0.0;

  private double turretFlywheelPercent = 0.0;
  private double turretHoodPercent = 0.0;
  private double turretRotationPercent = 0.0;
  private double turretSingulatorPercent = 0.0;
  private double intakePercent = 0.0;
  private double intakeExtensionPercent = 0.0;
  private double climberPullPercent = 0.0;
  private double climberRotationPercent = 0.0;
  
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

  public void setAlliance(edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
    if (this.alliance == alliance) {
      return;
    }
    this.alliance = alliance;
    SmartLogger.logReplay("RobotState/Alliance", alliance.toString());
  }

  public edu.wpi.first.wpilibj.DriverStation.Alliance getAlliance() { return alliance; }
  
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

  public void setIntakeState(IntakeState intakeState) {
    if (this.intakeState == intakeState) {
      return;
    }
    this.intakeState = intakeState;
    SmartLogger.logReplay("RobotState/IntakeState", intakeState.toString());
  }

  public IntakeState getIntakeState() { return intakeState; }

  public void setClimberState(ClimberState climberState) {
    if (this.climberState == climberState) {
      return;
    }
    this.climberState = climberState;
    SmartLogger.logReplay("RobotState/ClimberState", climberState.toString());
  }

  public ClimberState getClimberState() { return climberState; }

  public void setTurretSingulatorBeamBreakRaw(boolean beamBreakRaw) {
    if (turretSingulatorBeamBreakRaw == beamBreakRaw) {
      return;
    }
    turretSingulatorBeamBreakRaw = beamBreakRaw;
    SmartLogger.logReplay("RobotState/Turret/SingulatorBeamBreakRaw", beamBreakRaw);
  }

  public boolean getTurretSingulatorBeamBreakRaw() { return turretSingulatorBeamBreakRaw; }

  public void setTurretHoodBeamBreakRaw(boolean beamBreakRaw) {
    if (turretHoodBeamBreakRaw == beamBreakRaw) {
      return;
    }
    turretHoodBeamBreakRaw = beamBreakRaw;
    SmartLogger.logReplay("RobotState/Turret/HoodBeamBreakRaw", beamBreakRaw);
  }

  public boolean getTurretHoodBeamBreakRaw() { return turretHoodBeamBreakRaw; }

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

  public void setTurretHoodAbsolutePositionRotations(double rotations) {
    if (Math.abs(turretHoodAbsolutePositionRotations - rotations) < 0.0001) return;
    turretHoodAbsolutePositionRotations = rotations;
    SmartLogger.logReplay("RobotState/Turret/HoodAbsRot", rotations);
  }

  public double getTurretHoodAbsolutePositionRotations() { return turretHoodAbsolutePositionRotations; }

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

  public void setTurretSingulatorPercent(double percent) {
    if (Math.abs(turretSingulatorPercent - percent) < 0.001) return;
    turretSingulatorPercent = percent;
    SmartLogger.logReplay("RobotState/Turret/SingulatorPercent", percent);
  }

  public double getTurretSingulatorPercent() { return turretSingulatorPercent; }

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