package frc.robot;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.LinkedList;
import java.util.Queue;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Global robot state tracker - coordinates all subsystem states and robot intents.
 * 
 * ARCHITECTURE: Hierarchical State Machine with Intent Queue
 * - HIGH-LEVEL: Game modes (disabled, auto, teleop, test)
 * - MID-LEVEL: Robot intents (what the robot is trying to do)
 * - LOW-LEVEL: Mechanism states (actual positions of hardware)
 * 
 * CURRENT STATUS (Pre-2026 Game):
 *  High-level game modes working
 *  Navigation phase tracking working (SmartDriveToPosition)
 *  Intent system stubbed (ready for 2026 mechanisms)
 *  Mechanism states stubbed (examples for elevator, intake, etc.)
 * 
 * FUTURE IMPLEMENTATION (After 2026 Game Reveal):
 * 1. Add RobotIntent enum with game-specific actions (score coral, climb, etc.)
 * 2. Add mechanism state enums for each subsystem (elevator, intake, arm, etc.)
 * 3. Implement intent validation (prevent incompatible actions)
 * 4. Implement prerequisite checking (e.g., can't shoot until aimed)
 * 5. Enable intent queueing (driver can queue next action)
 * 
 * DESIGN GOALS:
 * - Separation of INTENT (what driver wants) vs ACTUAL (mechanism positions)
 * - Validation prevents dangerous/impossible states
 * - Subsystems report their actual state back to RobotState
 * - Commands request intents, RobotState validates and queues
 */
public class RobotState {
  
  // ===================================================================
  // HIGH-LEVEL: GAME STATE (FMS-driven, always active)
  // ===================================================================
  
  /**
   * Robot operating mode - set by Robot.java based on FMS/Driver Station
   */
  public enum Mode {
    DISABLED,       // Robot powered but not moving (safe for pit work)
    ENABLED_TELEOP, // Driver control with joysticks (ENABLED)
    ENABLED_AUTO,   // Running autonomous routine (ENABLED)
    TEST            // Test mode for manual subsystem testing
  }
  
  private Mode mode = Mode.DISABLED; // Start disabled for safety
  private boolean enabled = false; // Tracks if robot is enabled (auto/teleop/test)
  private boolean sysIdMode = false; // Special mode for motor characterization
  
  // ===================================================================
  // MID-LEVEL: ROBOT INTENT (What driver/auto WANTS to do)
  // ===================================================================
  
  /**
   * High-level robot intent - represents coordinated multi-subsystem actions.
   * 
   *  STUB FOR 2026 GAME - Update after game reveal!
   * 
   * Example intents for a hypothetical game with elevator, intake, shooter:
   * - IDLE: Stowed, doing nothing
   * - INTAKE_FLOOR: Extend intake, lower elevator
   * - SCORE_LOW: Raise elevator to low position, prepare to score
   * - SCORE_HIGH: Raise elevator to high position, prepare to score
   * - CLIMB: Deploy climb mechanism, raise hooks
   */
  public enum RobotIntent {
    // ===== CURRENT (2025 OFF-SEASON TESTING) =====
    IDLE,         // Robot stowed, doing nothing
    NAVIGATING,   // SmartDriveToPosition active (currently working!)
    
    // ===== FUTURE (UNCOMMENT AND MODIFY AFTER 2026 GAME REVEAL) =====
    // INTAKE_FLOOR,      // Collect game piece from floor
    // INTAKE_STATION,    // Collect from human player station
    // SCORE_LOW,         // Score in low goal
    // SCORE_MID,         // Score in mid goal
    // SCORE_HIGH,        // Score in high goal
    // PREPARE_CLIMB,     // Position for endgame climb
    // CLIMBING,          // Actively climbing
    // EJECT              // Emergency eject game piece
  }
  
  private RobotIntent currentIntent = RobotIntent.IDLE;
  private RobotIntent requestedIntent = RobotIntent.IDLE;
  
  // Intent queueing (for future use when mechanisms added)
  private final Queue<RobotIntent> intentQueue = new LinkedList<>();
  
  // ===================================================================
  // LOW-LEVEL: MECHANISM STATES (Actual hardware positions)
  // ===================================================================
  
  /**
   * Navigation phase - tracks SmartDriveToPosition progress (CURRENTLY ACTIVE)
   */
  public enum NavigationPhase {
    NONE,              // Not navigating (driver control)
    FAST_APPROACH,     // Phase 1 of SmartDrive - fast hook to vision point
    PRECISION,         // Phase 2 of SmartDrive - slow precision approach
    VISION_LOST,       // Vision was lost during precision phase - using odometry only
    DRIVER_OVERRIDE,   // Driver took control during navigation
    STUCK              // Robot detected as stuck (no movement for 2s)
  }
  
  private NavigationPhase navPhase = NavigationPhase.NONE; //  Working now!
  
  // ===================================================================
  //  FUTURE MECHANISM STATES (Uncomment after 2026 game reveal)
  // ===================================================================
  
  /*
  // Example: Elevator subsystem state
  public enum ElevatorState {
    STOWED(0.0),          // Fully retracted
    LOW(12.0),            // Low scoring position (12 inches)
    MID(24.0),            // Mid scoring position (24 inches)
    HIGH(36.0),           // High scoring position (36 inches)
    TRANSITIONING(-1.0);  // Moving between positions
    
    public final double heightInches;
    ElevatorState(double height) { this.heightInches = height; }
  }
  private ElevatorState elevatorState = ElevatorState.STOWED;
  
  // Example: Intake subsystem state
  public enum IntakeState {
    RETRACTED,   // Stowed inside frame
    DEPLOYED,    // Extended outside frame
    INTAKING,    // Running intake rollers (in)
    EJECTING     // Running intake rollers (out)
  }
  private IntakeState intakeState = IntakeState.RETRACTED;
  
  // Example: Shooter subsystem state
  public enum ShooterState {
    IDLE(0.0),            // Not spinning
    SPINNING_UP(-1.0),    // Accelerating to target speed
    AT_SPEED(4500.0);     // Ready to shoot
    
    public final double rpm;
    ShooterState(double rpm) { this.rpm = rpm; }
  }
  private ShooterState shooterState = ShooterState.IDLE;
  */
  
  // ===================================================================
  // FIELD POSITION TRACKING
  // ===================================================================
  
  private Pose2d robotPose = new Pose2d(); // Current field position (updated by PoseEstimator)
  
  // ===================================================================
  // LEGACY ASSIST MODE (Will be replaced by RobotIntent system)
  // ===================================================================
  
  /**
   * @deprecated Use RobotIntent instead. Kept for backward compatibility.
   */
  @Deprecated
  public enum AssistMode {
    NONE,
    ALIGN_TO_TARGET,
    AUTO_SCORE,
    FOLLOW_PREPLANNED_PATH
  }
  private AssistMode assistMode = AssistMode.NONE;
  
  // ===================================================================
  // PUBLIC API - INTENT MANAGEMENT (Future use)
  // ===================================================================
  
  /**
   * Request a robot action (driver button, auto command, etc.)
   *  STUB - Implement validation after 2026 game reveal
   * 
   * @param intent The high-level action to perform
   */
  public void requestIntent(RobotIntent intent) {
    // For now, just set it directly (no validation)
    currentIntent = intent;
    Logger.recordOutput("RobotState/Intent", intent.toString());
    
    //  FUTURE: Add validation and queueing
    /*
    if (!isIntentAllowed(intent)) {
      DriverStation.reportWarning("Intent " + intent + " blocked - invalid in current state", false);
      Logger.recordOutput("RobotState/IntentRejected", intent.toString());
      return;
    }
    
    intentQueue.add(intent);
    Logger.recordOutput("RobotState/IntentQueued", intent.toString());
    */
  }
  
  /**
   * Process queued intents - call this in Robot.periodic()
   *  STUB - Implement after mechanisms added
   */
  public void processIntents() {
    //  FUTURE: Process intent queue with prerequisite checking
    /*
    if (intentQueue.isEmpty()) return;
    
    RobotIntent nextIntent = intentQueue.peek();
    
    if (arePrerequisitesMet(nextIntent)) {
      requestedIntent = intentQueue.poll(); // Execute
      Logger.recordOutput("RobotState/IntentExecuting", requestedIntent.toString());
    } else {
      Logger.recordOutput("RobotState/IntentWaiting", nextIntent.toString());
    }
    */
  }
  
  // ===================================================================
  //  FUTURE: INTENT VALIDATION (Uncomment after mechanisms added)
  // ===================================================================
  
  /*
  private boolean isIntentAllowed(RobotIntent intent) {
    switch (intent) {
      case SCORE_HIGH:
        // Can't score while intaking
        return currentIntent != RobotIntent.INTAKE_FLOOR &&
               currentIntent != RobotIntent.INTAKE_STATION;
      
      case INTAKE_FLOOR:
        // Can't intake while scoring
        return currentIntent != RobotIntent.SCORE_LOW &&
               currentIntent != RobotIntent.SCORE_MID &&
               currentIntent != RobotIntent.SCORE_HIGH;
      
      case CLIMBING:
        // Only allow climb in last 30 seconds of match
        return DriverStation.getMatchTime() < 30.0;
      
      default:
        return true; // Most intents always allowed
    }
  }
  
  private boolean arePrerequisitesMet(RobotIntent intent) {
    switch (intent) {
      case SCORE_HIGH:
        // Need elevator at high position
        return elevatorState == ElevatorState.HIGH;
      
      case INTAKE_FLOOR:
        // Need intake deployed
        return intakeState == IntakeState.DEPLOYED;
      
      default:
        return true; // No prerequisites
    }
  }
  */
  
  // ===================================================================
  // PUBLIC API - MECHANISM STATE REPORTING (Future use)
  // ===================================================================
  
  /**
   *  FUTURE: Subsystems call these to report their actual state
   * Uncomment after mechanisms added
   */
  
  /*
  public void setElevatorState(ElevatorState state) {
    this.elevatorState = state;
    Logger.recordOutput("RobotState/Elevator", state.toString());
  }
  
  public void setIntakeState(IntakeState state) {
    this.intakeState = state;
    Logger.recordOutput("RobotState/Intake", state.toString());
  }
  
  public void setShooterState(ShooterState state) {
    this.shooterState = state;
    Logger.recordOutput("RobotState/Shooter", state.toString());
  }
  */
  
  // ===================================================================
  // PUBLIC API - GETTERS (Current + Future)
  // ===================================================================
  
  public RobotIntent getCurrentIntent() { return currentIntent; }
  
  /*
  //  FUTURE: Mechanism state getters
  public ElevatorState getElevatorState() { return elevatorState; }
  public IntakeState getIntakeState() { return intakeState; }
  public ShooterState getShooterState() { return shooterState; }
  */
  
  // ===================================================================
  // PUBLIC API - NAVIGATION (Currently working!)
  // ===================================================================
  
  public void setNavigationPhase(NavigationPhase navPhase) {
    this.navPhase = navPhase;
    log();
  }
  
  public NavigationPhase getNavigationPhase() {
    return navPhase;
  }
  
  // ===================================================================
  // PUBLIC API - FIELD POSITION
  // ===================================================================
  
  public void setRobotPose(Pose2d pose) {
    this.robotPose = pose;
  }
  
  public Pose2d getRobotPose() {
    return robotPose;
  }
  
  // ===================================================================
  // PUBLIC API - GAME STATE
  // ===================================================================
  
  public void setMode(Mode mode) {
    this.mode = mode;
    log();
  }
  
  public Mode getMode() {
    return mode;
  }
  
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    log();
  }
  
  public boolean isEnabled() {
    return enabled;
  }
  
  public boolean isSysIdMode() {
    return sysIdMode;
  }
  
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
  
  // ===================================================================
  // PUBLIC API - LEGACY (Deprecated, kept for compatibility)
  // ===================================================================
  
  /**
   * @deprecated Use requestIntent(RobotIntent) instead
   */
  @Deprecated
  public void setAssistMode(AssistMode assistMode) {
    this.assistMode = assistMode;
    log();
  }
  
  /**
   * @deprecated Use getCurrentIntent() instead
   */
  @Deprecated
  public AssistMode getAssistMode() {
    return assistMode;
  }
  
  // ===================================================================
  // LOGGING
  // ===================================================================
  
  private void log() {
    Logger.recordOutput("RobotState/Mode", mode.toString());
    Logger.recordOutput("RobotState/AssistMode", assistMode.toString()); // Legacy
    Logger.recordOutput("RobotState/Intent", currentIntent.toString());
    Logger.recordOutput("RobotState/NavigationPhase", navPhase.toString());
    Logger.recordOutput("RobotState/Enabled", enabled);
    
    //  FUTURE: Log mechanism states
    /*
    Logger.recordOutput("RobotState/Elevator", elevatorState.toString());
    Logger.recordOutput("RobotState/Intake", intakeState.toString());
    Logger.recordOutput("RobotState/Shooter", shooterState.toString());
    */
  }
  
  // ===================================================================
  // USAGE EXAMPLES (For future reference)
  // ===================================================================
  
  /*
  // ===== IN A COMMAND =====
  public class ScoreHighCommand extends Command {
    private final RobotState robotState;
    private final ElevatorSubsystem elevator;
    
    @Override
    public void initialize() {
      // Request high-level intent
      robotState.requestIntent(RobotIntent.SCORE_HIGH);
      
      // Command tells subsystem what to do
      elevator.setHeight(RobotState.ElevatorState.HIGH);
    }
    
    @Override
    public boolean isFinished() {
      // Wait for subsystem to reach desired state
      return robotState.getElevatorState() == RobotState.ElevatorState.HIGH;
    }
  }
  
  // ===== IN A SUBSYSTEM =====
  public class ElevatorSubsystem extends SubsystemBase {
    @Override
    public void periodic() {
      // Report actual state to RobotState
      if (Math.abs(encoder.getPosition() - targetHeight) < 0.5) {
        robotState.setElevatorState(RobotState.ElevatorState.HIGH); // Reached
      } else {
        robotState.setElevatorState(RobotState.ElevatorState.TRANSITIONING); // Moving
      }
    }
  }
  
  // ===== IN ROBOT.PERIODIC() =====
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotState.processIntents(); // Process queued intents
  }
  */
}