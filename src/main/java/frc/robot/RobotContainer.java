package frc.robot;

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.Auto.*;
import static frc.robot.Constants.StartingPositions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.DynamicBumpTraversalCommand;
import frc.robot.commands.drive.AllianceZoneSweepSimplifiedCommand;
import frc.robot.commands.drive.NeutralZoneSweepSimplifiedCommand;
import frc.robot.commands.drive.OpposingAllianceZoneSweepSimplifiedCommand;
import frc.robot.commands.drive.WallTraversalCommand;
import frc.robot.commands.drive.SnapToHeadingDynamic;
import frc.robot.commands.drive.SmartDriveToPosition;
import frc.robot.commands.util.SetStartingPoseCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.turret.TurretIOCTRE;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretAimPipeline;
import frc.robot.subsystems.turret.TurretAimSolver;
import frc.robot.subsystems.turret.TurretTargetSelector;
import frc.robot.util.SmartLogger;
import frc.robot.util.TouchscreenInterface;

// Wires up robot hardware, controllers, and commands
// To grab latest 10 logs and delete them: run .\scripts\storelogs.bat
public class RobotContainer {
  // === CONFIGURATION ===
  public static final boolean COMPETITION_MODE = false; // Disable logs/streams for matches
  private static final boolean ENABLE_CONSOLE_LOGGING = !COMPETITION_MODE;
  private static final boolean USE_TOUCHSCREEN_OPERATOR = true;
  private static final boolean SYSID_MODE = false; // Phoenix Tuner X characterization mode
  private static final boolean ENABLE_TURRET = true;   // rotation + hall sensor testing — hood/flywheel bindings remain commented out
  private static final boolean ENABLE_INTAKE = true;
  private static final boolean ENABLE_CLIMBER = false;
  private static final boolean ENABLE_SPINDEXER = true;
  private static final boolean ENABLE_SINGULATOR = true;
  private static final double AUTO_SEED_POS_TOL_METERS = 0.20;
  private static final double AUTO_SEED_ROT_TOL_DEG = 10.0;

  private static Alliance cachedAlliance = Alliance.Blue;

  // Driver Xbox controller on USB port defined in Constants
  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
  // Operator Xbox controller on the next USB slot (port 1)
  // Suppress unused warning - field is used when operator binding sections are uncommented
  @SuppressWarnings("unused")
  private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  // When REQUIRE_TURRET_FORWARD_CONFIRM=true, bindings are deferred until Back+A confirm.
  private boolean bindingsConfigured = false;
  // Flywheel warm-up toggle state (operator A button) - used when flywheel section is uncommented
  @SuppressWarnings("unused")
  private boolean flywheelOn = false;
  // Steps through 30/40/50/60/70/80/90/100% on each right bumper press, wraps back to 30
  private int flywheelSpeedIndex = 0;
  private static final double[] FLYWHEEL_SPEED_STEPS = { 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00 };
  // Right stick Y trims this continuously; kept in sync when bumpers/A button set a step
  private double flywheelSpeedPercent = 0.30;
  // Per-loop trim rate at full deflection: ~17s to sweep full 30-100% range
  private static final double FLYWHEEL_TRIM_RATE = 0.004;

  // Subsystems - order here matches initialization order in constructor
  final RobotState robotState;
  final GyroSubsystem gyro;
  final QuestNavSubsystem questNav;
  final DriveSubsystem driveSubsystem;
  final PoseEstimatorSubsystem poseEstimator;
  final TagVisionSubsystem tagVisionSubsystem;
  public final LEDSubsystem ledSubsystem;
  TurretSubsystem turretSubsystem;
  IntakeSubsystem intakeSubsystem;
  ClimberSubsystem climberSubsystem;
  SpindexerSubsystem spindexerSubsystem;
  SingulatorSubsystem singulatorSubsystem;

  // Autonomous chooser shown on dashboard; selection drives pose preview and auto init
  private final SendableChooser<Command> autoChooser;
  private Command lastSelectedAuto = null; // Track selection for preview updates

  // Preview thread writes these; main thread reads them via applyPendingAutoPreviewPose()
  // volatile ensures changes are visible across threads without synchronization
  private volatile Pose2d pendingAutoPreviewPose = null;
  private volatile String pendingAutoPreviewName = null;

  // Last values written to the robot - used to skip redundant resets
  private Pose2d lastAppliedPreviewPose = null;
  private String lastAppliedPreviewName = null;

  private volatile boolean previewThreadRunning = true;

  // Force preview thread to reapply pose/orientation on every boot
  private boolean bootPreviewApplied = false;

  private TouchscreenInterface touchscreen;
  private int periodicCounter = 0;

  // === CONSTRUCTOR - Runs once at robot boot ===
  public RobotContainer(RobotState robotState) {
    this.robotState = robotState;

    SmartLogger.configure(ENABLE_CONSOLE_LOGGING); // Configure first so all init logs work

    gyro = new GyroSubsystem();
    questNav = new QuestNavSubsystem();
    driveSubsystem = new DriveSubsystem(this.robotState, gyro);
    poseEstimator = new PoseEstimatorSubsystem(driveSubsystem, this.robotState, questNav);
    tagVisionSubsystem = new TagVisionSubsystem(poseEstimator);
    ledSubsystem = new LEDSubsystem(this.robotState);
    turretSubsystem = ENABLE_TURRET ? new TurretSubsystem(this.robotState, new TurretIOCTRE()) : null;
    intakeSubsystem = ENABLE_INTAKE ? new IntakeSubsystem(this.robotState) : null;

    // Wire pose-based tracking as the turret's default command (active in PHASE_2+).
    // While no higher-priority command holds the turret, it continuously solves bearing to target.
    // The aim goal only enables when phase >= PHASE_2 and pose is initialized.
    if (turretSubsystem != null) {
      TurretAimPipeline aimPipeline = new TurretAimPipeline(
          poseEstimator,
          driveSubsystem,
          new TurretTargetSelector(poseEstimator),
          new TurretAimSolver());
      turretSubsystem.setDefaultCommand(
          Commands.run(() -> {
            turretSubsystem.updateAimFromProvider(aimPipeline);
            // Right stick Y: trim flywheel speed up/down while flywheels are on (deadband 0.1)
            double trimAxis = -operatorController.getRightY(); // up = positive = faster
            if (Math.abs(trimAxis) < 0.1) trimAxis = 0.0;
            if (trimAxis != 0.0 && flywheelOn) {
              flywheelSpeedPercent = Math.max(0.30, Math.min(1.00, flywheelSpeedPercent + trimAxis * FLYWHEEL_TRIM_RATE));
              turretSubsystem.setFlywheelFrontPercent(flywheelSpeedPercent);
              turretSubsystem.setFlywheelBackPercent(flywheelSpeedPercent);
            }
          }, turretSubsystem)
              .withName("TurretTrackingDefault"));
    }
    climberSubsystem = ENABLE_CLIMBER ? new ClimberSubsystem(this.robotState) : null;
    spindexerSubsystem = ENABLE_SPINDEXER ? new SpindexerSubsystem(this.robotState) : null;
    singulatorSubsystem = ENABLE_SINGULATOR ? new SingulatorSubsystem(this.robotState) : null;

    if (!ENABLE_TURRET) {
      SmartLogger.logConsole("Turret disabled until hardware is ready", "Startup");
    }
    if (!ENABLE_INTAKE) {
      SmartLogger.logConsole("Intake disabled until hardware is ready", "Startup");
    }
    if (!ENABLE_CLIMBER) {
      SmartLogger.logConsole("Climber disabled until hardware is ready", "Startup");
    }
    if (!ENABLE_SPINDEXER) {
      SmartLogger.logConsole("Spindexer disabled until hardware is ready", "Startup");
    }
    if (!ENABLE_SINGULATOR) {
      SmartLogger.logConsole("Singulator disabled until hardware is ready", "Startup");
    }

    updateAllianceFromDriverStation();
    this.robotState.setAlliance(cachedAlliance); // Must be set before PathPlanner config

    if (COMPETITION_MODE) {
      SmartLogger.logReplay("Robot/CompetitionMode", true);
    }

    poseEstimator.setTagVisionSubsystem(tagVisionSubsystem); // Cross-wire vision into pose estimator
    SmartDriveToPosition.configure(poseEstimator, robotState, driveSubsystem, questNav); // Static config for SmartDrive commands

    configurePathPlanner();
    AutoCommands.register(intakeSubsystem, turretSubsystem, spindexerSubsystem, singulatorSubsystem, climberSubsystem);
    configureDefaultCommands();
    if (Constants.Turret.REQUIRE_TURRET_FORWARD_CONFIRM) {
      SmartLogger.logConsole("Waiting for turret forward confirm (Back+A) before enabling controls", "Homing");
      // Back+A: confirm turret is forward, then activate all controls.
      // The 0.1s delay ensures Back is fully released before the Back-reset binding activates.
      new Trigger(() -> operatorController.getBackButton() && operatorController.getAButton())
          .onTrue(Commands.sequence(
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> {
              if (bindingsConfigured) return;
              if (turretSubsystem != null) turretSubsystem.homeForward();
              configureButtonBindings();
              SmartLogger.logConsole("Turret confirmed forward — all controls now active", "Homing");
            })));
    } else {
      configureButtonBindings();
    }
    
    if (USE_TOUCHSCREEN_OPERATOR) {
      configureTouchscreenInterface();
    }
    
    autoChooser = AutoBuilder.buildAutoChooser(); // Scans deploy/pathplanner/autos/ for named autos
    SmartDashboard.putData("Auto Chooser", autoChooser); // Sends chooser widget to dashboard
    robotState.setSysIdMode(SYSID_MODE);
    poseEstimator.setAutoChooser(autoChooser); // Lets pose estimator read auto start poses
    startAutoPreviewMonitor(); // Background thread: watches chooser and queues pose previews
    
    SmartLogger.logConsole("RobotContainer initialized - all subsystems ready", "Init Complete", 5);
  }

  // Configure PathPlanner auto builder
  // Connects PathPlanner to this robot's drive system.
  // feedforwards are intentionally ignored - we use odometry-only closed-loop control.
  // Translation/rotation PID constants are tuned in Constants.java.
  private void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      
      AutoBuilder.configure(
          poseEstimator::getEstimatedPose,
          this::resetPose,
          driveSubsystem::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveSubsystem.driveRobotRelative(speeds), // feedforwards unused
          new PPHolonomicDriveController(
              new PIDConstants(
          TRANSLATION_KP,
          TRANSLATION_KI,
          TRANSLATION_KD),
              new PIDConstants(
          ROTATION_KP,
          ROTATION_KI,
          ROTATION_KD)),
          config,
          this::shouldFlipPath,
          driveSubsystem);
      
      SmartLogger.logConsole("PathPlanner configured - PID tunable in AdvantageScope", "PathPlanner");
    } catch (Exception e) {
      SmartLogger.logConsoleError("PathPlanner config failed: " + e.getMessage());
      DriverStation.reportWarning("PathPlanner config failed!", false);
    }
  }

  // Set default commands (run when subsystems idle)
  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(
        new DriveWithJoysticks(
            driveSubsystem, robotState,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> true,
      () -> false)); // Left bumper reserved for bump traversal.
  }

  // Map controller buttons to commands
  private void configureButtonBindings() {
    bindingsConfigured = true;

    // ========== UTILITY BUTTONS (ALWAYS ACTIVE) ==========

    // BACK: Reset field orientation
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(driveSubsystem.createOrientToFieldCommand(robotState));

    // START: Save current position
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .onTrue(new SetStartingPoseCommand(getRebuiltRightCornerPose(), "RIGHT CORNER", gyro, questNav, driveSubsystem, poseEstimator));

    // ========== NORMAL OPERATION BUTTONS (COMMENT OUT FOR SYSID) ==========

    // Y/B/X: SmartDrive to tags
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_ALLIANCE_LEFTBUMP, PRECISE_BLUE_ALLIANCE_LEFTBUMP));
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_ALLIANCE_RIGHTBUMP, PRECISE_BLUE_ALLIANCE_RIGHTBUMP));
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_ALLIANCE_RIGHTOWER, PRECISE_BLUE_ALLIANCE_RIGHTOWER));

    // A BUTTON: Smart sweep based on current zone
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(Commands.deferredProxy(this::createSmartSweepCommand));

    // LEFT/RIGHT BUMPER: Dynamic bump traversal
    // COMMENTED OUT during SysId — bumpers are used for SignalLogger start/stop
    /*
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(Commands.deferredProxy(() -> createBumpTraversalCommand(DynamicBumpTraversalCommand.Side.LEFT)));
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(Commands.deferredProxy(() -> createBumpTraversalCommand(DynamicBumpTraversalCommand.Side.RIGHT)));
    */

    // BOTH TRIGGERS: Dynamic heading snap - picks heading based on field position
    Trigger bothTriggers = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
        .and(new Trigger(() -> driverController.getRightTriggerAxis() > 0.5));
    bothTriggers.whileTrue(new SnapToHeadingDynamic(
        poseEstimator, driveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX()));

    // LEFT TRIGGER (alone): Hub auto-aim — turret tracks hub using pose while held.
    // Uses the same hubPipeline logic as operator right bumper but on the driver controller.
    // Only activates when right trigger is NOT also held (bothTriggers takes priority).
    if (turretSubsystem != null) {
      TurretAimPipeline driverHubPipeline = new TurretAimPipeline(
          poseEstimator,
          driveSubsystem,
          () -> cachedAlliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red
              ? Constants.HubCenters.RED_HUB_CENTER
              : Constants.HubCenters.BLUE_HUB_CENTER,
          new TurretAimSolver());
      new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
          .and(new Trigger(() -> driverController.getRightTriggerAxis() <= 0.5))
          .whileTrue(Commands.run(
              () -> turretSubsystem.updateAimFromProvider(driverHubPipeline), turretSubsystem)
              .withName("DriverHubAim"));
    }

    // D-PAD: Wall traversal commands (Up=Far, Down=Near, Left=Left wall, Right=Right wall)
    new Trigger(() -> driverController.getPOV() == 0)
        .whileTrue(Commands.deferredProxy(() -> new WallTraversalCommand(poseEstimator, driveSubsystem, WallTraversalCommand.Wall.FAR)));
    new Trigger(() -> driverController.getPOV() == 180)
        .whileTrue(Commands.deferredProxy(() -> new WallTraversalCommand(poseEstimator, driveSubsystem, WallTraversalCommand.Wall.NEAR)));
    new Trigger(() -> driverController.getPOV() == 270)
        .whileTrue(Commands.deferredProxy(() -> new WallTraversalCommand(poseEstimator, driveSubsystem, WallTraversalCommand.Wall.LEFT)));
    new Trigger(() -> driverController.getPOV() == 90)
        .whileTrue(Commands.deferredProxy(() -> new WallTraversalCommand(poseEstimator, driveSubsystem, WallTraversalCommand.Wall.RIGHT)));

    // ========== END NORMAL OPERATION BUTTONS ==========

    // ========== OPERATOR CONTROLLER BINDINGS ==========
    // Uncomment each section only after completing the corresponding commissioning checklist.
    // Each section is independently gated - you can enable intake without enabling turret, etc.

    // --- INTAKE ---
    /* Y: toggle extend/retract
    new JoystickButton(operatorController, XboxController.Button.kY.value)
        .onTrue(Commands.runOnce(() -> {
          if (intakeSubsystem == null) return;
          if (intakeSubsystem.isExtended()) {
            intakeSubsystem.retract();
          } else {
            intakeSubsystem.extend();
          }
        })); */
    // Right stick click: agitate — partial retract to 6.5 rot then re-extend (only when arm is out)
    new JoystickButton(operatorController, XboxController.Button.kRightStick.value)
        .onTrue(Commands.runOnce(() -> { if (intakeSubsystem != null) intakeSubsystem.agitate(); }));
    // X (hold): spin rollers inward - only works when arm is fully extended
    new JoystickButton(operatorController, XboxController.Button.kX.value)
        .whileTrue(Commands.startEnd(
          () -> { if (intakeSubsystem != null && intakeSubsystem.isExtended()) intakeSubsystem.spinIn(); },
          () -> { if (intakeSubsystem != null) intakeSubsystem.stopRollers(); }));
    /* B (hold): spin rollers outward (eject) - only works when arm is fully extended
    new JoystickButton(operatorController, XboxController.Button.kB.value)
        .whileTrue(Commands.startEnd(
          () -> { if (intakeSubsystem != null && intakeSubsystem.isExtended()) intakeSubsystem.spinOut(); },
          () -> { if (intakeSubsystem != null) intakeSubsystem.stopRollers(); })); */
    // --- END INTAKE ---

    // --- SHOT PRESETS ---
    // Y: HUBCLOSE preset (~1.04m from hub bumper-to-face)
    new JoystickButton(operatorController, XboxController.Button.kY.value)
        .onTrue(Commands.runOnce(() -> {
          if (turretSubsystem == null) return;
          turretSubsystem.setTurretPositionTarget(Constants.TurretTargets.HUBCLOSE_TURRET_ROT);
          turretSubsystem.setHoodPositionTarget(Constants.TurretTargets.HUBCLOSE_HOOD_ROT);
          turretSubsystem.setFlywheelFrontRps(Constants.TurretTargets.HUBCLOSE_FRONT_RPS);
          turretSubsystem.setFlywheelBackRps(Constants.TurretTargets.HUBCLOSE_BACK_RPS);
        }));
    // B: MIDRANGE preset (~3.5m from hub) — placeholder values, tune on robot
    new JoystickButton(operatorController, XboxController.Button.kB.value)
        .onTrue(Commands.runOnce(() -> {
          if (turretSubsystem == null) return;
          turretSubsystem.setTurretPositionTarget(Constants.TurretTargets.MIDRANGE_TURRET_ROT);
          turretSubsystem.setHoodPositionTarget(Constants.TurretTargets.MIDRANGE_HOOD_ROT);
          turretSubsystem.setFlywheelFrontRps(Constants.TurretTargets.MIDRANGE_FRONT_RPS);
          turretSubsystem.setFlywheelBackRps(Constants.TurretTargets.MIDRANGE_BACK_RPS);
        }));
    // --- END SHOT PRESETS ---

    // --- SINGULATOR (uncomment after SingulatorSubsystem commissioning checklist is complete) ---
    // Set ENABLE_SINGULATOR = true before uncommenting.
    
    // Right trigger (hold): singulator only — prime then feed; release: pause
    new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5)
        .whileTrue(Commands.startEnd(
          () -> { if (singulatorSubsystem != null) singulatorSubsystem.primeAndFeed(); },
          () -> { if (singulatorSubsystem != null) singulatorSubsystem.pause(); }));
    // Left trigger (hold): spindexer + singulator together — full feed cycle.
    // Block spindexer only while intake arm is actively moving (EXTENDING/RETRACTING) — arm could cross ball path.
    new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5)
        .whileTrue(Commands.startEnd(
          () -> {
            boolean spindexerAllowed = intakeSubsystem == null
                || (robotState.getIntakePosition() != RobotState.IntakePosition.EXTENDING
                &&  robotState.getIntakePosition() != RobotState.IntakePosition.RETRACTING);
            if (spindexerSubsystem  != null && spindexerAllowed) spindexerSubsystem.spinForward();
            if (singulatorSubsystem != null) singulatorSubsystem.primeAndFeed();
          },
          () -> {
            if (spindexerSubsystem  != null) spindexerSubsystem.stop();
            if (singulatorSubsystem != null) singulatorSubsystem.pause();
          }));
     // --- END SINGULATOR ---

    // --- SPINDEXER (uncomment after SpindexerSubsystem commissioning checklist is complete) ---
    // Set ENABLE_SPINDEXER = true before uncommenting.
    // Note: Start button is reserved for emergency re-home.
    /*
    new JoystickButton(operatorController, XboxController.Button.kStart.value)
        .onTrue(Commands.runOnce(() -> {
          if (spindexerSubsystem == null) return;
          if (robotState.getSpindexerState() == RobotState.SpindexerState.STOPPED) {
            spindexerSubsystem.spinForward();
          } else {
            spindexerSubsystem.stop();
          }
        }));
    */
    // Operator Back (alone): reset turret homed state — commented out to avoid accidental de-home.
    // new Trigger(() -> operatorController.getBackButton() && !operatorController.getAButton())
    //     .onTrue(Commands.runOnce(() -> {
    //       if (turretSubsystem != null) turretSubsystem.resetHomed();
    //       SmartLogger.logConsole("Turret homed state reset — press Start to re-home", "Homing");
    //     }));
    // --- END SPINDEXER ---

    // --- TURRET HOOD TEST (commissioning — remove after HOOD CHECKLIST is complete) ---
    // D-pad up: step hood up (ping-pong through 0%, 33%, 67%, 100% of travel)
    new Trigger(() -> operatorController.getPOV() == 0)
        .onTrue(Commands.runOnce(() -> { if (turretSubsystem != null) turretSubsystem.hoodStepUp(); }));
    // D-pad down: toggle hood between bottom (0) and top
    new Trigger(() -> operatorController.getPOV() == 180)
        .onTrue(Commands.runOnce(() -> { if (turretSubsystem != null) turretSubsystem.hoodStepDown(); }));
    // --- END TURRET HOOD TEST ---

    // Operator A: load OUTPOST preset.
    new JoystickButton(operatorController, XboxController.Button.kA.value)
        .onTrue(Commands.runOnce(() -> {
          if (turretSubsystem == null) return;
          turretSubsystem.setTurretPositionTarget(Constants.TurretTargets.OUTPOST_TURRET_ROT);
          turretSubsystem.setHoodPositionTarget(Constants.TurretTargets.OUTPOST_HOOD_ROT);
          turretSubsystem.setFlywheelFrontRps(Constants.TurretTargets.OUTPOST_FRONT_RPS);
          turretSubsystem.setFlywheelBackRps(Constants.TurretTargets.OUTPOST_BACK_RPS);
          flywheelOn = true;
          flywheelSpeedIndex = 6;
          flywheelSpeedPercent = 0.90;
        }));

    // --- TURRET ROTATION ---
    // D-pad left/right (hold): rotate turret CCW/CW — open-loop, no soft limits.
    new Trigger(() -> operatorController.getPOV() == 270)
        .whileTrue(Commands.startEnd(
          () -> { if (turretSubsystem != null) turretSubsystem.setTurretPercent(-0.09); },
          () -> { if (turretSubsystem != null) turretSubsystem.setTurretPercent(0.0); }));
    new Trigger(() -> operatorController.getPOV() == 90)
        .whileTrue(Commands.startEnd(
          () -> { if (turretSubsystem != null) turretSubsystem.setTurretPercent(0.09); },
          () -> { if (turretSubsystem != null) turretSubsystem.setTurretPercent(0.0); }));
    // --- END TURRET ROTATION ---

    // --- TURRET FLYWHEELS ---
    // Right bumper (press): step speed up 10% per press (30->40->...->100->30->...)
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
        .onTrue(Commands.runOnce(() -> {
          if (turretSubsystem == null) return;
          double speed = FLYWHEEL_SPEED_STEPS[flywheelSpeedIndex];
          turretSubsystem.setFlywheelFrontPercent(speed);
          turretSubsystem.setFlywheelBackPercent(speed);
          turretSubsystem.scheduleFlywheelRpmRecord(speed);
          flywheelOn = true;
          flywheelSpeedPercent = speed;
          flywheelSpeedIndex = (flywheelSpeedIndex + 1) % FLYWHEEL_SPEED_STEPS.length;
        }));
    // Left bumper (press): force both flywheels off, reset speed index to 30%
    new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
        .onTrue(Commands.runOnce(() -> {
          if (turretSubsystem == null) return;
          turretSubsystem.setFlywheelFrontPercent(0.0);
          turretSubsystem.setFlywheelBackPercent(0.0);
          flywheelOn = false;
          flywheelSpeedIndex = 0;
          flywheelSpeedPercent = 0.30;
        }));
    // --- END TURRET FLYWHEELS ---

    // --- CLIMBER (uncomment after CLIMBER COMMISSIONING CHECKLIST in ClimberSubsystem is complete) ---
    // Set ENABLE_CLIMBER = true before uncommenting.
    /*
    // Left trigger (hold): run pull motor to climb
    new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5)
        .whileTrue(Commands.startEnd(
          () -> { if (climberSubsystem != null) climberSubsystem.setPullPercent(Constants.Climber.PULL_SPEED); },
          () -> { if (climberSubsystem != null) climberSubsystem.setPullPercent(0.0); }));
    // Left bumper up (hold): pivot arm forward
    new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(Commands.startEnd(
          () -> { if (climberSubsystem != null) climberSubsystem.setRotationPercent(Constants.Climber.ROTATION_SPEED); },
          () -> { if (climberSubsystem != null) climberSubsystem.setRotationPercent(0.0); }));
    */ // --- END CLIMBER ---

    // --- EMERGENCY STOP ---
    // Right bumper is currently used for flywheel back commissioning test.
    // Re-enable this block (and remove the flywheel test section above) once commissioning is done.
    /*
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
        .onTrue(Commands.runOnce(() -> {
          flywheelOn = false;
          if (intakeSubsystem     != null) intakeSubsystem.stopAll();
          if (singulatorSubsystem != null) singulatorSubsystem.stopAll();
          if (spindexerSubsystem  != null) spindexerSubsystem.stopAll();
          if (turretSubsystem     != null) turretSubsystem.stopAll();
          if (climberSubsystem    != null) climberSubsystem.stopAll();
          SmartLogger.logConsole("OPERATOR E-STOP triggered", "EStop");
        }));
    */ // --- END EMERGENCY STOP ---

    // ========== END OPERATOR CONTROLLER BINDINGS ==========

    // Remove stick-movement cancel behavior (operator takes over until driver interrupts)
    // new Trigger(() ->
    //     Math.abs(driverController.getLeftX()) > 0.10
    //         || Math.abs(driverController.getLeftY()) > 0.10
    //         || Math.abs(driverController.getRightX()) > 0.10)
    //     .onTrue(Commands.runOnce(() -> {
    //       if (touchscreen != null) {
    //         touchscreen.cancelActiveOperatorDrive();
    //       }
    //     }));

    // Operator Start: emergency re-home for mid-match reboot recovery only.
    // Normal startup uses Back+A (forward confirm) instead of this hall sweep.
    new JoystickButton(operatorController, XboxController.Button.kStart.value)
        .onTrue(Commands.runOnce(() -> {
          if (turretSubsystem != null) turretSubsystem.home();
          if (turretSubsystem != null) turretSubsystem.hoodHome();
          if (intakeSubsystem != null) intakeSubsystem.startHoming();
          SmartLogger.logConsole("Emergency re-home triggered (Start) — hall sweep + intake + hood", "Homing");
        }));

    // --- TURRET HUB TARGETING --- (disabled 2026-03-06 — re-enable when needed)
    /*
    if (turretSubsystem != null) {
      TurretAimPipeline hubPipeline = new TurretAimPipeline(
          poseEstimator,
          driveSubsystem,
          () -> cachedAlliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red
              ? Constants.HubCenters.RED_HUB_CENTER
              : Constants.HubCenters.BLUE_HUB_CENTER,
          new TurretAimSolver());
      new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
          .whileTrue(Commands.run(
              () -> turretSubsystem.updateAimFromProvider(hubPipeline), turretSubsystem)
              .withName("TurretHubTracking"));
    }
    */
    // --- END TURRET HUB TARGETING ---
  }

  // HTML touchscreen interface
  private void configureTouchscreenInterface() {
    touchscreen = new TouchscreenInterface(robotState, driveSubsystem, poseEstimator, questNav);
    touchscreen.configure();
  }

  // Called when auto starts
  public Command getAutonomousCommand() { 
    Command selectedAuto = autoChooser.getSelected();
    return (selectedAuto != null) ? wrapPathWithLogging(selectedAuto) : selectedAuto;
  }

  // Reset robot position
  private void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    SmartLogger.logConsole("Pose reset to: " + SmartLogger.formatPose(pose));
  }

  // Add start/end logging to auto paths
  private Command wrapPathWithLogging(Command pathCommand) {
    String pathName = (pathCommand.getName() != null && !pathCommand.getName().isEmpty()) 
        ? pathCommand.getName() 
        : "Unknown Path";
    
    final String finalPathName = pathName;
    
    return pathCommand
        .beforeStarting(() -> {
          Pose2d startPose = poseEstimator.getEstimatedPose();
          SmartLogger.logConsole("Segment: " + finalPathName + " | Start: " + SmartLogger.formatPose(startPose), "Path Start");
          SmartLogger.logReplay("Auto/CurrentSegment", finalPathName);
          SmartLogger.logReplay("Auto/SegmentStart", startPose);
        })
        .finallyDo((interrupted) -> {
          Pose2d endPose = poseEstimator.getEstimatedPose();
          SmartLogger.logConsole("Segment: " + finalPathName + " | End: " + SmartLogger.formatPose(endPose) + " | Interrupted: " + interrupted, "Path End");
          SmartLogger.logReplay("Auto/SegmentEnd", endPose);
          SmartLogger.logReplay("Auto/SegmentInterrupted", interrupted);
        });
  }

  // Mirror red alliance paths
  private boolean shouldFlipPath() {
    return isRedAlliance();
  }

  // Runs at 2Hz as a daemon thread while disabled.
  // Writes pendingAutoPreviewPose/Name (volatile) when the selected auto changes.
  // The main thread reads them in periodic() via applyPendingAutoPreviewPose().
  private void startAutoPreviewMonitor() {
    Thread previewThread = new Thread(() -> {
      while (previewThreadRunning && !Thread.currentThread().isInterrupted()) {
        try {
          if (DriverStation.isDisabled()) {
            Command selectedAuto = autoChooser.getSelected();

            // On first pass after boot, always apply even if auto hasn't changed.
            // This ensures orientation is correct regardless of prior cached state.
            if (selectedAuto != null && (selectedAuto != lastSelectedAuto || !bootPreviewApplied)) {
              lastSelectedAuto = selectedAuto;
              bootPreviewApplied = true;
              String autoName = selectedAuto.getName();

              Pose2d startingPose = poseEstimator
                  .getPoseInitializer()
                  .getStartPoseForAutoName(autoName);

              pendingAutoPreviewPose = startingPose;
              pendingAutoPreviewName = autoName;
            }
          }
          Thread.sleep(500);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          SmartLogger.logConsole("[Auto Preview] Thread interrupted - stopping");
          break;
        } catch (Exception e) {
          SmartLogger.logConsoleError("[Auto Preview] Error: " + e.getMessage());
        }
      }
      SmartLogger.logConsole("[Auto Preview] Thread stopped cleanly");
    });

    previewThread.setDaemon(true); // Daemon thread - dies automatically when robot code exits
    previewThread.setName("AutoPreview");
    previewThread.start();
  }

  public void periodic() {
    periodicCounter++;
    updateAllianceFromDriverStation();
    robotState.setAlliance(cachedAlliance);
    if (DriverStation.isDisabled()) {
      applyPendingAutoPreviewPose();
    }

    // Publish slow-changing fields at 10Hz - alliance/station/auto don't change every loop
    if (periodicCounter % 5 == 0) {
      SmartDashboard.putBoolean("Robot/IsRedAlliance", cachedAlliance == Alliance.Red);
      SmartDashboard.putNumber("Robot/StationNumber", DriverStation.getLocation().orElse(1));
      // Read the chooser's active option name from SmartDashboard - getSelected().getName() returns
      // the Java class name, not the auto name, so we read the NT entry directly
      String activeAuto = SmartDashboard.getString("Auto Chooser/active", "None");
      SmartDashboard.putString("Robot/AutoSelected", activeAuto);
    }
  }

  public static Alliance getAlliance() {
    return cachedAlliance;
  }

  public static boolean isRedAlliance() {
    return cachedAlliance == Alliance.Red;
  }

  private static void updateAllianceFromDriverStation() {
    cachedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  // Called from periodic() (main thread) to apply a pose queued by the preview thread.
  // Uses volatile reads - no locks needed because Pose2d is immutable.
  private void applyPendingAutoPreviewPose() {
    Pose2d pose = pendingAutoPreviewPose;
    String autoName = pendingAutoPreviewName;

    if (pose == null || autoName == null) {
      return;
    }

    if (pose.equals(lastAppliedPreviewPose) && autoName.equals(lastAppliedPreviewName)) {
      return;
    }

    pendingAutoPreviewPose = null;
    pendingAutoPreviewName = null;

    lastAppliedPreviewPose = pose;
    lastAppliedPreviewName = autoName;

    // Seed the gyro to the auto start pose's field heading, then set perspective = allianceDownfield.
    // CTRE field-centric: effectiveHeading = gyro - perspective, so forward = Red wall.
    boolean isRedForPreview = cachedAlliance == Alliance.Red;
    Rotation2d allianceDownfield = Rotation2d.fromDegrees(isRedForPreview ? 180.0 : 0.0);
    Rotation2d poseHeading = pose.getRotation();
    driveSubsystem.setGyroHeading(poseHeading);
    driveSubsystem.setOperatorPerspectiveForward(allianceDownfield);

    poseEstimator.resetPose(pose, poseHeading, driveSubsystem.getModulePositions());

    SmartLogger.logConsole(
        "Preview orient: alliance=" + (isRedForPreview ? "Red" : "Blue")
        + " | poseHeading=" + String.format("%.1f", poseHeading.getDegrees())
        + " | perspective=" + String.format("%.1f", allianceDownfield.getDegrees()),
        "Preview");

    if (questNavNeedsSeed(pose)) {
      questNav.seedToPose(pose);
    }

    SmartLogger.logConsole("Auto: " + autoName + " | Pose: " + SmartLogger.formatPose(pose), "Preview");
    SmartLogger.logReplay("Auto/PreviewPose", pose);
  }

  // Returns true if QuestNav's current pose is far enough from desiredPose to need re-seeding.
  // Tolerances are defined in Constants to avoid seeding on minor drift.
  private boolean questNavNeedsSeed(Pose2d desiredPose) {
    if (!questNav.isTracking()) {
      return false;
    }

    Pose2d qPose = questNav.getRobotPose().orElse(null);
    if (qPose == null) {
      return true;
    }

    double posErr = qPose.getTranslation().getDistance(desiredPose.getTranslation());
    double rotErr = Math.abs(qPose.getRotation().minus(desiredPose.getRotation()).getDegrees());

    SmartLogger.logReplay("Auto/QuestPoseErrMeters", posErr);
    SmartLogger.logReplay("Auto/QuestRotErrDeg", rotErr);

    return posErr > AUTO_SEED_POS_TOL_METERS || rotErr > AUTO_SEED_ROT_TOL_DEG;
  }

  // Picks the sweep command based on the robot's current X position.
  // Pose estimator is always Blue-origin, so we mirror X for Red before comparing.
  // This matches the same zone-detection pattern used by DynamicBumpTraversalCommand.
  private Command createSmartSweepCommand() {
    boolean isRed = isRedAlliance();
    double rawX = poseEstimator.getEstimatedPose().getX();
    double robotX = isRed ? (Constants.Field.FIELD_LENGTH_METERS - rawX) : rawX;
    boolean inAllianceZone  = robotX < Constants.Field.ALLIANCE_ZONE_LENGTH_METERS;
    boolean inOpposingZone  = robotX > (Constants.Field.FIELD_LENGTH_METERS - Constants.Field.ALLIANCE_ZONE_LENGTH_METERS);

    if (inAllianceZone) {
      SmartLogger.logConsole("->SWEEP: Alliance zone selected (x=" + String.format("%.2f", robotX) + ")", "Sweep");
      return new AllianceZoneSweepSimplifiedCommand(poseEstimator, driveSubsystem);
    }
    if (inOpposingZone) {
      SmartLogger.logConsole("->SWEEP: Opposing zone selected (x=" + String.format("%.2f", robotX) + ")", "Sweep");
      return new OpposingAllianceZoneSweepSimplifiedCommand(poseEstimator, driveSubsystem);
    }
    SmartLogger.logConsole("->SWEEP: Neutral zone selected (x=" + String.format("%.2f", robotX) + ")", "Sweep");
    return new NeutralZoneSweepSimplifiedCommand(poseEstimator, driveSubsystem);
  }

  // modifier=false means no modifier button is currently pressed.
  // Future: pass a button trigger here to enable the modified traversal variant.
  private Command createBumpTraversalCommand(DynamicBumpTraversalCommand.Side side) {
    boolean modifier = false; // no modifier active
    return new DynamicBumpTraversalCommand(
        poseEstimator,
        driveSubsystem,
        side,
        modifier,
        DynamicBumpTraversalCommand.createPrototypeConfig());
  }

  private Pose2d getRebuiltRightCornerPose() {
    if (!isRedAlliance()) return BLUE_REBUILT_RIGHT_CORNER;
    // On a full competition field use the mirrored far corner.
    // On the practice field use the dedicated Red seed pose instead.
    return COMPETITION_MODE ? RED_REBUILT_RIGHT_CORNER : RED_PRACTICE_SEED;
  }
}
