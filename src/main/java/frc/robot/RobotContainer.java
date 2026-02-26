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
  private static final boolean ENABLE_TURRET = false;
  private static final boolean ENABLE_INTAKE = false;
  private static final boolean ENABLE_CLIMBER = false;
  private static final boolean ENABLE_SPINDEXER = false;
  private static final boolean ENABLE_SINGULATOR = false;
  private static final double AUTO_SEED_POS_TOL_METERS = 0.20;
  private static final double AUTO_SEED_ROT_TOL_DEG = 10.0;

  private static Alliance cachedAlliance = Alliance.Blue;

  // Driver Xbox controller on USB port defined in Constants
  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
  // Operator Xbox controller on the next USB slot (port 1)
  // Suppress unused warning - field is used when operator binding sections are uncommented
  @SuppressWarnings("unused")
  private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  // Flywheel warm-up toggle state (operator A button) - used when flywheel section is uncommented
  @SuppressWarnings("unused")
  private boolean flywheelOn = false;

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
    ledSubsystem = new LEDSubsystem();
    turretSubsystem = ENABLE_TURRET ? new TurretSubsystem(this.robotState, new TurretIOCTRE()) : null;
    intakeSubsystem = ENABLE_INTAKE ? new IntakeSubsystem(this.robotState) : null;
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
    configureDefaultCommands();
    configureButtonBindings();
    
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
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(Commands.deferredProxy(() -> createBumpTraversalCommand(DynamicBumpTraversalCommand.Side.LEFT)));
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(Commands.deferredProxy(() -> createBumpTraversalCommand(DynamicBumpTraversalCommand.Side.RIGHT)));

    // BOTH TRIGGERS: Dynamic heading snap - picks heading based on field position
    Trigger bothTriggers = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
        .and(new Trigger(() -> driverController.getRightTriggerAxis() > 0.5));
    bothTriggers.whileTrue(new SnapToHeadingDynamic(
        poseEstimator, driveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX()));

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

    // --- INTAKE (uncomment after IntakeSubsystem commissioning checklist is complete) ---
    // Set ENABLE_INTAKE = true before uncommenting.
    /*
    // Y: toggle extend/retract
    new JoystickButton(operatorController, XboxController.Button.kY.value)
        .onTrue(Commands.runOnce(() -> {
          if (intakeSubsystem == null) return;
          if (intakeSubsystem.isExtended()) {
            intakeSubsystem.retract();
          } else {
            intakeSubsystem.extend();
          }
        }));
    // X (hold): spin rollers inward - only works when arm is fully extended
    new JoystickButton(operatorController, XboxController.Button.kX.value)
        .whileTrue(Commands.startEnd(
          () -> { if (intakeSubsystem != null && intakeSubsystem.isExtended()) intakeSubsystem.spinIn(); },
          () -> { if (intakeSubsystem != null) intakeSubsystem.stopRollers(); }));
    // B (hold): spin rollers outward (eject) - only works when arm is fully extended
    new JoystickButton(operatorController, XboxController.Button.kB.value)
        .whileTrue(Commands.startEnd(
          () -> { if (intakeSubsystem != null && intakeSubsystem.isExtended()) intakeSubsystem.spinOut(); },
          () -> { if (intakeSubsystem != null) intakeSubsystem.stopRollers(); }));
    */ // --- END INTAKE ---

    // --- SINGULATOR (uncomment after SingulatorSubsystem commissioning checklist is complete) ---
    // Set ENABLE_SINGULATOR = true before uncommenting.
    /*
    // Right trigger (hold): feed; release: pause
    new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5)
        .whileTrue(Commands.startEnd(
          () -> { if (singulatorSubsystem != null) singulatorSubsystem.spinFeed(); },
          () -> { if (singulatorSubsystem != null) singulatorSubsystem.pause(); }));
    */ // --- END SINGULATOR ---

    // --- SPINDEXER (uncomment after SpindexerSubsystem commissioning checklist is complete) ---
    // Set ENABLE_SPINDEXER = true before uncommenting.
    /*
    // Start: toggle spindexer on (forward) / off
    new JoystickButton(operatorController, XboxController.Button.kStart.value)
        .onTrue(Commands.runOnce(() -> {
          if (spindexerSubsystem == null) return;
          if (robotState.getSpindexerState() == RobotState.SpindexerState.STOPPED) {
            spindexerSubsystem.spinForward();
          } else {
            spindexerSubsystem.stop();
          }
        }));
    // Back: toggle spindexer between forward and reverse (for manual unjams)
    new JoystickButton(operatorController, XboxController.Button.kBack.value)
        .onTrue(Commands.runOnce(() -> {
          if (spindexerSubsystem == null) return;
          if (robotState.getSpindexerState() == RobotState.SpindexerState.REVERSE) {
            spindexerSubsystem.spinForward();
          } else {
            spindexerSubsystem.spinReverse();
          }
        }));
    */ // --- END SPINDEXER ---

    // --- TURRET HOOD (uncomment after HOOD CHECKLIST in TurretIOCTRE is complete) ---
    // Set ENABLE_TURRET = true before uncommenting.
    /*
    // D-pad up (hold): nudge hood angle down (negative percent)
    new Trigger(() -> operatorController.getPOV() == 0)
        .whileTrue(Commands.startEnd(
          () -> { if (turretSubsystem != null) turretSubsystem.setHoodPercent(-Constants.Turret.TURRET_HOME_SPEED_PERCENT); },
          () -> { if (turretSubsystem != null) turretSubsystem.setHoodPercent(0.0); }));
    // D-pad down (hold): nudge hood angle up (positive percent)
    new Trigger(() -> operatorController.getPOV() == 180)
        .whileTrue(Commands.startEnd(
          () -> { if (turretSubsystem != null) turretSubsystem.setHoodPercent(Constants.Turret.TURRET_HOME_SPEED_PERCENT); },
          () -> { if (turretSubsystem != null) turretSubsystem.setHoodPercent(0.0); }));
    */ // --- END TURRET HOOD ---

    // --- TURRET ROTATION (uncomment after TURRET ROTATION CHECKLIST in TurretIOCTRE is complete) ---
    // Set ENABLE_TURRET = true before uncommenting.
    /*
    // D-pad left (hold): nudge turret left
    new Trigger(() -> operatorController.getPOV() == 270)
        .whileTrue(Commands.startEnd(
          () -> { if (turretSubsystem != null) turretSubsystem.setTurretPercent(-Constants.Turret.TURRET_HOME_SPEED_PERCENT); },
          () -> { if (turretSubsystem != null) turretSubsystem.setTurretPercent(0.0); }));
    // D-pad right (hold): nudge turret right
    new Trigger(() -> operatorController.getPOV() == 90)
        .whileTrue(Commands.startEnd(
          () -> { if (turretSubsystem != null) turretSubsystem.setTurretPercent(Constants.Turret.TURRET_HOME_SPEED_PERCENT); },
          () -> { if (turretSubsystem != null) turretSubsystem.setTurretPercent(0.0); }));
    */ // --- END TURRET ROTATION ---

    // --- TURRET FLYWHEELS (uncomment after FLYWHEEL CHECKLIST in TurretIOCTRE is complete) ---
    // Set ENABLE_TURRET = true before uncommenting.
    /*
    // A: toggle flywheels on/off at close-range warm-up speed
    new JoystickButton(operatorController, XboxController.Button.kA.value)
        .onTrue(Commands.runOnce(() -> {
          if (turretSubsystem == null) return;
          flywheelOn = !flywheelOn;
          turretSubsystem.setFlywheelPercent(flywheelOn ? Constants.Turret.SHOT_TABLE_FLYWHEEL_FRONT_PCT[0] : 0.0);
        }));
    */ // --- END TURRET FLYWHEELS ---

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

    // --- EMERGENCY STOP (uncomment once at least one mechanism section above is uncommented) ---
    /*
    // Right bumper: immediately stops all mechanism subsystems and disables fire interlock.
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

    // To run SysId: comment out normal buttons above and uncomment configureSysIdBindings() below
    // configureSysIdBindings();
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

  // SysId characterization bindings - swap in for normal buttons when characterizing.
  // Before running: comment out configureButtonBindings() normal buttons and call this instead.
  // For STEER tests: set STEER_FEEDBACK_TYPE = RemoteCANcoder in Constants.java first.
  // Run all 4 tests (quasistatic fwd/rev, dynamic fwd/rev) in one session.
  @SuppressWarnings("unused")
  private void configureSysIdBindings() {
    // Left/Right bumper: start/stop SignalLogger
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(Commands.runOnce(() -> com.ctre.phoenix6.SignalLogger.start()));
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .onTrue(Commands.runOnce(() -> com.ctre.phoenix6.SignalLogger.stop()));

    // Y/A/B/X: translation tests (drive motors)
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(driveSubsystem.sysIdQuasistaticTranslation(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(driveSubsystem.sysIdQuasistaticTranslation(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(driveSubsystem.sysIdDynamicTranslation(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileTrue(driveSubsystem.sysIdDynamicTranslation(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));

    // D-Pad: steer tests (module steering motors)
    new Trigger(() -> driverController.getPOV() == 0)
        .whileTrue(driveSubsystem.sysIdQuasistaticSteer(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    new Trigger(() -> driverController.getPOV() == 180)
        .whileTrue(driveSubsystem.sysIdQuasistaticSteer(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
    new Trigger(() -> driverController.getPOV() == 90)
        .whileTrue(driveSubsystem.sysIdDynamicSteer(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    new Trigger(() -> driverController.getPOV() == 270)
        .whileTrue(driveSubsystem.sysIdDynamicSteer(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
  }
}
