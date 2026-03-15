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
import java.util.Optional;
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
import frc.robot.commands.auto.CenterMoveToShootAuto;
import frc.robot.commands.auto.DoNothingCenterAuto;
import frc.robot.commands.auto.DoNothingLeftAuto;
import frc.robot.commands.auto.DoNothingRightAuto;
import frc.robot.commands.auto.ShootInPlaceLeftAuto;
import frc.robot.commands.auto.ShootInPlaceLeftBotRotateAuto;
import frc.robot.commands.auto.ShootInPlaceRightAuto;
import frc.robot.commands.auto.ShootInPlaceRightBotRotateAuto;
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
  public static final boolean COMPETITION_MODE = true; // Disable logs/streams for matches
  private static final boolean ENABLE_CONSOLE_LOGGING = !COMPETITION_MODE;
  private static final boolean USE_TOUCHSCREEN_OPERATOR = false;
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
  private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  // When REQUIRE_TURRET_FORWARD_CONFIRM=true, bindings are deferred until Back+A confirm.
  private boolean bindingsConfigured = false;
  // Intake roller global enable — toggled by operator B. True by default so deploy auto-starts rollers.
  private boolean intakeRollersEnabled = true;

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

  // Shot seed pose chooser — shown in Elastic as a dropdown; START button reads the selection.
  private final SendableChooser<Pose2d> shotSeedChooser = new SendableChooser<>();

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
    if (intakeSubsystem != null) {
      // When arm finishes extending, start rollers only if the global roller enable is on.
      intakeSubsystem.setOnExtendComplete(() -> {
        if (intakeRollersEnabled) intakeSubsystem.spinIn();
      });
    }

    // Wire pose-based tracking as the turret's default command (active in PHASE_2+).
    // While no higher-priority command holds the turret, it continuously solves bearing to target.
    // The aim goal only enables when phase >= PHASE_2 and pose is initialized.
    if (turretSubsystem != null) {
      TurretAimSolver aimSolver = new TurretAimSolver();
      TurretTargetSelector targetSelector = new TurretTargetSelector(poseEstimator, robotState);
      TurretAimPipeline aimPipeline = new TurretAimPipeline(
          poseEstimator,
          driveSubsystem,
          targetSelector,
          aimSolver);
      turretSubsystem.setDefaultCommand(
          Commands.run(() -> {
            // Phase1Fallback or QuestNav emergency: hold turret forward under PID,
            // hood+flywheel still track distance. Bypasses trackingEnabled.
            if (robotState.isTurretPhase1Fallback() || robotState.isQuestNavEmergencyMode()) {
              edu.wpi.first.math.geometry.Pose2d robotPose = poseEstimator.getEstimatedPose();
              edu.wpi.first.math.geometry.Pose2d targetPose = targetSelector.get();
              double distanceM = (robotPose != null && targetPose != null)
                  ? robotPose.getTranslation().getDistance(targetPose.getTranslation())
                  : frc.robot.Constants.Turret.FALLBACK_DISTANCE_METERS;
              turretSubsystem.holdForwardUnderPID(distanceM);
            } else {
              turretSubsystem.updateAimFromProvider(aimPipeline);
            }
            if (robotState.isFlywheelOn()) {
              double frontRps = turretSubsystem.getAimGoalFrontRps();
              double backRps  = turretSubsystem.getAimGoalBackRps();
              if (frontRps <= 0.0) frontRps = Constants.Turret.FLYWHEEL_WARMUP_FRONT_RPS;
              if (backRps  <= 0.0) backRps  = Constants.Turret.FLYWHEEL_WARMUP_BACK_RPS;
              turretSubsystem.setFlywheelFrontRps(frontRps);
              turretSubsystem.setFlywheelBackRps(backRps);
            } else {
              turretSubsystem.setFlywheelPercent(0.0);
            }
          }, turretSubsystem)
              .beforeStarting(() -> aimSolver.resetLatch())
              .withName("TurretTrackingDefault"));
    }
    //climberSubsystem = ENABLE_CLIMBER ? new ClimberSubsystem(this.robotState) : null;
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
      SmartLogger.logConsole("Waiting for turret forward confirm (LB+RB) before enabling controls", "Homing");
      // LB+RB: confirm turret is forward, then activate all controls.
      new Trigger(() -> operatorController.getLeftBumperButton() && operatorController.getRightBumperButton())
          .onTrue(Commands.sequence(
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> {
              if (bindingsConfigured) return;
              if (turretSubsystem != null) turretSubsystem.homeForward();
              if (turretSubsystem != null) turretSubsystem.enableTracking();
              configureButtonBindings();
              SmartLogger.logConsole("Turret confirmed forward — all controls now active", "Homing");
            })));
    } else {
      if (turretSubsystem != null) turretSubsystem.enableTracking();
      configureButtonBindings();
    }
    
    if (USE_TOUCHSCREEN_OPERATOR) {
      configureTouchscreenInterface();
    }
    
    autoChooser = AutoBuilder.buildAutoChooser(""); // Scans deploy/pathplanner/autos/ for named autos
    autoChooser.setDefaultOption("ShootInPlaceRight", new ShootInPlaceRightAuto(turretSubsystem, spindexerSubsystem, singulatorSubsystem, intakeSubsystem, poseEstimator, driveSubsystem, robotState));
    autoChooser.addOption("ShootInPlaceLeft",  new ShootInPlaceLeftAuto(turretSubsystem, spindexerSubsystem, singulatorSubsystem, intakeSubsystem, poseEstimator, driveSubsystem, robotState));
    autoChooser.addOption("DoNothingCenter",   new DoNothingCenterAuto(turretSubsystem, poseEstimator, driveSubsystem));
    autoChooser.addOption("DoNothingLeft",     new DoNothingLeftAuto(turretSubsystem, poseEstimator, driveSubsystem));
    autoChooser.addOption("DoNothingRight",    new DoNothingRightAuto(turretSubsystem, poseEstimator, driveSubsystem));
    autoChooser.addOption("ShootInPlaceRightBotRotate", new ShootInPlaceRightBotRotateAuto(turretSubsystem, spindexerSubsystem, singulatorSubsystem, intakeSubsystem, poseEstimator, driveSubsystem, robotState));
    autoChooser.addOption("ShootInPlaceLeftBotRotate",  new ShootInPlaceLeftBotRotateAuto(turretSubsystem, spindexerSubsystem, singulatorSubsystem, intakeSubsystem, poseEstimator, driveSubsystem, robotState));
    autoChooser.addOption("CenterMoveToShoot",          new CenterMoveToShootAuto(turretSubsystem, spindexerSubsystem, singulatorSubsystem, intakeSubsystem, poseEstimator, driveSubsystem, robotState));
    SmartDashboard.putData("Auto Chooser", autoChooser); // Sends chooser widget to dashboard
    robotState.setSysIdMode(SYSID_MODE);
    poseEstimator.setAutoChooser(autoChooser); // Lets pose estimator read auto start poses
    startAutoPreviewMonitor(); // Background thread: watches chooser and queues pose previews

    shotSeedChooser.setDefaultOption("HUBCLOSE (1.28m)", Constants.StartingPositions.SHOT_SEED_HUBCLOSE);
    shotSeedChooser.addOption("2M",   Constants.StartingPositions.SHOT_SEED_2M);
    shotSeedChooser.addOption("2.5M", Constants.StartingPositions.SHOT_SEED_2_5M);
    shotSeedChooser.addOption("3M",   Constants.StartingPositions.SHOT_SEED_3M);
    shotSeedChooser.addOption("4M",   Constants.StartingPositions.SHOT_SEED_4M);
    shotSeedChooser.addOption("4.5M", Constants.StartingPositions.SHOT_SEED_4_5M);
    SmartDashboard.putData("Shot Seed Pose", shotSeedChooser);
    
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

    // START: Seed pose selected from "Shot Seed Pose" dropdown in Elastic.
    // Restore to getRebuiltRightCornerPose() for competition.
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .onTrue(Commands.runOnce(() -> {
          Pose2d seed = shotSeedChooser.getSelected();
          if (seed == null) seed = Constants.StartingPositions.SHOT_SEED_2M;
          new SetStartingPoseCommand(seed, "SHOT SEED", gyro, questNav, driveSubsystem, poseEstimator)
              .schedule();
        }));

    // D-PAD DOWN: Toggle QuestNav emergency mode (locks turret to HUBCLOSE, blocks pose-dependent commands).
    new Trigger(() -> driverController.getPOV() == 180)
        .onTrue(Commands.runOnce(this::toggleQuestNavEmergencyMode));

    // D-PAD UP: Toggle turret Phase1 fallback — locks turret forward under PID, hood/flywheel
    // still track distance. Use mid-match if turret aim is wrong but shooting is still needed.
    new Trigger(() -> driverController.getPOV() == 0)
        .onTrue(Commands.runOnce(() -> {
          boolean nowActive = !robotState.isTurretPhase1Fallback();
          robotState.setTurretPhase1Fallback(nowActive);
        }));

    // ========== NORMAL OPERATION BUTTONS (COMMENT OUT FOR SYSID) ==========

    // Reusable gate — all pose-dependent commands check this before starting.
    Trigger poseAvailable = new Trigger(() -> !robotState.isQuestNavEmergencyMode());

    // Y/B/A/X: Wall traversal (face buttons match D-pad compass layout)
    // Y = far wall, B = right wall, A = near wall, X = left wall
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .and(poseAvailable)
        .whileTrue(Commands.deferredProxy(() -> new WallTraversalCommand(poseEstimator, driveSubsystem, WallTraversalCommand.Wall.FAR)));
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .and(poseAvailable)
        .whileTrue(Commands.deferredProxy(() -> new WallTraversalCommand(poseEstimator, driveSubsystem, WallTraversalCommand.Wall.RIGHT)));
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .and(poseAvailable)
        .whileTrue(Commands.deferredProxy(() -> new WallTraversalCommand(poseEstimator, driveSubsystem, WallTraversalCommand.Wall.NEAR)));
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .and(poseAvailable)
        .whileTrue(Commands.deferredProxy(() -> new WallTraversalCommand(poseEstimator, driveSubsystem, WallTraversalCommand.Wall.LEFT)));

    // LEFT TRIGGER (alone): Smart zone sweep based on current zone
    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
        .and(new Trigger(() -> driverController.getRightTriggerAxis() <= 0.5))
        .and(poseAvailable)
        .whileTrue(Commands.deferredProxy(this::createSmartSweepCommand));

    // LEFT/RIGHT BUMPER: Dynamic bump traversal
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .and(poseAvailable)
        .whileTrue(Commands.deferredProxy(() -> createBumpTraversalCommand(DynamicBumpTraversalCommand.Side.LEFT)));
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .and(poseAvailable)
        .whileTrue(Commands.deferredProxy(() -> createBumpTraversalCommand(DynamicBumpTraversalCommand.Side.RIGHT)));

    // BOTH TRIGGERS: Dynamic heading snap — gyro only, safe in emergency mode (no gate needed).
    // Holds a field-relative heading based on zone/position while driver steers with left stick.
    Trigger bothTriggers = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
        .and(new Trigger(() -> driverController.getRightTriggerAxis() > 0.5));
    bothTriggers.whileTrue(new SnapToHeadingDynamic(
        poseEstimator, driveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX()));

    // ========== END NORMAL OPERATION BUTTONS ==========

    // ========== OPERATOR CONTROLLER BINDINGS ==========

    // --- INTAKE ---
    // Y: toggle extend/retract.
    // On deploy: starts rollers if intakeRollersEnabled (via onExtendComplete callback).
    // On retract: always stops rollers regardless of flag.
    new JoystickButton(operatorController, XboxController.Button.kY.value)
        .onTrue(Commands.runOnce(() -> {
          if (intakeSubsystem == null) return;
          RobotState.IntakePosition pos = robotState.getIntakePosition();
          boolean armIsOut = pos == RobotState.IntakePosition.EXTENDED
              || pos == RobotState.IntakePosition.EXTENDING
              || pos == RobotState.IntakePosition.AGITATING
              || pos == RobotState.IntakePosition.BUMP_LIFTING;
          if (armIsOut) {
            intakeSubsystem.stopRollers();
            intakeSubsystem.retract();
          } else {
            intakeSubsystem.extend();
            // rollers start via onExtendComplete callback once arm reaches full extension
          }
        }));
    // X (hold): reverse rollers to spit out.
    new JoystickButton(operatorController, XboxController.Button.kX.value)
        .whileTrue(Commands.startEnd(
          () -> { if (intakeSubsystem != null) intakeSubsystem.spinOut(); },
          () -> { if (intakeSubsystem != null) intakeSubsystem.stopRollers(); }));
    // A (press): agitate — arm retracts to mid-point then re-extends.
    // When arm returns to EXTENDED, onExtendComplete fires and restarts rollers if flag is on.
    new JoystickButton(operatorController, XboxController.Button.kA.value)
        .onTrue(Commands.runOnce(() -> { if (intakeSubsystem != null) intakeSubsystem.agitate(); }));
    // B: toggle global roller enable. Immediately applies the new state to the running rollers.
    // When on: rollers start now if arm is out in any deployed state (not just fully EXTENDED).
    // When off: rollers stop now and stay off until B is pressed again.
    new JoystickButton(operatorController, XboxController.Button.kB.value)
        .onTrue(Commands.runOnce(() -> {
          if (intakeSubsystem == null) return;
          intakeRollersEnabled = !intakeRollersEnabled;
          if (intakeRollersEnabled) {
            RobotState.IntakePosition pos = robotState.getIntakePosition();
            boolean armIsDeployed = true;/*pos == RobotState.IntakePosition.EXTENDED
                || pos == RobotState.IntakePosition.EXTENDING
                || pos == RobotState.IntakePosition.AGITATING
                || pos == RobotState.IntakePosition.BUMP_LIFTING;*/
            if (armIsDeployed) intakeSubsystem.spinIn();
          } else {
            intakeSubsystem.stopRollers();
          }
        }));
    // --- END INTAKE ---

    // --- TURRET FLYWHEELS + SHOOT ---
    // LT (press): toggle flywheels on/off.
    new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5)
        .onTrue(Commands.runOnce(() -> {
          if (turretSubsystem == null) return;
          robotState.setFlywheelOn(!robotState.isFlywheelOn());
          if (!robotState.isFlywheelOn()) turretSubsystem.setFlywheelPercent(0.0);
        }));

    // new Trigger(() -> (operatorController.getLeftTriggerAxis() > 0.1 && operatorController.getLeftTriggerAxis() < 0.5)) // Allow LT to toggle flywheels even when both triggers are pressed (for dynamic snap+shoot)
    //     .onTrue(Commands.runOnce(() -> {
    //       turretSubsystem.setFlywheelPercent(30);
    //     })); //remove soon

    // RT (hold): shoot.
    // If flywheels are already on, feed immediately.
    // If flywheels are off, spin them up first and wait for them to reach speed before feeding.
    // Releases spindexer and singulator when trigger is released.
    new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5)
        .whileTrue(Commands.run(() -> {
          if (turretSubsystem == null) return;

          // If flywheel was off, turn it on now — it will spin up this loop and be checked below
          if (!robotState.isFlywheelOn()) {
            robotState.setFlywheelOn(true);
          }

          // Only start feeding once flywheels are up to speed
          if (!turretSubsystem.isFlywheelSpinningFast()) return;

          boolean spindexerAllowed = intakeSubsystem == null
              || (robotState.getIntakePosition() != RobotState.IntakePosition.EXTENDING
              &&  robotState.getIntakePosition() != RobotState.IntakePosition.RETRACTING);
          if (spindexerSubsystem  != null && spindexerAllowed) spindexerSubsystem.spinForward();
          if (singulatorSubsystem != null) singulatorSubsystem.primeAndFeed();
        }).finallyDo(() -> {
          if (spindexerSubsystem  != null) spindexerSubsystem.stop();
          if (singulatorSubsystem != null) singulatorSubsystem.pause();
        }));
    // --- END TURRET FLYWHEELS + SHOOT ---

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

    // --- TURRET HOOD ---
    // D-pad up: step hood up. D-pad down: step hood down.
    new Trigger(() -> operatorController.getPOV() == 0)
        .onTrue(Commands.runOnce(() -> { if (turretSubsystem != null) turretSubsystem.hoodStepUp(); }));
    new Trigger(() -> operatorController.getPOV() == 180)
        .onTrue(Commands.runOnce(() -> { if (turretSubsystem != null) turretSubsystem.hoodStepDown(); }));
    // --- END TURRET HOOD ---

    // ========== END OPERATOR CONTROLLER BINDINGS ==========

    // Back+Start: emergency re-home — use when turret homing is bad from startup.
    // Manually rotate turret to forward with D-pad L/R first, then press both together.
    new Trigger(() -> operatorController.getBackButton() && operatorController.getStartButton())
        .onTrue(Commands.runOnce(() -> {
          if (turretSubsystem != null) turretSubsystem.home();
          SmartLogger.logConsole("Emergency turret re-home triggered (Back+Start) — hall sweep", "Homing");
        }));
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

  // Called at the start of teleop. If auto already ran and enabled tracking, the turret
  // forward lockout is automatically cleared so operator controls are immediately active.
  // This covers real matches where auto runs before teleop — no button confirm needed.
  // In practice/pit mode (no auto run), the LB+RB confirm is still required.
  public void onTeleopInit() {
    // Reset roller flag to OFF so first B press in teleop always turns rollers ON cleanly.
    // Avoids the case where auto left the flag true and B immediately turns it off.
    intakeRollersEnabled = false;

    if (!Constants.Turret.REQUIRE_TURRET_FORWARD_CONFIRM) return;
    if (bindingsConfigured) return;
    if (turretSubsystem != null && turretSubsystem.isTrackingEnabled()) {
      turretSubsystem.homeForward();
      configureButtonBindings();
      SmartLogger.logConsole("Teleop: auto already ran — turret homed forward, controls active", "Homing");
    }
  }

  // Toggles QuestNav emergency mode on/off.
  // ON: locks turret to HUBCLOSE preset, blocks all pose-dependent drive commands.
  // Operator still controls flywheels and shooting manually via LT/RT as normal.
  // OFF: clears the flag — pose-dependent commands and auto-tracking resume.
  public void toggleQuestNavEmergencyMode() {
    boolean nowActive = !robotState.isQuestNavEmergencyMode();
    robotState.setQuestNavEmergencyMode(nowActive);
    if (nowActive) {
      // Disable tracking so the default command stops overwriting the emergency setpoints every loop.
      if (turretSubsystem != null) {
        turretSubsystem.disableTracking();
        turretSubsystem.activateEmergencyHubClose();
      }
    } else {
      // Re-enable tracking when emergency mode is cleared.
      if (turretSubsystem != null) turretSubsystem.enableTracking();
    }
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

  private void updateAllianceFromDriverStation() {
    Optional<Alliance> fmsAlliance = DriverStation.getAlliance();
    if (fmsAlliance.isPresent()) {
      cachedAlliance = fmsAlliance.get();
    } else {
      // No FMS — infer from seeded pose X. Robot on Red side has X > field midpoint.
      double midX = Constants.Field.FIELD_LENGTH_METERS / 2.0;
      double poseX = poseEstimator.getEstimatedPose().getX();
      cachedAlliance = (poseX > midX) ? Alliance.Red : Alliance.Blue;
    }
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
        DynamicBumpTraversalCommand.createPrototypeConfig(),
        intakeSubsystem);
  }

  private Pose2d getRebuiltRightCornerPose() {
    if (!isRedAlliance()) return BLUE_REBUILT_RIGHT_CORNER;
    // On a full competition field use the mirrored far corner.
    // On the practice field use the dedicated Red seed pose instead.
    return COMPETITION_MODE ? RED_REBUILT_RIGHT_CORNER : RED_PRACTICE_SEED;
  }
}
