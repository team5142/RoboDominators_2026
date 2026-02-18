package frc.robot;

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.Auto.*;
import static frc.robot.Constants.StartingPositions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.commands.drive.NeutralZoneSweepSimplifiedCommand;
import frc.robot.commands.drive.SmartDriveToPosition;
import frc.robot.commands.util.LogCurrentPoseCommand;
import frc.robot.commands.util.SetStartingPoseCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.turret.TurretIOCTRE;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.SmartLogger;
import frc.robot.util.TouchscreenInterface;

// Wires up robot hardware, controllers, and commands
public class RobotContainer {
  
  // === CONFIGURATION ===
  public static final boolean COMPETITION_MODE = false; // Disable logs/streams for matches
  private static final boolean ENABLE_CONSOLE_LOGGING = !COMPETITION_MODE;
  private static final boolean USE_TOUCHSCREEN_OPERATOR = true;
  private static final boolean SYSID_MODE = false; // Phoenix Tuner X characterization mode
  private static final boolean ENABLE_TURRET = false;
  private static final boolean ENABLE_INTAKE = false;
  private static final boolean ENABLE_CLIMBER = false;
  private static final double AUTO_SEED_POS_TOL_METERS = 0.20;
  private static final double AUTO_SEED_ROT_TOL_DEG = 10.0;

  private static Alliance cachedAlliance = Alliance.Blue;
  
  // Hardware
  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);

  // Subsystems (declared first, initialized in constructor)
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

  // Autonomous
  private final SendableChooser<Command> autoChooser;
  private Command lastSelectedAuto = null; // Track selection for preview updates

  private volatile Pose2d pendingAutoPreviewPose = null;
  private volatile String pendingAutoPreviewName = null;

  private Pose2d lastAppliedPreviewPose = null;
  private String lastAppliedPreviewName = null;

  private volatile boolean previewThreadRunning = true;

  private TouchscreenInterface touchscreen;

  // === CONSTRUCTOR - Runs once at robot boot ===
  public RobotContainer(RobotState robotState) {
    this.robotState = robotState;

    gyro = new GyroSubsystem();
    questNav = new QuestNavSubsystem();
    driveSubsystem = new DriveSubsystem(this.robotState, gyro);
    poseEstimator = new PoseEstimatorSubsystem(driveSubsystem, this.robotState, questNav);
    tagVisionSubsystem = new TagVisionSubsystem(poseEstimator, gyro);
    ledSubsystem = new LEDSubsystem(this.robotState, tagVisionSubsystem);
    turretSubsystem = ENABLE_TURRET ? new TurretSubsystem(this.robotState, new TurretIOCTRE()) : null;
    intakeSubsystem = ENABLE_INTAKE ? new IntakeSubsystem(this.robotState) : null;
    climberSubsystem = ENABLE_CLIMBER ? new ClimberSubsystem(this.robotState) : null;

    if (!ENABLE_TURRET) {
      SmartLogger.logConsole("Turret disabled until hardware is ready", "Startup");
    }
    if (!ENABLE_INTAKE) {
      SmartLogger.logConsole("Intake disabled until hardware is ready", "Startup");
    }
    if (!ENABLE_CLIMBER) {
      SmartLogger.logConsole("Climber disabled until hardware is ready", "Startup");
    }

    SmartLogger.configure(ENABLE_CONSOLE_LOGGING);

    updateAllianceFromDriverStation();
    this.robotState.setAlliance(cachedAlliance);
    
    if (COMPETITION_MODE) {
      SmartLogger.logReplay("Robot/CompetitionMode", true);
    }
    
    poseEstimator.setTagVisionSubsystem(tagVisionSubsystem);
    SmartDriveToPosition.configure(poseEstimator, robotState, driveSubsystem, questNav);
    
    configurePathPlanner();
    registerSmartDriveEvents(); // Register PathPlanner event markers
    configureDefaultCommands();
    configureButtonBindings();
    
    if (USE_TOUCHSCREEN_OPERATOR) {
      configureTouchscreenInterface();
    }
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    robotState.setSysIdMode(SYSID_MODE);
    poseEstimator.setAutoChooser(autoChooser);
    startAutoPreviewMonitor();
    
    SmartLogger.logConsole("RobotContainer initialized - all subsystems ready", "Init Complete", 5);
  }

  // Configure PathPlanner auto builder
  private void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      
      AutoBuilder.configure(
          poseEstimator::getEstimatedPose,
          this::resetPose,
          driveSubsystem::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveSubsystem.driveRobotRelative(speeds),
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

    // D-PAD: PathPlanner PID Tuning Autos
    new Trigger(() -> driverController.getPOV() == 0)
        .whileTrue(AutoBuilder.buildAuto("PathPlannerTuning"));
    new Trigger(() -> driverController.getPOV() == 180)
        .whileTrue(AutoBuilder.buildAuto("PathPlannerTuningReturn"));
    new Trigger(() -> driverController.getPOV() == 90)
        .whileTrue(AutoBuilder.buildAuto("PathPlannerTuningCurve"));

    // ========== END NORMAL OPERATION BUTTONS ==========

    // ========== SYSID CHARACTERIZATION BUTTONS (COMMENT OUT FOR NORMAL OPERATION) ==========
    /*
    // IMPORTANT: Before running SysId tests:
    // 1. Comment out normal operation buttons above
    // 2. For STEER tests: set STEER_FEEDBACK_TYPE = RemoteCANcoder in Constants.java
    // 3. Start SignalLogger with left bumper, stop with right bumper
    // 4. Run all 4 tests in one session: quasistatic fwd/rev, dynamic fwd/rev

    // SIGNAL LOGGER CONTROL
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(Commands.runOnce(() -> com.ctre.phoenix6.SignalLogger.start()));
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .onTrue(Commands.runOnce(() -> com.ctre.phoenix6.SignalLogger.stop()));

    // TRANSLATION TESTS (drive motors)
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(driveSubsystem.sysIdQuasistaticTranslation(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(driveSubsystem.sysIdQuasistaticTranslation(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(driveSubsystem.sysIdDynamicTranslation(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileTrue(driveSubsystem.sysIdDynamicTranslation(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));

    // STEER TESTS (module steering motors) - set STEER_FEEDBACK_TYPE = RemoteCANcoder before running!
    // POV UP: Quasistatic Forward
    new Trigger(() -> driverController.getPOV() == 0)
        .whileTrue(driveSubsystem.sysIdQuasistaticSteer(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    // POV DOWN: Quasistatic Reverse
    new Trigger(() -> driverController.getPOV() == 180)
        .whileTrue(driveSubsystem.sysIdQuasistaticSteer(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
    // POV RIGHT: Dynamic Forward
    new Trigger(() -> driverController.getPOV() == 90)
        .whileTrue(driveSubsystem.sysIdDynamicSteer(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    // POV LEFT: Dynamic Reverse
    new Trigger(() -> driverController.getPOV() == 270)
        .whileTrue(driveSubsystem.sysIdDynamicSteer(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
    */
    // ========== END SYSID BUTTONS ==========

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
  }

  // HTML touchscreen interface
  private void configureTouchscreenInterface() {
    touchscreen = new TouchscreenInterface(robotState, driveSubsystem, poseEstimator, gyro, questNav);
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
    Alliance alliance = getAlliance();
    boolean isRed = isRedAlliance();
    SmartLogger.logConsole("Alliance: " + alliance + " | Flipping: " + isRed, "Path Flip");
    return isRed;
  }

  private void startAutoPreviewMonitor() {
    Thread previewThread = new Thread(() -> {
      while (previewThreadRunning && !Thread.currentThread().isInterrupted()) {
        try {
          if (DriverStation.isDisabled()) {
            Command selectedAuto = autoChooser.getSelected();

            if (selectedAuto != null && selectedAuto != lastSelectedAuto) {
              lastSelectedAuto = selectedAuto;
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

    previewThread.setDaemon(true);  // CRITICAL: Thread dies when robot code exits
    previewThread.setName("AutoPreview");
    previewThread.start();
  }

  public void periodic() {
    updateAllianceFromDriverStation();
    robotState.setAlliance(cachedAlliance);
    if (DriverStation.isDisabled()) {
      applyPendingAutoPreviewPose();
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

    // Reset gyro to match the auto start heading before resetting pose estimator
    gyro.setHeading(pose.getRotation().getDegrees());

    poseEstimator.resetPose(
        pose,
        driveSubsystem.getGyroRotation(),
        driveSubsystem.getModulePositions());

    if (questNavNeedsSeed(pose)) {
      questNav.seedToPose(pose);
    }

    SmartLogger.logConsole("Auto: " + autoName + " | Pose: " + SmartLogger.formatPose(pose), "Preview");
    SmartLogger.logReplay("Auto/PreviewPose", pose);
  }

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

  private Command createSmartSweepCommand() {
    return new NeutralZoneSweepSimplifiedCommand(poseEstimator, driveSubsystem);
  }

  private Command createBumpTraversalCommand(DynamicBumpTraversalCommand.Side side) {
    boolean modifier = driverController.getPOV() == 0; // D-pad up
    return new DynamicBumpTraversalCommand(
        poseEstimator,
        driveSubsystem,
        side,
        modifier,
        DynamicBumpTraversalCommand.createPrototypeConfig());
  }

  private Pose2d getRebuiltRightCornerPose() {
    return isRedAlliance() ? RED_REBUILT_RIGHT_CORNER : BLUE_REBUILT_RIGHT_CORNER;
  }

  // Register SmartDrive as PathPlanner events
  private void registerSmartDriveEvents() {
   // NamedCommands.registerCommand("SmartPrecision:Tag12", 
        //SmartDriveToPosition.createPrecisionPhase(PRECISE_12_POSE));
  
  }
}