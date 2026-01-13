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
import frc.robot.commands.drive.SmartDriveToPosition;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.util.LogCurrentPoseCommand;
import frc.robot.commands.util.SetStartingPoseCommand;
import frc.robot.subsystems.*;
import frc.robot.util.SmartLogger;
import frc.robot.util.TouchscreenInterface;
import frc.robot.util.LimelightHelpers;

// Wires up robot hardware, controllers, and commands
public class RobotContainer {
  
  // === CONFIGURATION ===
  public static final boolean COMPETITION_MODE = false; // Disable logs/streams for matches
  private static final boolean ENABLE_CONSOLE_LOGGING = !COMPETITION_MODE;
  private static final boolean USE_TOUCHSCREEN_OPERATOR = true;
  private static final boolean SYSID_MODE = false; // Phoenix Tuner X characterization mode
  private static final double AUTO_SEED_POS_TOL_METERS = 0.20;
  private static final double AUTO_SEED_ROT_TOL_DEG = 10.0;
  
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
  final ShooterSubsystem shooterSubsystem; // NEW: Make it a field
  final TurretSubsystem turretSubsystem;   // NEW: Add turret subsystem

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
    
    // Initialize shooter and turret subsystems
    shooterSubsystem = new ShooterSubsystem();
    shooterSubsystem.setPoseEstimator(poseEstimator);
    
    turretSubsystem = new TurretSubsystem();
    turretSubsystem.setPoseEstimator(poseEstimator);
    turretSubsystem.setShooter(shooterSubsystem);
    
    SmartLogger.configure(ENABLE_CONSOLE_LOGGING);
    
    if (COMPETITION_MODE) {
      SmartLogger.logReplay("Robot/CompetitionMode", true);
    }
    
    poseEstimator.setTagVisionSubsystem(tagVisionSubsystem);
    SmartDriveToPosition.configure(poseEstimator, robotState, driveSubsystem, questNav);
    
    configurePathPlanner();
    registerSmartDriveEvents();
    configureDefaultCommands(); // This will set turret auto-tracking
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
                  TunablePathPlannerPID.TRANSLATION_KP.get(), 
                  TunablePathPlannerPID.TRANSLATION_KI.get(), 
                  TunablePathPlannerPID.TRANSLATION_KD.get()),
              new PIDConstants(
                  TunablePathPlannerPID.ROTATION_KP.get(), 
                  TunablePathPlannerPID.ROTATION_KI.get(), 
                  TunablePathPlannerPID.ROTATION_KD.get())),
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
            () -> driverController.getLeftBumper()));
    
    // Turret continuously tracks hub when in shooting zone
    turretSubsystem.setDefaultCommand(turretSubsystem.createAutoTrackCommand());
  }

  // Map controller buttons to commands
  private void configureButtonBindings() {
    // BACK: Reset field orientation
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(driveSubsystem.createOrientToFieldCommand(robotState));

    // START: Save current position
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .onTrue(new SetStartingPoseCommand(PID_TUNING_POSITION, "PID TUNER", gyro, questNav, driveSubsystem, poseEstimator));

    // LEFT TRIGGER: Random LED color (only works when enabled)
    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
        .onTrue(Commands.runOnce(() -> ledSubsystem.setRandomColor(), ledSubsystem));

    // BOTH TRIGGERS: Log robot pose (hold fully)
    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.9 && 
                      driverController.getRightTriggerAxis() > 0.9)
        .onTrue(new LogCurrentPoseCommand(poseEstimator, "LOGGED_POSITION"));

    // BOTH BUMPERS: Log Limelight pose
    // Disabled for now. Both bumpers is used as operator SmartDrive interrupt.
    // new Trigger(() -> driverController.getLeftBumper() && driverController.getRightBumper())
    //     .onTrue(Commands.runOnce(() -> {
    //       Pose2d limelightPose = tagVisionSubsystem.getLatestPose();
    //       if (limelightPose != null) {
    //         SmartLogger.logConsole("========== LIMELIGHT POSE ==========", "Limelight");
    //         SmartLogger.logConsole("Pose: " + SmartLogger.formatPose(limelightPose), "Limelight");
    //         SmartLogger.logConsole("X: " + String.format("%.4f", limelightPose.getX()) + " meters", "Limelight");
    //         SmartLogger.logConsole("Y: " + String.format("%.4f", limelightPose.getY()) + " meters", "Limelight");
    //         SmartLogger.logConsole("Rotation: " + String.format("%.2f", limelightPose.getRotation().getDegrees()) + " degrees", "Limelight");
    //         SmartLogger.logConsole("===================================", "Limelight");
    //         SmartLogger.logReplay("Limelight/CapturedPose", limelightPose);
    //       } else {
    //         SmartLogger.logConsoleError("No Limelight pose available - check camera connection");
    //       }
    //     }));

    // Y/B/A/X/Stick: SmartDrive to tags
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_REEF_TAG_21, PRECISE_21_POSE));
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_REEF_TAG_22, PRECISE_22_POSE));
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_REEF_TAG_17, PRECISE_17_POSE));
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_REEF_TAG_18, PRECISE_18_POSE));
    new JoystickButton(driverController, XboxController.Button.kRightStick.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_AUTO_START_POS_FAR_RIGHT, PRECISE_BLUE_AUTO_START_POS_FAR_RIGHT));
    // RIGHT BUMPER - PROCESSOR
    new Trigger(() -> driverController.getRightBumper())
        .whileTrue(SmartDriveToPosition.create(BLUE_TAG_16, PRECISE_16_POSE));
    // RIGHT TRIGGER - CORAL STATION
    new Trigger(() -> driverController.getRightTriggerAxis() > 0.9)
        .whileTrue(SmartDriveToPosition.create(BLUE_TAG_12, PRECISE_12_POSE));

    // Driver interrupt for operator-triggered SmartDrive
    new Trigger(() -> driverController.getLeftBumper() && driverController.getRightBumper())
        .onTrue(Commands.runOnce(() -> {
          if (touchscreen != null) {
            touchscreen.cancelActiveOperatorDrive();
          }
        }));

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
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.map(a -> a == Alliance.Red).orElse(false);
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
    if (DriverStation.isDisabled()) {
      applyPendingAutoPreviewPose();
    }

    // Re-apply PathPlanner PID if changed in AdvantageScope
    if (TunablePathPlannerPID.hasChanged()) {
      try {
        RobotConfig config = RobotConfig.fromGUISettings();
        
        AutoBuilder.configure(
            poseEstimator::getEstimatedPose,
            this::resetPose,
            driveSubsystem::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveSubsystem.driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(
                    TunablePathPlannerPID.TRANSLATION_KP.get(), 
                    TunablePathPlannerPID.TRANSLATION_KI.get(), 
                    TunablePathPlannerPID.TRANSLATION_KD.get()),
                new PIDConstants(
                    TunablePathPlannerPID.ROTATION_KP.get(), 
                    TunablePathPlannerPID.ROTATION_KI.get(), 
                    TunablePathPlannerPID.ROTATION_KD.get())),
            config,
            this::shouldFlipPath,
            driveSubsystem);
        
        SmartLogger.logConsole("Translation kP=" + TunablePathPlannerPID.TRANSLATION_KP.get() + 
                              " | Rotation kP=" + TunablePathPlannerPID.ROTATION_KP.get() + 
                              " | Restart path (X) to apply", "PID Updated");
      } catch (Exception e) {
        SmartLogger.logConsoleError("Failed to reconfigure PID: " + e.getMessage());
      }
    }
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

  // Register SmartDrive as PathPlanner events
  private void registerSmartDriveEvents() {
    NamedCommands.registerCommand("SmartPrecision:Tag12", 
        SmartDriveToPosition.createPrecisionPhase(PRECISE_12_POSE));
    NamedCommands.registerCommand("SmartPrecision:Tag16", 
        SmartDriveToPosition.createPrecisionPhase(PRECISE_16_POSE));
    NamedCommands.registerCommand("SmartPrecision:Tag17", 
        SmartDriveToPosition.createPrecisionPhase(PRECISE_17_POSE));
    NamedCommands.registerCommand("SmartPrecision:Tag18", 
        SmartDriveToPosition.createPrecisionPhase(PRECISE_18_POSE));
    NamedCommands.registerCommand("SmartPrecision:Tag21", 
        SmartDriveToPosition.createPrecisionPhase(PRECISE_21_POSE));
    NamedCommands.registerCommand("SmartPrecision:Tag22", 
        SmartDriveToPosition.createPrecisionPhase(PRECISE_22_POSE));
    
    SmartLogger.logConsole("SmartDrive events ready for PathPlanner (6 targets)", "Events");
    SmartLogger.logReplay("SmartDrive/EventsRegistered", 6.0);
  }
}