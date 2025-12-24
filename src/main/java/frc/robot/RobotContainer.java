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

// Wires up robot hardware, controllers, and commands
public class RobotContainer {
  
  // === CONFIGURATION ===
  public static final boolean COMPETITION_MODE = false; // Disable logs/streams for matches
  private static final boolean ENABLE_CONSOLE_LOGGING = !COMPETITION_MODE;
  private static final boolean USE_TOUCHSCREEN_OPERATOR = true;
  private static final boolean SYSID_MODE = false; // Phoenix Tuner X characterization mode
  
  // Hardware
  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);

  // Subsystems (created in order of dependency)
  final RobotState robotState = new RobotState();
  final GyroSubsystem gyro = new GyroSubsystem();
  final QuestNavSubsystem questNav = new QuestNavSubsystem();
  final DriveSubsystem driveSubsystem = new DriveSubsystem(robotState, gyro);
  final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(driveSubsystem, robotState, gyro, questNav);
  final TagVisionSubsystem tagVisionSubsystem = new TagVisionSubsystem(poseEstimator, gyro);
  final LEDSubsystem ledSubsystem = new LEDSubsystem(robotState, tagVisionSubsystem);

  // Autonomous
  private final SendableChooser<Command> autoChooser;
  private Command lastSelectedAuto = null; // Track selection for preview updates

  // === CONSTRUCTOR - Runs once at robot boot ===
  public RobotContainer() {
    SmartLogger.configure(ENABLE_CONSOLE_LOGGING);
    
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
            () -> driverController.getRightBumper()));
  }

  // Map controller buttons to commands
  private void configureButtonBindings() {
    // BACK: Reset field orientation
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(driveSubsystem.createOrientToFieldCommand(robotState));

    // START: Save current position
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .onTrue(new SetStartingPoseCommand(PID_TUNING_POSITION, "PID TUNER", gyro, questNav, driveSubsystem, poseEstimator));

    // BOTH TRIGGERS: Log pose (hold fully)
    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.9 && 
                      driverController.getRightTriggerAxis() > 0.9)
        .onTrue(new LogCurrentPoseCommand(poseEstimator, "LOGGED_POSITION"));

    // Y/B/A: SmartDrive to tags
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_REEF_TAG_17, PRECISE_17_POSE));
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_TAG_16, PRECISE_16_POSE));
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_TAG_12, PRECISE_12_POSE));

    // X: PathPlanner PID tuning path
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .onTrue(Commands.runOnce(() -> {
          try {
            PathPlannerPath tunePath = PathPlannerPath.fromPathFile("TuneStart180");
            AutoBuilder.followPath(tunePath).schedule();
            SmartLogger.logConsole("Executing TuneStart180 for PID tuning");
          } catch (Exception e) {
            SmartLogger.logConsoleError("Failed to load TuneStart180.path: " + e.getMessage());
          }
        }));

    SmartLogger.logConsole("Y/B/A: SmartDrive tags | X: Tune path | Triggers: Log pose", "Controls");
  }

  // HTML touchscreen interface
  private void configureTouchscreenInterface() {
    TouchscreenInterface touchscreen = new TouchscreenInterface(
        robotState, driveSubsystem, poseEstimator, gyro, questNav);
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
    SmartLogger.logConsole("Pose reset to: " + formatPose(pose));
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
          SmartLogger.logConsole("Segment: " + finalPathName + " | Start: " + formatPose(startPose), "Path Start");
          SmartLogger.logReplay("Auto/CurrentSegment", finalPathName);
          SmartLogger.logReplay("Auto/SegmentStart", startPose);
        })
        .finallyDo((interrupted) -> {
          Pose2d endPose = poseEstimator.getEstimatedPose();
          SmartLogger.logConsole("Segment: " + finalPathName + " | End: " + formatPose(endPose) + " | Interrupted: " + interrupted, "Path End");
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

  // Monitor auto selection for pose preview
  private void startAutoPreviewMonitor() {
    new Thread(() -> {
      while (true) {
        try {
          if (DriverStation.isDisabled()) {
            Command selectedAuto = autoChooser.getSelected();
            
            if (selectedAuto != null && selectedAuto != lastSelectedAuto) {
              lastSelectedAuto = selectedAuto;
              String autoName = selectedAuto.getName();
              Pose2d startingPose = getStartingPoseForAuto(autoName);
              
              if (startingPose != null) {
                poseEstimator.resetPose(startingPose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
                SmartLogger.logConsole("Auto: " + autoName + " | Pose: " + formatPose(startingPose), "Preview");
                SmartLogger.logReplay("Auto/PreviewPose", startingPose);
              }
            }
          }
          Thread.sleep(500);
        } catch (Exception e) {
          SmartLogger.logConsoleError("[Auto Preview] Error: " + e.getMessage());
        }
      }
    }).start();
  }

  private String formatPose(Pose2d pose) {
    return String.format("(%.2f, %.2f, %.1f°)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  // Map auto name to starting pose
  private Pose2d getStartingPoseForAuto(String autoName) {
    switch (autoName.toLowerCase()) {
      case "leftside1piece":
      case "leftside3piece":
        return new Pose2d(7.20, 0.45, Rotation2d.fromDegrees(180.0));
      case "rightside1piece":
        return new Pose2d(7.20, 5.50, Rotation2d.fromDegrees(180.0));
      default:
        SmartLogger.logConsoleError("[Auto Preview] Unknown auto: " + autoName);
        return null;
    }
  }

  // Runs every 20ms
  public void periodic() {
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
