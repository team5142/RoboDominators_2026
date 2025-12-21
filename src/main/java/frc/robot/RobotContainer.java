package frc.robot;

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.Auto.*;
import static frc.robot.Constants.StartingPositions.*;

import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.SetStartingPoseCommand;
import frc.robot.commands.drive.SmartDriveToPosition;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.subsystems.*;
import frc.robot.util.SmartLogger;
import frc.robot.util.TouchscreenInterface;

// Wires up subsystems, controllers, and commands - runs once at robot startup
public class RobotContainer {
  
  // === CONFIGURATION TOGGLES ===
  private static final boolean ENABLE_CONSOLE_LOGGING = true;  // System.out for live debugging
  private static final boolean ENABLE_REPLAY_LOGGING = true;   // AdvantageKit for post-match replay
  private static final boolean USE_TOUCHSCREEN_OPERATOR = true; // Enable HTML touchscreen interface
  private static final boolean SYSID_MODE = false; // SysId characterization mode (use Phoenix Tuner X)
  // ============================
  
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
    // 1. Configure logging first (everything else may use it)
    SmartLogger.configure(ENABLE_CONSOLE_LOGGING, ENABLE_REPLAY_LOGGING);
    
    // 2. Link subsystems that reference each other
    poseEstimator.setTagVisionSubsystem(tagVisionSubsystem);
    SmartDriveToPosition.configure(poseEstimator, robotState, driveSubsystem, questNav);
    
    // 3. Configure robot behavior
    configurePathPlanner();
    configureDefaultCommands();
    configureButtonBindings();
    
    // 4. Optional: Touchscreen operator interface
    if (USE_TOUCHSCREEN_OPERATOR) {
      configureTouchscreenInterface();
    }
    
    // 5. Setup autonomous chooser and monitoring
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    robotState.setSysIdMode(SYSID_MODE);
    poseEstimator.setAutoChooser(autoChooser);
    startAutoPreviewMonitor();
    
    SmartLogger.logConsole("RobotContainer initialized - all subsystems ready", "Init Complete", 5);
  }

  // === PATHPLANNER SETUP - Configures autonomous path following ===
  private void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      
      AutoBuilder.configure(
          poseEstimator::getEstimatedPose, // Where are we?
          this::resetPose, // Force robot to specific position
          driveSubsystem::getRobotRelativeSpeeds, // How fast are we moving?
          (speeds, feedforwards) -> driveSubsystem.driveRobotRelative(speeds), // Drive command
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
          this::shouldFlipPath, // Mirror paths for red alliance
          driveSubsystem);
      
      SmartLogger.logConsole("PathPlanner configured - PID gains tunable in AdvantageScope", "PathPlanner Ready");
    } catch (Exception e) {
      SmartLogger.logConsoleError("Failed to configure PathPlanner: " + e.getMessage());
      DriverStation.reportWarning("PathPlanner config failed!", false);
    }
  }

  // === DEFAULT COMMANDS - Run when subsystems are idle ===
  private void configureDefaultCommands() {
    // DriveSubsystem: Driver control with joysticks
    driveSubsystem.setDefaultCommand(
        new DriveWithJoysticks(
            driveSubsystem, robotState,
            () -> -driverController.getLeftY(), // Forward/back (inverted for pilot preference)
            () -> -driverController.getLeftX(), // Strafe left/right (inverted)
            () -> -driverController.getRightX(), // Rotate (inverted)
            () -> true, // Field-relative (forward is always downfield)
            () -> driverController.getRightBumper())); // Slow mode toggle
  }

  // === BUTTON BINDINGS - Map controller buttons to robot actions ===
  private void configureButtonBindings() {
    // BACK: Reset field orientation (current direction = 0° downfield)
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(driveSubsystem.createOrientToFieldCommand(robotState));

    // START: Save current position as starting pose (for testing/tuning)
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .onTrue(new SetStartingPoseCommand(PID_TUNING_POSITION, "PID TUNER POSITION", gyro, questNav, driveSubsystem, poseEstimator));

    // Y: Drive to Blue Reef Tag 17 (hybrid PathPlanner + AutoPilot)
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_REEF_TAG_17, PRECISE_17_POSE));

    // B: Drive to Blue Tag 16 (processor station)
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_TAG_16, PRECISE_16_POSE));

    // A: Drive to Blue Tag 12 (coral station)
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(SmartDriveToPosition.create(BLUE_TAG_12, PRECISE_12_POSE));

    // X: Execute TuneStart180 path (for PID tuning)
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

    String bindingsMsg = "D-Pad: AVAILABLE\n" +
                        "Y: SmartDrive Tag 17\n" +
                        "B: SmartDrive Tag 16\n" +
                        "A: SmartDrive Tag 12\n" +
                        "X: PathPlanner PID tuning";
    SmartLogger.logConsole(bindingsMsg, "Button Bindings", 5);
  }

  // === TOUCHSCREEN INTERFACE - HTML dashboard controls ===
  private void configureTouchscreenInterface() {
    TouchscreenInterface touchscreen = new TouchscreenInterface(
        robotState, driveSubsystem, poseEstimator, gyro, questNav);
    touchscreen.configure();
  }

  // === AUTONOMOUS COMMAND - Called when auto starts ===
  public Command getAutonomousCommand() { 
    Command selectedAuto = autoChooser.getSelected();
    return (selectedAuto != null) ? wrapPathWithLogging(selectedAuto) : selectedAuto;
  }

  // === UTILITY METHODS ===

  // Reset robot pose to specific position (for auto start or manual reset)
  private void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    SmartLogger.logConsole("Pose reset to: " + formatPose(pose));
  }

  // Add logging to path commands (tracks start/end pose for each auto segment)
  private Command wrapPathWithLogging(Command pathCommand) {
    String pathName = (pathCommand.getName() != null && !pathCommand.getName().isEmpty()) 
        ? pathCommand.getName() 
        : "Unknown Path";
    
    final String finalPathName = pathName;
    
    return pathCommand
        .beforeStarting(() -> {
          Pose2d startPose = poseEstimator.getEstimatedPose();
          String startMsg = "Segment: " + finalPathName + "\n" +
                           "Start: " + formatPose(startPose) + "\n" +
                           "Time: " + Timer.getFPGATimestamp();
          SmartLogger.logConsole(startMsg, "Path Start", 5);
          SmartLogger.logReplay("Auto/CurrentSegment", finalPathName);
          SmartLogger.logReplay("Auto/SegmentStart", startPose);
          SmartLogger.logReplay("Auto/SegmentStartTime", Timer.getFPGATimestamp());
        })
        .finallyDo((interrupted) -> {
          Pose2d endPose = poseEstimator.getEstimatedPose();
          double endTime = Timer.getFPGATimestamp();
          
          String endMsg = "Segment: " + finalPathName + "\n" +
                         "End: " + formatPose(endPose) + "\n" +
                         "Interrupted: " + interrupted + "\n" +
                         "Time: " + endTime;
          SmartLogger.logConsole(endMsg, "Path End", 5);
          
          SmartLogger.logReplay("Auto/SegmentEnd", endPose);
          SmartLogger.logReplay("Auto/SegmentEndTime", endTime);
          SmartLogger.logReplay("Auto/SegmentInterrupted", interrupted);
          SmartLogger.logReplay("Auto/CurrentSegment", "None");
        });
  }

  // Check if paths should be mirrored for red alliance
  private boolean shouldFlipPath() {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.map(a -> a == Alliance.Red).orElse(false);
    
    String flipMsg = "Alliance: " + alliance + "\n" +
                    "Is Red: " + isRed + "\n" +
                    "Flipping: " + isRed;
    SmartLogger.logConsole(flipMsg, "Path Flip", 5);
    
    return isRed;
  }

  // Background thread: Update robot pose preview when auto selection changes
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
                SmartLogger.logConsole("Auto: " + autoName + " | Pose: " + formatPose(startingPose), "Auto Preview");
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

  // Format pose for console output (x, y, angle)
  private String formatPose(Pose2d pose) {
    return String.format("(%.2f, %.2f, %.1f°)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  // Get starting pose for known autonomous routines
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

  // === PERIODIC - Runs every robot loop (~20ms) ===
  public void periodic() {
    // Check if PathPlanner PID gains changed in AdvantageScope
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
        
        String pidMsg = "Translation: kP=" + TunablePathPlannerPID.TRANSLATION_KP.get() + "\n" +
                       "Rotation: kP=" + TunablePathPlannerPID.ROTATION_KP.get() + "\n" +
                       "Restart path (X) to apply";
        SmartLogger.logConsole(pidMsg, "PID Updated");
      } catch (Exception e) {
        SmartLogger.logConsoleError("Failed to reconfigure PathPlanner PID: " + e.getMessage());
      }
    }
  }
}
