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
import edu.wpi.first.wpilibj.Timer; // ADD THIS
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.commands.auto.FollowPreplannedPath;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.SnapToHeadingFixed;
import frc.robot.commands.SetStartingPoseCommand;
import frc.robot.commands.auto.DriveToSavedPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ObjectVisionSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.QuestNavSubsystem; // NEW: Add this import
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

import java.util.Map;
import frc.robot.commands.auto.DriveToSavedPosition; // NEW: Add this import
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

// RobotContainer: Connects subsystems, controllers, and commands. Created once at robot boot.
public class RobotContainer {
  
  // === SYSID MODE TOGGLE ===
  // Set to true when running SysID characterization, false for normal operation
  private static final boolean SYSID_MODE = false; // CHANGED: Back to normal operation
  // =========================
  
  // Controllers
  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT); // Port 0
  // TODO: Add operator controller when manipulator subsystems ready (Port 1 + HTML touch interface)

  // Subsystems
  private final RobotState robotState = new RobotState();
  private final GyroSubsystem gyro = new GyroSubsystem();
  private final QuestNavSubsystem questNav = new QuestNavSubsystem(); // NEW
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(robotState, gyro);
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(driveSubsystem, robotState, gyro, questNav); // CHANGED: Added questNav parameter
  private final TagVisionSubsystem tagVisionSubsystem = new TagVisionSubsystem(poseEstimator, gyro);
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(robotState, tagVisionSubsystem);

  // Autonomous
  private final SendableChooser<Command> autoChooser; // Populated by PathPlanner
  private Command lastSelectedAuto = null; // Track selected auto for pose preview

  // Touchscreen operator interface toggle
  private static final boolean USE_TOUCHSCREEN_OPERATOR = true; // Toggle between touchscreen and Xbox controller

  public RobotContainer() {
    // NEW: Connect TagVision to PoseEstimator (avoids circular dependency)
    poseEstimator.setTagVisionSubsystem(tagVisionSubsystem);
    
    configurePathPlanner();
    configureDefaultCommands();
    configureButtonBindings();
    
    // Configure touchscreen or Xbox operator controls
    if (USE_TOUCHSCREEN_OPERATOR) {
      configureTouchscreenInterface();
    }
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Set SysID mode based on constant
    robotState.setSysIdMode(SYSID_MODE);
    
    // NEW: Share auto chooser with PoseEstimator for validation
    poseEstimator.setAutoChooser(autoChooser);

    // NEW: Monitor auto selection and update pose preview
    startAutoPreviewMonitor();
    
    System.out.println("=== PathPlanner Autos Loaded ===");
    System.out.println("================================");
    
    System.out.println("RobotContainer initialized - all subsystems and bindings ready!");
  }

  // PathPlanner Configuration - Tells PathPlanner how to drive the robot
  private void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings(); // Load physical properties from GUI
      
      AutoBuilder.configure(
          poseEstimator::getEstimatedPose, // Get current position
          this::resetPose, // Reset position
          driveSubsystem::getRobotRelativeSpeeds, // Get current speed
          (speeds, feedforwards) -> driveSubsystem.driveRobotRelative(speeds), // Drive command
          new PPHolonomicDriveController(
              new PIDConstants(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD), // Translation PID
              new PIDConstants(ROTATION_KP, ROTATION_KI, ROTATION_KD)), // Rotation PID
          config, // Robot physical config
          this::shouldFlipPath, // Mirror for red alliance
          driveSubsystem); // Subsystem requirement
      
      System.out.println("PathPlanner configured successfully!");
    } catch (Exception e) {
      System.err.println("Failed to configure PathPlanner:");
      e.printStackTrace();
      DriverStation.reportWarning("PathPlanner config failed - auto may not work!", false);
    }
  }

  /**
   * Wraps a PathPlanner path following command with start/end logging
   * Records accuracy for each leg of the auto
   */
  private Command wrapPathWithLogging(Command pathCommand) {
    // Extract path name if available
    String pathName = "Unknown Path";
    
    // Try to get path name from command name
    if (pathCommand.getName() != null && !pathCommand.getName().isEmpty()) {
      pathName = pathCommand.getName();
    }
    
    final String finalPathName = pathName; // For lambda capture
    
    return pathCommand
        .beforeStarting(() -> {
          // Log start of path segment
          Pose2d startPose = poseEstimator.getEstimatedPose();
          
          System.out.println("========== PATH SEGMENT START ==========");
          System.out.println("Segment: " + finalPathName);
          System.out.println("Start pose: " + formatPose(startPose));
          System.out.println("Time: " + Timer.getFPGATimestamp());
          System.out.println("=======================================");
          
          Logger.recordOutput("Auto/CurrentSegment", finalPathName);
          Logger.recordOutput("Auto/SegmentStart", startPose);
          Logger.recordOutput("Auto/SegmentStartTime", Timer.getFPGATimestamp());
        })
        .finallyDo((interrupted) -> {
          // Log end of path segment with accuracy
          Pose2d endPose = poseEstimator.getEstimatedPose();
          double endTime = Timer.getFPGATimestamp();
          
          System.out.println("========== PATH SEGMENT END ==========");
          System.out.println("Segment: " + finalPathName);
          System.out.println("End pose: " + formatPose(endPose));
          System.out.println("Interrupted: " + interrupted);
          System.out.println("Time: " + endTime);
          
          // TODO: Calculate accuracy vs expected end pose
          // This requires storing target poses from .auto file
          // For now, just log actual end pose
          
          Logger.recordOutput("Auto/SegmentEnd", endPose);
          Logger.recordOutput("Auto/SegmentEndTime", endTime);
          Logger.recordOutput("Auto/SegmentInterrupted", interrupted);
          Logger.recordOutput("Auto/CurrentSegment", "None");
          
          System.out.println("======================================");
        });
  }

  // Returns true if red alliance (flip paths), false if blue
  private boolean shouldFlipPath() {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.map(a -> a == Alliance.Red).orElse(false);
    
    System.out.println("========== PATH FLIPPING DEBUG ==========");
    System.out.println("Alliance detected: " + alliance);
    System.out.println("Is Red alliance: " + isRed);
    System.out.println("Flipping path: " + isRed);
    System.out.println("========================================");
    
    return isRed;
  }

  // Reset robot pose estimate to a specific position
  private void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    System.out.println("Pose reset to: " + pose);
  }

  // Default Commands - Run when subsystems aren't doing anything else
  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(
        new DriveWithJoysticks(
            driveSubsystem, robotState,
            () -> -driverController.getLeftY(), // Forward/back (inverted)
            () -> -driverController.getLeftX(), // Strafe (inverted)
            () -> -driverController.getRightX(), // Rotation (inverted)
            () -> true, // Field-relative
            () -> driverController.getRightBumper())); // Slow mode - FIXED: Use button() instead
  }

  // Button Bindings - Map controller buttons to commands
  private void configureButtonBindings() {
    
    // Only add SysId bindings if in SysId mode
    if (SYSID_MODE) {
      configureSysIdButtons();
      return; // Skip normal button bindings during SysId
    }
    
    // BACK: Reset field orientation (gyro zero)
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(driveSubsystem.createOrientToFieldCommand(robotState));

    // D-PAD: Snap to cardinal directions (hold to maintain heading)
    new POVButton(driverController, 0) // UP: Face 0deg (forward)
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> 0.0));
    new POVButton(driverController, 90) // RIGHT: Face -90deg
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> -90.0));
    new POVButton(driverController, 180) // DOWN: Face 180deg (backward)
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> 180.0));
    new POVButton(driverController, 270) // LEFT: Face 90deg
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> 90.0));

    // START: Set starting position (drive to spot, press to save as starting pose)
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .onTrue(new SetStartingPoseCommand(BLUE_REEF_TAG_17, "Blue Reef Tag 17", gyro, questNav, driveSubsystem, poseEstimator)); // CHANGED: Pass questNav

    // Y: Auto-drive to Blue Tag 17
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(new DriveToSavedPosition(
            BLUE_REEF_TAG_17,
            "Blue Reef Tag 17",
            poseEstimator));

    // B: Auto-drive to Blue Tag 16
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(new DriveToSavedPosition(
            BLUE_TAG_16,
            "Blue Processor Station (Tag 16)",
            poseEstimator));

    // A: Auto-drive to Blue Tag 12
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(new DriveToSavedPosition(
            BLUE_TAG_12,
            "Blue Coral Station (Tag 12)",
            poseEstimator));

    System.out.println("Button bindings configured");
    System.out.println("Y/A/B: Simple PathPlanner pathfinding (QuestNav-only testing)");
  }
  
  private void configureSysIdButtons() {
    System.out.println("=== SysId Button Bindings Active ===");
    System.out.println("Testing: DRIVE MOTORS (Translation)");
    System.out.println("Using CTRE's built-in SysId routines (logs to .hoot file)");
    
    // Use parent class methods - these log to CTRE SignalLogger
    // Y: Quasistatic Forward
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    
    // A: Quasistatic Reverse
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    
    // B: Dynamic Forward
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    
    // X: Dynamic Reverse
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    
    System.out.println("Y = Quasistatic Forward");
    System.out.println("A = Quasistatic Reverse");
    System.out.println("B = Dynamic Forward");
    System.out.println("X = Dynamic Reverse");
    System.out.println("");
    System.out.println("SysId data will be in .hoot file");
    System.out.println("Look for 'SysIdTranslation_State' signal in Phoenix Tuner X");
  }

  private void configureTouchscreenInterface() {
    NetworkTable opTable = NetworkTableInstance.getDefault().getTable("OperatorInterface");
    
    // Map of position names to Pose2d constants
    Map<String, Pose2d> positions = Map.of(
        "BLUE_REEF_TAG_17", BLUE_REEF_TAG_17,
        "BLUE_REEF_TAG_18", BLUE_REEF_TAG_18,
        "BLUE_REEF_TAG_21", BLUE_REEF_TAG_21,
        "BLUE_REEF_TAG_22", BLUE_REEF_TAG_22,
        "BLUE_TAG_16", BLUE_TAG_16,
        "BLUE_TAG_12", BLUE_TAG_12
    );
    
    // Subscribe to drive-to-position commands (2025 API)
    positions.forEach((positionName, pose) -> {
      BooleanSubscriber sub = opTable
          .getSubTable("DriveToPosition")
          .getBooleanTopic(positionName)
          .subscribe(false);
      
      // Poll in periodic (add this to a periodic method or use a notifier)
      new Thread(() -> {
        while (true) {
          if (sub.get()) {
            String displayName = positionName.replace("_", " ").toLowerCase();
            new DriveToSavedPosition(pose, displayName, poseEstimator).schedule();
            System.out.println("[Touchscreen] Drive to: " + displayName);
          }
          try {
            Thread.sleep(50); // Check every 50ms
          } catch (InterruptedException e) {
            break;
          }
        }
      }).start();
    });
    
    // Subscribe to action commands (2025 API)
    BooleanSubscriber orientFieldSub = opTable
        .getSubTable("Action")
        .getBooleanTopic("OrientToField")
        .subscribe(false);
    
    new Thread(() -> {
      while (true) {
        if (orientFieldSub.get()) {
          driveSubsystem.createOrientToFieldCommand(robotState).schedule();
          System.out.println("[Touchscreen] Orient to Field");
        }
        try {
          Thread.sleep(50);
        } catch (InterruptedException e) {
          break;
        }
      }
    }).start();
    
    BooleanSubscriber setReef17Sub = opTable
        .getSubTable("Action")
        .getBooleanTopic("SetReef17")
        .subscribe(false);
    
    new Thread(() -> {
      while (true) {
        if (setReef17Sub.get()) {
          new SetStartingPoseCommand(BLUE_REEF_TAG_17, "Blue Reef Tag 17", gyro, questNav, driveSubsystem, poseEstimator).schedule(); // FIXED: Added questNav parameter
          System.out.println("[Touchscreen] Set Reef 17 Start Position");
        }
        try {
          Thread.sleep(50);
        } catch (InterruptedException e) {
          break;
        }
      }
    }).start();

    
    // Publish robot state to touchscreen (2025 API)
    BooleanPublisher connectedPub = opTable
        .getBooleanTopic("RobotConnected")
        .publish();
    
    BooleanPublisher enabledPub = opTable
        .getBooleanTopic("RobotEnabled")
        .publish();
    
    new Thread(() -> {
      while (true) {
        connectedPub.set(true);
        enabledPub.set(robotState.isEnabled());
        
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
          break;
        }
      }
    }).start();
    
  System.out.println("Touchscreen operator interface configured");
}

private void configureSysIdCommands() {
    System.out.println("=== SysId Commands Available ===");
    System.out.println("Check SmartDashboard for SysId buttons");
    
    // Translation characterization (drive motors)
    SmartDashboard.putData("SysId/Quasistatic Forward", 
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Quasistatic Reverse", 
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Dynamic Forward", 
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Dynamic Reverse", 
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  // Public Accessors
  public Command getAutonomousCommand() { 
    Command selectedAuto = autoChooser.getSelected();
    // Wrap auto command with logging if it exists
    if (selectedAuto != null) {
      return wrapPathWithLogging(selectedAuto);
    }
    return selectedAuto;
  }
  

  /**
   * Monitors the auto chooser and updates the robot pose estimate
   * to match the selected auto's starting position (for AdvantageScope preview)
   */
  private void startAutoPreviewMonitor() {
    new Thread(() -> {
      while (true) {
        try {
          // Only update when disabled (not during auto or teleop)
          if (DriverStation.isDisabled()) {
            Command selectedAuto = autoChooser.getSelected();
            
            // Check if selection changed
            if (selectedAuto != null && selectedAuto != lastSelectedAuto) {
              lastSelectedAuto = selectedAuto;
              
              String autoName = selectedAuto.getName();
              System.out.println("[Auto Preview] Selected: " + autoName);
              
              // Try to get starting pose from known autos
              Pose2d startingPose = getStartingPoseForAuto(autoName);
              
              if (startingPose != null) {
                // Set pose estimate to auto's starting position
                poseEstimator.resetPose(
                    startingPose,
                    driveSubsystem.getGyroRotation(),
                    driveSubsystem.getModulePositions());
                
                System.out.println("[Auto Preview] Pose set to: " + startingPose);
                Logger.recordOutput("Auto/PreviewPose", startingPose);
              }
            }
          }
          
          Thread.sleep(500); // Check every 500ms
        } catch (Exception e) {
          System.err.println("[Auto Preview] Error: " + e.getMessage());
        }
      }
    }).start();
  }

  /**
   * Format a Pose2d for readable console output
   */
  private String formatPose(Pose2d pose) {
    return String.format("(%.2f, %.2f, %.1f°)",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  /**
   * Get the starting pose for a known auto routine
   * Returns null if auto not recognized
   */
  private Pose2d getStartingPoseForAuto(String autoName) {
    // Map of auto names to their starting poses
    // These should match the starting positions defined in your .auto files
    switch (autoName.toLowerCase()) {
      case "leftside1piece":
      case "leftside3piece":
        // Both start at (7.20, 0.45, 180°)
        return new Pose2d(7.20, 0.45, Rotation2d.fromDegrees(180.0));
      
      // Add more autos as you create them
      case "rightside1piece":
        // Example: Right side starting position
        return new Pose2d(7.20, 5.50, Rotation2d.fromDegrees(180.0));
      
      default:
        System.err.println("[Auto Preview] Unknown auto: " + autoName);
        return null;
    }
  }
}
