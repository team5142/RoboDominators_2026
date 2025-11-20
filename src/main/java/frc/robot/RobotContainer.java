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
import edu.wpi.first.wpilibj2.command.button.POVButton;
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

// RobotContainer: Connects subsystems, controllers, and commands. Created once at robot boot.
public class RobotContainer {
  
  // Controllers
  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT); // Port 0
  // TODO: Add operator controller when manipulator subsystems ready (Port 1 + HTML touch interface)

  // Subsystems
  private final RobotState robotState = new RobotState(); // Tracks robot mode and global state
  private final GyroSubsystem gyro = new GyroSubsystem(); // Manages Pigeon2 + Quest3 gyro with failover
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(robotState, gyro); // CTRE swerve drive
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(driveSubsystem, robotState, gyro); // NEW: Pass gyro
  private final TagVisionSubsystem tagVisionSubsystem = new TagVisionSubsystem(poseEstimator); // AprilTag cameras
  private final ObjectVisionSubsystem objectVisionSubsystem = new ObjectVisionSubsystem(robotState); // Re-enabled!
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(robotState, tagVisionSubsystem); // LED status indicators

  // Autonomous
  private final SendableChooser<Command> autoChooser; // Populated by PathPlanner

  public RobotContainer() {
    configurePathPlanner();
    configureDefaultCommands();
    configureButtonBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
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

  // Returns true if red alliance (flip paths), false if blue
  private boolean shouldFlipPath() {
    return DriverStation.getAlliance().map(alliance -> alliance == Alliance.Red).orElse(false);
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
        .onTrue(new SetStartingPoseCommand(BLUE_REEF_TAG_17, "Blue Reef Tag 17", gyro, driveSubsystem, poseEstimator));

    // X: Save current position to custom slot
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .onTrue(Commands.runOnce(() -> {
          Pose2d currentPose = poseEstimator.getEstimatedPose();
          SavedPositions.saveCustomPosition(currentPose);
          System.out.println("Position saved: " + currentPose);
        }));

    // Y: Auto-drive to Blue Reef Tag 17 (hold to drive, release to stop)
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(new DriveToSavedPosition(BLUE_REEF_TAG_17, "Blue Reef Tag 17"));

    // B: Auto-drive to Processor position
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(new DriveToSavedPosition(PROCESSOR_POS, "Processor Position"));

    // A: Auto-drive to Intake position
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(new DriveToSavedPosition(INTAKE_POS, "Intake Position"));

    System.out.println("Button bindings configured");
  }

  // Public Accessors
  public Command getAutonomousCommand() { return autoChooser.getSelected(); } // Called by Robot.java at auto start
  public RobotState getRobotState() { return robotState; } // Global state tracker
}
