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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ObjectVisionSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.auto.DriveToSavedPosition;

public class RobotContainer {
  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);

  private final RobotState robotState = new RobotState();
  private final GyroSubsystem gyro = new GyroSubsystem();

  // ✅ Simplified - CTRE CommandSwerveDrivetrain handles Tuner X constants internally
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(robotState, gyro);
  
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(
      driveSubsystem,
      robotState);
  
  private final TagVisionSubsystem tagVisionSubsystem = new TagVisionSubsystem(poseEstimator);
  private final ObjectVisionSubsystem objectVisionSubsystem = new ObjectVisionSubsystem(robotState);
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(robotState, tagVisionSubsystem);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configurePathPlanner();
    configureDefaultCommands();
    configureButtonBindings();
    
    // Build auto chooser AFTER PathPlanner is configured
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configurePathPlanner() {
    try {
      // Load robot configuration from PathPlanner GUI
      RobotConfig config = RobotConfig.fromGUISettings();
      
      // Configure AutoBuilder
      AutoBuilder.configure(
          poseEstimator::getEstimatedPose,  // Pose supplier
          this::resetPose,                  // Pose reset consumer
          driveSubsystem::getRobotRelativeSpeeds,  // ChassisSpeeds supplier
          (speeds, feedforwards) -> driveSubsystem.driveRobotRelative(speeds),  // Drive consumer
          new PPHolonomicDriveController(
              new PIDConstants(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD),
              new PIDConstants(ROTATION_KP, ROTATION_KI, ROTATION_KD)),
          config,
          this::shouldFlipPath,  // Mirror path based on alliance
          driveSubsystem
      );
      
      System.out.println("PathPlanner configured successfully!");
    } catch (Exception e) {
      System.err.println("Failed to configure PathPlanner: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /**
   * Determine if path should be flipped based on alliance color
   */
  private boolean shouldFlipPath() {
    return DriverStation.getAlliance()
        .map(alliance -> alliance == Alliance.Red)
        .orElse(false);
  }

  /**
   * Reset pose with validation
   */
  private void resetPose(Pose2d pose) {
    poseEstimator.resetPose(
        pose,
        driveSubsystem.getGyroRotation(),
        driveSubsystem.getModulePositions());
  }

  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(
        new DriveWithJoysticks(
            driveSubsystem,
            robotState,
            () -> -driverController.getLeftY(),  // ✅ REMOVED * 0.4
            () -> -driverController.getLeftX(),  // ✅ REMOVED * 0.4
            () -> -driverController.getRightX(), // ✅ REMOVED * 0.4
            () -> true,
            () -> driverController.getRightBumper()));
  }

  private void configureButtonBindings() {
    // Back button: orient to field (manual heading reset)
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(driveSubsystem.createOrientToFieldCommand(robotState));

    // D-pad up/down/left/right for fixed heading snaps
    new POVButton(driverController, 0)
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> 0.0));

    new POVButton(driverController, 90)
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -90.0));

    new POVButton(driverController, 180)
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> 180.0));

    new POVButton(driverController, 270)
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> 90.0));

    // Start button: Set starting position for Blue Reef Tag 17
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .onTrue(new SetStartingPoseCommand(
            BLUE_REEF_TAG_17,
            "Blue Reef Tag 17",
            gyro,
            driveSubsystem,
            poseEstimator));

    // ✅ X Button: Save current position as custom waypoint
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .onTrue(Commands.runOnce(() -> {
          Pose2d currentPose = poseEstimator.getEstimatedPose();
          SavedPositions.saveCustomPosition(currentPose);
        }));

    // ✅ Y Button: Drive to Blue Reef Tag 17 (HOLD to drive, release to stop)
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(new DriveToSavedPosition(BLUE_REEF_TAG_17, "Blue Reef Tag 17"));

    // ✅ B Button: Drive to Processor Position (HOLD to drive, release to stop)
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(new DriveToSavedPosition(PROCESSOR_POS, "Processor Position"));

    // ✅ A Button: Drive to Intake Position (HOLD to drive, release to stop)
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(new DriveToSavedPosition(INTAKE_POS, "Intake Position"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public RobotState getRobotState() {
    return robotState;
  }
}
