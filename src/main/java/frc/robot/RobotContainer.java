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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.SnapToHeadingFixed;
import frc.robot.commands.SetStartingPoseCommand;
import frc.robot.commands.auto.DriveToSavedPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

import java.util.Map;
import frc.robot.commands.drive.SmartDriveToPosition;

public class RobotContainer {
  
  private static final boolean SYSID_MODE = false;
  
  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);

  private final RobotState robotState = new RobotState();
  private final GyroSubsystem gyro = new GyroSubsystem();
  private final QuestNavSubsystem questNav = new QuestNavSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(robotState, gyro);
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(driveSubsystem, robotState, gyro, questNav);
  private final TagVisionSubsystem tagVisionSubsystem = new TagVisionSubsystem(poseEstimator, gyro);
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(robotState, tagVisionSubsystem);

  private final SendableChooser<Command> autoChooser;
  private Command lastSelectedAuto = null;

  private static final boolean USE_TOUCHSCREEN_OPERATOR = true;

  public RobotContainer() {
    poseEstimator.setTagVisionSubsystem(tagVisionSubsystem);
    
    configurePathPlanner();
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
    
    System.out.println("=== PathPlanner Autos Loaded ===");
    System.out.println("================================");
    System.out.println("RobotContainer initialized - all subsystems and bindings ready!");
  }

  private void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      
      AutoBuilder.configure(
          poseEstimator::getEstimatedPose,
          this::resetPose,
          driveSubsystem::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveSubsystem.driveRobotRelative(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD),
              new PIDConstants(ROTATION_KP, ROTATION_KI, ROTATION_KD)),
          config,
          this::shouldFlipPath,
          driveSubsystem);
      
      System.out.println("PathPlanner configured successfully!");
    } catch (Exception e) {
      System.err.println("Failed to configure PathPlanner:");
      e.printStackTrace();
      DriverStation.reportWarning("PathPlanner config failed - auto may not work!", false);
    }
  }

  private Command wrapPathWithLogging(Command pathCommand) {
    String pathName = "Unknown Path";
    
    if (pathCommand.getName() != null && !pathCommand.getName().isEmpty()) {
      pathName = pathCommand.getName();
    }
    
    final String finalPathName = pathName;
    
    return pathCommand
        .beforeStarting(() -> {
          Pose2d startPose = poseEstimator.getEstimatedPose();
          Logger.recordOutput("Auto/CurrentSegment", finalPathName);
          Logger.recordOutput("Auto/SegmentStart", startPose);
          Logger.recordOutput("Auto/SegmentStartTime", Timer.getFPGATimestamp());
        })
        .finallyDo((interrupted) -> {
          Pose2d endPose = poseEstimator.getEstimatedPose();
          double endTime = Timer.getFPGATimestamp();
        });
  }

  private boolean shouldFlipPath() {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.map(a -> a == Alliance.Red).orElse(false);
    return isRed;
  }

  private void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    System.out.println("Pose reset to: " + pose);
  }

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

  private void configureButtonBindings() {
    
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(driveSubsystem.createOrientToFieldCommand(robotState));

    new POVButton(driverController, 0)
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> 0.0));
    new POVButton(driverController, 90)
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> -90.0));
    new POVButton(driverController, 180)
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> 180.0));
    new POVButton(driverController, 270)
        .whileTrue(new SnapToHeadingFixed(driveSubsystem, robotState,
            () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> 90.0));

    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .onTrue(new SetStartingPoseCommand(PID_TUNING_POSITION, "PID Tuning Position", gyro, questNav, driveSubsystem, poseEstimator));

    // Y: Drive to BLUE_REEF_TAG_17 (with precision path)
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(
            SmartDriveToPosition.create(
                BLUE_REEF_TAG_17,              // Staging pose
                "Stage17toPrecise17",           // Precision path file
                poseEstimator,
                tagVisionSubsystem,
                robotState,
                driverController,
                driveSubsystem,
                questNav));

    // B: Drive to BLUE_TAG_16 (with precision path)
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(
            SmartDriveToPosition.create(
                BLUE_TAG_16,                    // Staging pose
                "Stage16toPrecise16",           // Precision path file
                poseEstimator,
                tagVisionSubsystem,
                robotState,
                driverController,
                driveSubsystem,
                questNav));

    // A: Drive to BLUE_TAG_12 (with precision path)
    new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(
            SmartDriveToPosition.create(
                BLUE_TAG_12,                    // Staging pose
                "Stage12toPrecise12",           // Precision path file
                poseEstimator,
                tagVisionSubsystem,
                robotState,
                driverController,
                driveSubsystem,
                questNav));

    // X: Drive to BLUE_AUTO_START_POS_FAR_RIGHT (with precision path)
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileTrue(
            SmartDriveToPosition.create(
                BLUE_AUTO_START_POS_FAR_RIGHT,         // Staging pose
                "StageAutoRightToPreciseAutoRight",    // Precision path file
                poseEstimator,
                tagVisionSubsystem,
                robotState,
                driverController,
                driveSubsystem,
                questNav));

    System.out.println("Button bindings configured");
    System.out.println("Y: BLUE_REEF_TAG_17, B: BLUE_TAG_16, A: BLUE_TAG_12, X: BLUE_AUTO_START_POS_FAR_RIGHT (all 2-phase precision)");
  }

  private void configureTouchscreenInterface() {
    NetworkTable opTable = NetworkTableInstance.getDefault().getTable("OperatorInterface");
    
    // Map of position names to their precision path configs (stagingPose, pathFile)
    Map<String, String> precisionPaths = Map.of(
        "BLUE_REEF_TAG_17", "Stage17toPrecise17",
        "BLUE_REEF_TAG_18", "Stage18toPrecise18",
        "BLUE_REEF_TAG_21", "Stage21toPrecise21",
        "BLUE_REEF_TAG_22", "Stage22toPrecise22",
        "BLUE_TAG_16", "Stage16toPrecise16",
        "BLUE_TAG_12", "Stage12toPrecise12"
    );
    
    Map<String, Pose2d> positions = Map.of(
        "BLUE_REEF_TAG_17", BLUE_REEF_TAG_17,
        "BLUE_REEF_TAG_18", BLUE_REEF_TAG_18,
        "BLUE_REEF_TAG_21", BLUE_REEF_TAG_21,
        "BLUE_REEF_TAG_22", BLUE_REEF_TAG_22,
        "BLUE_TAG_16", BLUE_TAG_16,
        "BLUE_TAG_12", BLUE_TAG_12
    );
    
    // Subscribe to drive-to-position commands - now using SmartDrive
    positions.forEach((positionName, pose) -> {
      BooleanSubscriber sub = opTable
          .getSubTable("DriveToPosition")
          .getBooleanTopic(positionName)
          .subscribe(false);
      
      new Thread(() -> {
        while (true) {
          if (sub.get()) {
            String displayName = positionName.replace("_", " ").toLowerCase();
            String pathFile = precisionPaths.get(positionName);
            
            SmartDriveToPosition.create(
                pose,
                pathFile,
                poseEstimator,
                tagVisionSubsystem,
                robotState,
                driverController,
                driveSubsystem,
                questNav
            ).schedule();
            
            System.out.println("[Touchscreen] SmartDrive to: " + displayName + 
                             (pathFile != null ? " (2-phase precision)" : " (direct)"));
          }
          try {
            Thread.sleep(50);
          } catch (InterruptedException e) {
            break;
          }
        }
      }).start();
    });
    
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
          new SetStartingPoseCommand(BLUE_REEF_TAG_17, "Blue Reef Tag 17", gyro, questNav, driveSubsystem, poseEstimator).schedule();
          System.out.println("[Touchscreen] Set Reef 17 Start Position");
        }
        try {
          Thread.sleep(50);
        } catch (InterruptedException e) {
          break;
        }
      }
    }).start();

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

  public Command getAutonomousCommand() { 
    Command selectedAuto = autoChooser.getSelected();
    if (selectedAuto != null) {
      return wrapPathWithLogging(selectedAuto);
    }
    return selectedAuto;
  }

  private void startAutoPreviewMonitor() {
    new Thread(() -> {
      while (true) {
        try {
          if (DriverStation.isDisabled()) {
            Command selectedAuto = autoChooser.getSelected();
            
            if (selectedAuto != null && selectedAuto != lastSelectedAuto) {
              lastSelectedAuto = selectedAuto;
              
              String autoName = selectedAuto.getName();
              System.out.println("[Auto Preview] Selected: " + autoName);
              
              Pose2d startingPose = getStartingPoseForAuto(autoName);
              
              if (startingPose != null) {
                poseEstimator.resetPose(
                    startingPose,
                    driveSubsystem.getGyroRotation(),
                    driveSubsystem.getModulePositions());
                
                System.out.println("[Auto Preview] Pose set to: " + startingPose);
                Logger.recordOutput("Auto/PreviewPose", startingPose);
              }
            }
          }
          
          Thread.sleep(500);
        } catch (Exception e) {
          System.err.println("[Auto Preview] Error: " + e.getMessage());
        }
      }
    }).start();
  }

  private String formatPose(Pose2d pose) {
    return String.format("(%.2f, %.2f, %.1f°)",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  private Pose2d getStartingPoseForAuto(String autoName) {
    switch (autoName.toLowerCase()) {
      case "leftside1piece":
      case "leftside3piece":
        return new Pose2d(7.20, 0.45, Rotation2d.fromDegrees(180.0));
      case "rightside1piece":
        return new Pose2d(7.20, 5.50, Rotation2d.fromDegrees(180.0));
      default:
        System.err.println("[Auto Preview] Unknown auto: " + autoName);
        return null;
    }
  }
}
