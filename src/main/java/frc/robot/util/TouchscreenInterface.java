package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.robot.commands.auto.DriveToSavedPosition;
import frc.robot.commands.util.SetStartingPoseCommand;
import frc.robot.subsystems.*;
import static frc.robot.Constants.StartingPositions.*;

import java.util.Map;

// Touchscreen operator interface - subscribes to NetworkTables commands from HTML dashboard
public class TouchscreenInterface {
  
  private final RobotState robotState;
  private final DriveSubsystem driveSubsystem;
  private final PoseEstimatorSubsystem poseEstimator;
  private final GyroSubsystem gyro;
  private final QuestNavSubsystem questNav;
  
  public TouchscreenInterface(
      RobotState robotState,
      DriveSubsystem driveSubsystem,
      PoseEstimatorSubsystem poseEstimator,
      GyroSubsystem gyro,
      QuestNavSubsystem questNav) {
    
    this.robotState = robotState;
    this.driveSubsystem = driveSubsystem;
    this.poseEstimator = poseEstimator;
    this.gyro = gyro;
    this.questNav = questNav;
  }
  
  // Configure all touchscreen subscriptions and publishers
  public void configure() {
    // Position map - centralized in TouchscreenInterface
    Map<String, Pose2d> positions = Map.of(
        "BLUE_REEF_TAG_17", BLUE_REEF_TAG_17,
        "BLUE_REEF_TAG_18", BLUE_REEF_TAG_18,
        "BLUE_REEF_TAG_21", BLUE_REEF_TAG_21,
        "BLUE_REEF_TAG_22", BLUE_REEF_TAG_22,
        "BLUE_TAG_16", BLUE_TAG_16,
        "BLUE_TAG_12", BLUE_TAG_12
    );
    
    NetworkTable opTable = NetworkTableInstance.getDefault().getTable("OperatorInterface");
    
    // Drive-to-position commands
    positions.forEach((positionName, pose) -> {
      BooleanSubscriber sub = opTable
          .getSubTable("DriveToPosition")
          .getBooleanTopic(positionName)
          .subscribe(false);
      
      new Thread(() -> {
        while (true) {
          if (sub.get()) {
            String displayName = positionName.replace("_", " ").toLowerCase();
            new DriveToSavedPosition(pose, displayName, poseEstimator).schedule();
            SmartLogger.logConsole("[Touchscreen] Drive to: " + displayName);
          }
          try {
            Thread.sleep(50);
          } catch (InterruptedException e) {
            break;
          }
        }
      }).start();
    });
    
    // Orient to field command
    BooleanSubscriber orientFieldSub = opTable
        .getSubTable("Action")
        .getBooleanTopic("OrientToField")
        .subscribe(false);
    
    new Thread(() -> {
      while (true) {
        if (orientFieldSub.get()) {
          driveSubsystem.createOrientToFieldCommand(robotState).schedule();
          SmartLogger.logConsole("[Touchscreen] Orient to Field");
        }
        try {
          Thread.sleep(50);
        } catch (InterruptedException e) {
          break;
        }
      }
    }).start();
    
    // Set Reef 17 position command
    BooleanSubscriber setReef17Sub = opTable
        .getSubTable("Action")
        .getBooleanTopic("SetReef17")
        .subscribe(false);
    
    new Thread(() -> {
      while (true) {
        if (setReef17Sub.get()) {
          new SetStartingPoseCommand(BLUE_REEF_TAG_17, "Blue Reef Tag 17", gyro, questNav, driveSubsystem, poseEstimator).schedule();
          SmartLogger.logConsole("[Touchscreen] Set Reef 17 Start Position");
        }
        try {
          Thread.sleep(50);
        } catch (InterruptedException e) {
          break;
        }
      }
    }).start();
    
    // Publish robot state to touchscreen
    BooleanPublisher connectedPub = opTable.getBooleanTopic("RobotConnected").publish();
    BooleanPublisher enabledPub = opTable.getBooleanTopic("RobotEnabled").publish();
    
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
    
    SmartLogger.logConsole("Touchscreen operator interface configured", "Touchscreen Ready", 5);
  }
}