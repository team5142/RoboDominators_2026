package frc.robot.util;

import static frc.robot.Constants.StartingPositions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.commands.drive.SmartDriveToPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

// Touchscreen operator interface - subscribes to NetworkTables commands from HTML dashboard
public class TouchscreenInterface {

  private final RobotState robotState;
  private final DriveSubsystem driveSubsystem;
  private final PoseEstimatorSubsystem poseEstimator;
  private final QuestNavSubsystem questNav;

  private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();

  private Command activeOperatorDrive = null;

  private static class SmartDriveTarget {
    public final Pose2d staging;
    public final Pose2d precise;

    public SmartDriveTarget(Pose2d staging, Pose2d precise) {
      this.staging = staging;
      this.precise = precise;
    }
  }

  private final Map<String, SmartDriveTarget> targets = new HashMap<>();
  private final Map<String, Boolean> lastValueByKey = new HashMap<>();

  public TouchscreenInterface(
      RobotState robotState,
      DriveSubsystem driveSubsystem,
      PoseEstimatorSubsystem poseEstimator,
      GyroSubsystem gyro,
      QuestNavSubsystem questNav) {

    this.robotState = robotState;
    this.driveSubsystem = driveSubsystem;
    this.poseEstimator = poseEstimator;
    this.questNav = questNav;
  }

  public void configure() {
    targets.put("BLUE_REEF_TAG_17", new SmartDriveTarget(BLUE_REEF_TAG_17, PRECISE_17_POSE));
    targets.put("BLUE_REEF_TAG_18", new SmartDriveTarget(BLUE_REEF_TAG_18, PRECISE_18_POSE));
    targets.put("BLUE_REEF_TAG_21", new SmartDriveTarget(BLUE_REEF_TAG_21, PRECISE_21_POSE));
    targets.put("BLUE_REEF_TAG_22", new SmartDriveTarget(BLUE_REEF_TAG_22, PRECISE_22_POSE));
    targets.put("BLUE_TAG_16", new SmartDriveTarget(BLUE_TAG_16, PRECISE_16_POSE));
    targets.put("BLUE_TAG_12", new SmartDriveTarget(BLUE_TAG_12, PRECISE_12_POSE));

    NetworkTable opTable = ntInst.getTable("OperatorInterface");
    NetworkTable driveTable = opTable.getSubTable("DriveToPosition");

    targets.forEach((key, target) -> {
      BooleanTopic topic = driveTable.getBooleanTopic(key);
      lastValueByKey.put(key, false);

      ntInst.addListener(
          topic,
          EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          event -> onDriveTopicEvent(key, event));
    });

    SmartLogger.logConsole("Touchscreen operator interface configured", "Touchscreen Ready", 5);
  }

  private void onDriveTopicEvent(String key, NetworkTableEvent event) {
    if (event.valueData == null) {
      return;
    }

    boolean value;
    try {
      value = event.valueData.value.getBoolean();
    } catch (Exception e) {
      return;
    }

    boolean last = lastValueByKey.getOrDefault(key, false);
    lastValueByKey.put(key, value);

    if (!value || last) {
      return;
    }

    SmartDriveTarget target = targets.get(key);
    if (target == null) {
      return;
    }

    scheduleOperatorSmartDrive(key, target);
  }

  private void scheduleOperatorSmartDrive(String key, SmartDriveTarget target) {
    cancelActiveOperatorDrive();

    robotState.setOperatorDriveLockout(true);

    Command cmd = SmartDriveToPosition.create(target.staging, target.precise)
        .finallyDo(interrupted -> robotState.setOperatorDriveLockout(false));

    activeOperatorDrive = cmd;
    cmd.schedule();

    SmartLogger.logConsole("[Touchscreen] SmartDrive: " + key);
  }

  public void cancelActiveOperatorDrive() {
    if (activeOperatorDrive != null) {
      activeOperatorDrive.cancel();
      activeOperatorDrive = null;
    }
    robotState.setOperatorDriveLockout(false);
    SmartLogger.logConsole("[Touchscreen] Driver override - canceled operator SmartDrive");
  }
}