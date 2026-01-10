package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.SmartLogger;

// Publishes robot data to Elastic Dashboard via NetworkTables
// Elastic auto-creates widgets from published data
public class ElasticDashboard {
  
  private final NetworkTable elasticTable;
  private final RobotState robotState;
  private final PoseEstimatorSubsystem poseEstimator;
  private final QuestNavSubsystem questNav;
  private final TagVisionSubsystem tagVision;
  private final DriveSubsystem drive;
  
  private final NetworkTable statusTable;
  private final NetworkTable poseTable;
  private final NetworkTable questTable;
  private final NetworkTable visionTable;
  private final NetworkTable driveTable;

  private final NetworkTableEntry statusMode;
  private final NetworkTableEntry statusEnabled;
  private final NetworkTableEntry statusMatchTime;
  private final NetworkTableEntry statusBatteryVoltage;

  private final NetworkTableEntry poseX;
  private final NetworkTableEntry poseY;
  private final NetworkTableEntry poseRotation;
  private final NetworkTableEntry poseInitialized;

  private final NetworkTableEntry questConnected;
  private final NetworkTableEntry questTracking;
  private final NetworkTableEntry questBattery;
  private final NetworkTableEntry questX;
  private final NetworkTableEntry questY;
  private final NetworkTableEntry questRotation;

  private final NetworkTableEntry visionActiveCameras;
  private final NetworkTableEntry visionTotalCameras;
  private final NetworkTableEntry visionHasPose;

  private final NetworkTableEntry driveVx;
  private final NetworkTableEntry driveVy;
  private final NetworkTableEntry driveOmega;
  
  public ElasticDashboard(
      RobotState robotState,
      PoseEstimatorSubsystem poseEstimator,
      QuestNavSubsystem questNav,
      TagVisionSubsystem tagVision,
      DriveSubsystem drive) {
    
    this.robotState = robotState;
    this.poseEstimator = poseEstimator;
    this.questNav = questNav;
    this.tagVision = tagVision;
    this.drive = drive;
    
    // Create Elastic-specific table
    this.elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
    
    // Publish camera stream URLs (use mDNS hostname)
    NetworkTable cameraTable = elasticTable.getSubTable("Cameras");
    cameraTable.getEntry("Limelight/URL").setString("http://limelight-front.local:5800");
    cameraTable.getEntry("Limelight/Name").setString("Limelight Front");
    cameraTable.getEntry("Limelight/FPS").setInteger(30);
    
    SmartLogger.logConsole("Elastic Dashboard initialized - stream: http://limelight-front.local:5800");

    statusTable = elasticTable.getSubTable("Status");
    poseTable = elasticTable.getSubTable("Pose");
    questTable = elasticTable.getSubTable("QuestNav");
    visionTable = elasticTable.getSubTable("Vision");
    driveTable = elasticTable.getSubTable("Drive");

    statusMode = statusTable.getEntry("Mode");
    statusEnabled = statusTable.getEntry("Enabled");
    statusMatchTime = statusTable.getEntry("MatchTime");
    statusBatteryVoltage = statusTable.getEntry("BatteryVoltage");

    poseX = poseTable.getEntry("X");
    poseY = poseTable.getEntry("Y");
    poseRotation = poseTable.getEntry("Rotation");
    poseInitialized = poseTable.getEntry("Initialized");

    questConnected = questTable.getEntry("Connected");
    questTracking = questTable.getEntry("Tracking");
    questBattery = questTable.getEntry("Battery");
    questX = questTable.getEntry("X");
    questY = questTable.getEntry("Y");
    questRotation = questTable.getEntry("Rotation");

    visionActiveCameras = visionTable.getEntry("ActiveCameras");
    visionTotalCameras = visionTable.getEntry("TotalCameras");
    visionHasPose = visionTable.getEntry("HasPose");

    driveVx = driveTable.getEntry("VelocityX");
    driveVy = driveTable.getEntry("VelocityY");
    driveOmega = driveTable.getEntry("Omega");
  }
  
  // Update dashboard - call from Robot.robotPeriodic()
  public void update(double batteryVoltage) {
    // Robot status
    statusMode.setString(robotState.getMode().toString());
    statusEnabled.setBoolean(robotState.isEnabled());
    statusMatchTime.setDouble(round(DriverStation.getMatchTime(), 1));
    statusBatteryVoltage.setDouble(round(batteryVoltage, 2));
    
    // Pose estimation
    var pose = poseEstimator.getEstimatedPose();
    poseX.setDouble(round(pose.getX(), 2));
    poseY.setDouble(round(pose.getY(), 2));
    poseRotation.setDouble(round(pose.getRotation().getDegrees(), 1));
    poseInitialized.setBoolean(poseEstimator.isInitialized());
    
    // QuestNav
    questConnected.setBoolean(questNav.isConnected());
    questTracking.setBoolean(questNav.isTracking());
    questBattery.setInteger(questNav.getBatteryPercent());
    
    questNav.getRobotPose().ifPresent(qPose -> {
      questX.setDouble(round(qPose.getX(), 2));
      questY.setDouble(round(qPose.getY(), 2));
      questRotation.setDouble(round(qPose.getRotation().getDegrees(), 1));
    });
    
    // Vision
    visionActiveCameras.setInteger(tagVision.getActiveCameraCount());
    visionTotalCameras.setInteger(tagVision.getCameraCount());
    visionHasPose.setBoolean(tagVision.hasRecentTagPose());
    
    // Drive
    var speeds = drive.getRobotRelativeSpeeds();
    driveVx.setDouble(round(speeds.vxMetersPerSecond, 2));
    driveVy.setDouble(round(speeds.vyMetersPerSecond, 2));
    driveOmega.setDouble(round(speeds.omegaRadiansPerSecond, 2));
  }
  
  // Round value to specified decimal places (reduces NetworkTable spam)
  private double round(double value, int decimals) {
    double multiplier = Math.pow(10, decimals);
    return Math.round(value * multiplier) / multiplier;
  }
}