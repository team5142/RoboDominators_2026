package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
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
  }
  
  // Update dashboard - call from Robot.robotPeriodic()
  public void update() {
    // Robot status
    NetworkTable statusTable = elasticTable.getSubTable("Status");
    statusTable.getEntry("Mode").setString(robotState.getMode().toString());
    statusTable.getEntry("Enabled").setBoolean(robotState.isEnabled());
    statusTable.getEntry("MatchTime").setDouble(round(DriverStation.getMatchTime(), 1));
    statusTable.getEntry("BatteryVoltage").setDouble(
        round(edu.wpi.first.wpilibj.RobotController.getBatteryVoltage(), 2));
    
    // Pose estimation
    NetworkTable poseTable = elasticTable.getSubTable("Pose");
    var pose = poseEstimator.getEstimatedPose();
    poseTable.getEntry("X").setDouble(round(pose.getX(), 2));
    poseTable.getEntry("Y").setDouble(round(pose.getY(), 2));
    poseTable.getEntry("Rotation").setDouble(round(pose.getRotation().getDegrees(), 1));
    poseTable.getEntry("Initialized").setBoolean(poseEstimator.isInitialized());
    
    // QuestNav
    NetworkTable questTable = elasticTable.getSubTable("QuestNav");
    questTable.getEntry("Connected").setBoolean(questNav.isConnected());
    questTable.getEntry("Tracking").setBoolean(questNav.isTracking());
    questTable.getEntry("Battery").setInteger(questNav.getBatteryPercent());
    
    questNav.getRobotPose().ifPresent(qPose -> {
      questTable.getEntry("X").setDouble(round(qPose.getX(), 2));
      questTable.getEntry("Y").setDouble(round(qPose.getY(), 2));
      questTable.getEntry("Rotation").setDouble(round(qPose.getRotation().getDegrees(), 1));
    });
    
    // Vision
    NetworkTable visionTable = elasticTable.getSubTable("Vision");
    visionTable.getEntry("ActiveCameras").setInteger(tagVision.getActiveCameraCount());
    visionTable.getEntry("TotalCameras").setInteger(tagVision.getCameraCount());
    visionTable.getEntry("HasPose").setBoolean(tagVision.hasRecentTagPose());
    
    // Drive
    NetworkTable driveTable = elasticTable.getSubTable("Drive");
    var speeds = drive.getRobotRelativeSpeeds();
    driveTable.getEntry("VelocityX").setDouble(round(speeds.vxMetersPerSecond, 2));
    driveTable.getEntry("VelocityY").setDouble(round(speeds.vyMetersPerSecond, 2));
    driveTable.getEntry("Omega").setDouble(round(speeds.omegaRadiansPerSecond, 2));
  }
  
  // Round value to specified decimal places (reduces NetworkTable spam)
  private double round(double value, int decimals) {
    double multiplier = Math.pow(10, decimals);
    return Math.round(value * multiplier) / multiplier;
  }
}