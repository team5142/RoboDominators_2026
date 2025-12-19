package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Publishes robot data to Elastic Dashboard
 * 
 * Elastic auto-discovers NetworkTables data and creates widgets automatically.
 * This class organizes data into logical groups for better dashboard layout.
 */
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
    
    // NEW: Publish camera stream URLs (CHANGED: Use mDNS hostname)
    NetworkTable cameraTable = elasticTable.getSubTable("Cameras");
    cameraTable.getEntry("Limelight/URL").setString("http://limelight-front.local:5800");
    cameraTable.getEntry("Limelight/Name").setString("Limelight Front");
    cameraTable.getEntry("Limelight/FPS").setInteger(30);
    
    System.out.println("Elastic Dashboard integration initialized");
    System.out.println("  - Limelight stream: http://limelight-front.local:5800");
  }
  
  /**
   * Update dashboard - call from Robot.robotPeriodic()
   */
  public void update() {
    // ===== ROBOT STATUS =====
    NetworkTable statusTable = elasticTable.getSubTable("Status");
    statusTable.getEntry("Mode").setString(robotState.getMode().toString());
    statusTable.getEntry("Enabled").setBoolean(robotState.isEnabled());
    statusTable.getEntry("MatchTime").setDouble(round(DriverStation.getMatchTime(), 1)); // 1 decimal
    statusTable.getEntry("BatteryVoltage").setDouble(
        round(edu.wpi.first.wpilibj.RobotController.getBatteryVoltage(), 2)); // 2 decimals
    
    // ===== POSE ESTIMATION =====
    NetworkTable poseTable = elasticTable.getSubTable("Pose");
    var pose = poseEstimator.getEstimatedPose();
    poseTable.getEntry("X").setDouble(round(pose.getX(), 2)); // 2 decimals (0.01m = 1cm)
    poseTable.getEntry("Y").setDouble(round(pose.getY(), 2)); // 2 decimals
    poseTable.getEntry("Rotation").setDouble(round(pose.getRotation().getDegrees(), 1)); // 1 decimal (0.1°)
    poseTable.getEntry("Initialized").setBoolean(poseEstimator.isInitialized());
    
    // ===== QUESTNAV =====
    NetworkTable questTable = elasticTable.getSubTable("QuestNav");
    questTable.getEntry("Connected").setBoolean(questNav.isConnected());
    questTable.getEntry("Tracking").setBoolean(questNav.isTracking());
    questTable.getEntry("Battery").setInteger(questNav.getBatteryPercent()); // Already whole number
    
    questNav.getRobotPose().ifPresent(qPose -> {
      questTable.getEntry("X").setDouble(round(qPose.getX(), 2));
      questTable.getEntry("Y").setDouble(round(qPose.getY(), 2));
      questTable.getEntry("Rotation").setDouble(round(qPose.getRotation().getDegrees(), 1));
    });
    
    // ===== VISION =====
    NetworkTable visionTable = elasticTable.getSubTable("Vision");
    visionTable.getEntry("ActiveCameras").setInteger(tagVision.getActiveCameraCount());
    visionTable.getEntry("TotalCameras").setInteger(tagVision.getCameraCount());
    visionTable.getEntry("HasPose").setBoolean(tagVision.hasRecentTagPose());
    
    // ===== DRIVE =====
    NetworkTable driveTable = elasticTable.getSubTable("Drive");
    var speeds = drive.getRobotRelativeSpeeds();
    driveTable.getEntry("VelocityX").setDouble(round(speeds.vxMetersPerSecond, 2)); // 2 decimals
    driveTable.getEntry("VelocityY").setDouble(round(speeds.vyMetersPerSecond, 2));
    driveTable.getEntry("Omega").setDouble(round(speeds.omegaRadiansPerSecond, 2));
  }
  
  /**
   * Round a double to specified decimal places
   * @param value Value to round
   * @param decimals Number of decimal places (0 = whole number)
   * @return Rounded value
   */
  private double round(double value, int decimals) {
    double multiplier = Math.pow(10, decimals);
    return Math.round(value * multiplier) / multiplier;
  }
}
