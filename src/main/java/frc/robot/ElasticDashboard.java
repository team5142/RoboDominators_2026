package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.MatchPhaseTracker;
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

  // Phase detail topics for teleop shift display
  private final NetworkTable phaseTable;
  private final NetworkTableEntry phaseName;
  private final NetworkTableEntry phaseHubStatus;
  private final NetworkTableEntry phaseSecondsUntilNext;
  private final NetworkTableEntry phaseCountdownText;
  private final NetworkTableEntry phaseShiftNumber;

  private final NetworkTableEntry statusMode;
  private final NetworkTableEntry statusEnabled;
  private final NetworkTableEntry statusMatchTime;
  private final NetworkTableEntry statusBatteryVoltage;
  private final NetworkTableEntry statusHubActive;

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

  // Tab switching - written to /Elastic/SelectedTab every loop
  private final NetworkTableEntry selectedTab;
  private String lastSelectedTab = "";

  // Ball counters
  private final NetworkTable scoringTable;
  private final NetworkTableEntry scoringAuto;
  private final NetworkTableEntry scoringTeleop;
  private final NetworkTableEntry scoringEndgame;
  private final NetworkTableEntry scoringPassed;
  private final NetworkTableEntry scoringTotal;
  private final NetworkTableEntry scoringResetButton; // Elastic boolean toggle — reset when true

  // Tracks how long the robot has been enabled (used when no FMS match time)
  private final Timer enabledTimer = new Timer();
  private boolean wasEnabled = false;
  private int updateCounter = 0; // Used to throttle slow-changing fields to 10Hz
  
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

    statusTable = elasticTable.getSubTable("Status");
    poseTable = elasticTable.getSubTable("Pose");
    questTable = elasticTable.getSubTable("QuestNav");
    visionTable = elasticTable.getSubTable("Vision");
    driveTable = elasticTable.getSubTable("Drive");
    phaseTable = elasticTable.getSubTable("Phase");

    // Publish camera stream URLs once at startup (used by Elastic Camera Stream widgets)
    NetworkTable cameraTable = elasticTable.getSubTable("Cameras");
    cameraTable.getEntry("Limelight/URL").setString("http://limelight-front.local:5800");
    cameraTable.getEntry("Limelight/Name").setString("Limelight Front");
    cameraTable.getEntry("Limelight/FPS").setInteger(30);
    cameraTable.getEntry("QuestNav/URL").setString("http://10.51.42.200:5801/video");
    cameraTable.getEntry("QuestNav/Name").setString("QuestNav Camera");

    SmartLogger.logConsole("Elastic Dashboard initialized", "Elastic");

    statusMode = statusTable.getEntry("Mode");
    statusEnabled = statusTable.getEntry("Enabled");
    statusMatchTime = statusTable.getEntry("MatchTime");
    statusBatteryVoltage = statusTable.getEntry("BatteryVoltage");
    statusHubActive = statusTable.getEntry("HubActive");

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

    phaseName = phaseTable.getEntry("Name");
    phaseHubStatus = phaseTable.getEntry("HubStatus");
    phaseSecondsUntilNext = phaseTable.getEntry("SecondsUntilNext");
    phaseCountdownText = phaseTable.getEntry("CountdownText");
    phaseShiftNumber = phaseTable.getEntry("ShiftNumber");

    scoringTable = elasticTable.getSubTable("Scoring");
    scoringAuto        = scoringTable.getEntry("Auto");
    scoringTeleop      = scoringTable.getEntry("Teleop");
    scoringEndgame     = scoringTable.getEntry("Endgame");
    scoringPassed      = scoringTable.getEntry("Passed");
    scoringTotal       = scoringTable.getEntry("Total");
    scoringResetButton = scoringTable.getEntry("Reset");
    scoringResetButton.setBoolean(false); // ensure it starts as false

    selectedTab = NetworkTableInstance.getDefault().getTable("Elastic").getEntry("SelectedTab");
  }
  
  // Update dashboard - call from Robot.robotPeriodic()
  public void update(double batteryVoltage) {
    updateCounter++;

    // Switch Elastic tab based on match phase (only write when it changes)
    String tab = computeTabName();
    if (!tab.equals(lastSelectedTab)) {
      selectedTab.setString(tab);
      lastSelectedTab = tab;
    }

    // Track enabled timer - reset on enable, stop on disable
    boolean enabled = robotState.isEnabled();
    if (enabled && !wasEnabled) enabledTimer.restart();
    else if (!enabled && wasEnabled) enabledTimer.stop();
    wasEnabled = enabled;

    // Status fields change slowly - publish at 10Hz (every 5 loops at 50Hz)
    if (updateCounter % 5 == 0) {
      statusMode.setString(robotState.getMode().toString());
      statusEnabled.setBoolean(enabled);
      double dsTime = DriverStation.getMatchTime();
      statusMatchTime.setDouble(round(dsTime >= 0 ? dsTime : enabledTimer.get(), 1));
      statusBatteryVoltage.setDouble(round(batteryVoltage, 2));
      statusHubActive.setBoolean(robotState.isHubActive());
      questConnected.setBoolean(questNav.isConnected());
      questTracking.setBoolean(questNav.isTracking());
      questBattery.setInteger(questNav.getBatteryPercent());
      visionActiveCameras.setInteger(tagVision.getActiveCameraCount());
      visionTotalCameras.setInteger(tagVision.getCameraCount());
      visionHasPose.setBoolean(tagVision.hasRecentTagPose());
      poseInitialized.setBoolean(poseEstimator.isInitialized());

      // Ball counters — check reset button first, then publish current values
      if (scoringResetButton.getBoolean(false)) {
        robotState.resetBallCounters();
        scoringResetButton.setBoolean(false);
      }
      scoringAuto.setInteger(robotState.getBallsShotAuto());
      scoringTeleop.setInteger(robotState.getBallsShotTeleop());
      scoringEndgame.setInteger(robotState.getBallsShotEndgame());
      scoringPassed.setInteger(robotState.getBallsPassed());
      scoringTotal.setInteger(robotState.getBallsShotTotal());
    }

    // Pose, QuestNav position, and drive velocity change every loop - publish at full rate
    var pose = poseEstimator.getEstimatedPose();
    poseX.setDouble(round(pose.getX(), 2));
    poseY.setDouble(round(pose.getY(), 2));
    poseRotation.setDouble(round(pose.getRotation().getDegrees(), 1));

    questNav.getRobotPose().ifPresent(qPose -> {
      questX.setDouble(round(qPose.getX(), 2));
      questY.setDouble(round(qPose.getY(), 2));
      questRotation.setDouble(round(qPose.getRotation().getDegrees(), 1));
    });

    // Drive velocity changes every loop
    var speeds = drive.getRobotRelativeSpeeds();
    driveVx.setDouble(round(speeds.vxMetersPerSecond, 2));
    driveVy.setDouble(round(speeds.vyMetersPerSecond, 2));
    driveOmega.setDouble(round(speeds.omegaRadiansPerSecond, 2));

    // Shift phase detail for Teleop tab (countdown needs per-loop resolution)
    double secsUntilNext = robotState.getSecondsUntilPhaseEnd();
    phaseName.setString(robotState.getPhaseName());
    phaseHubStatus.setString(robotState.isHubActive() ? "ACTIVE" : "INACTIVE");
    phaseSecondsUntilNext.setDouble(round(secsUntilNext, 1));
    phaseShiftNumber.setInteger(robotState.getShiftNumber());
    // Countdown text: "3", "2", "1" in the last 3s of a shift, else empty
    int shiftNum = robotState.getShiftNumber();
    if (shiftNum > 0 && secsUntilNext <= 3.5 && secsUntilNext > 0) {
      phaseCountdownText.setString(String.valueOf((int) Math.ceil(secsUntilNext)));
    } else if (shiftNum == 0 && secsUntilNext <= 3.5 && secsUntilNext > 0) {
      phaseCountdownText.setString("ENDING " + (int) Math.ceil(secsUntilNext));
    } else {
      phaseCountdownText.setString("");
    }
  }
  
  // Map current game phase to the matching Elastic tab name
  private String computeTabName() {
    MatchPhaseTracker.GamePhase phase = robotState.getGamePhase();
    if (phase == MatchPhaseTracker.GamePhase.DISABLED) {
      return "Disabled";
    } else if (phase == MatchPhaseTracker.GamePhase.AUTO) {
      return "Auto";
    } else if (phase == MatchPhaseTracker.GamePhase.END_GAME) {
      return "Endgame";
    } else {
      return "Teleop";
    }
  }

  // Round value to specified decimal places (reduces NetworkTable spam)
  private double round(double value, int decimals) {
    double multiplier = Math.pow(10, decimals);
    return Math.round(value * multiplier) / multiplier;
  }
}