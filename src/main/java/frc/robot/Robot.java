package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.util.SmartLogger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import frc.robot.util.LogSpaceMonitor;

// Main robot class - runs on boot, manages all modes (auto/teleop/test), 20ms loop
public class Robot extends LoggedRobot {
  
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private RobotState robotState;
  
  private boolean lowBatteryWarningShown = false;
  private boolean criticalBatteryWarningShown = false;
  private boolean matchActive = false;
  private ElasticDashboard elasticDashboard;
  
  private double cachedBatteryVoltage = 0.0; // Cache to avoid multiple reads per loop

  // Runs ONCE at robot boot - setup logging and create subsystems
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "RoboDominators_2026");
    Logger.recordMetadata("TeamNumber", "5142");
    Logger.recordMetadata("RobotName", "Lebron2");
    
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
    Logger.start();
    
    String initMsg = "RoboDominators 5142 - Lebron2\n" +
                    "AdvantageKit: ACTIVE\n" +
                    "Battery: " + RobotController.getBatteryVoltage() + "V";
    SmartLogger.logConsole(initMsg, "Robot Init", 15);
    
    try {
      robotState = new RobotState();
      robotContainer = new RobotContainer();
      SmartLogger.logConsole("Robot container initialized successfully");
      
      // Only setup Limelight stream in practice mode
      if (!RobotContainer.COMPETITION_MODE) {
        setupLimelightStream();
      } else {
        SmartLogger.logConsole("Limelight stream DISABLED (competition mode)", "Bandwidth Saving");
      }
      
      elasticDashboard = new ElasticDashboard(
          robotContainer.robotState,
          robotContainer.poseEstimator,
          robotContainer.questNav,
          robotContainer.tagVisionSubsystem,
          robotContainer.driveSubsystem);
    } catch (Exception e) {
      SmartLogger.logConsoleError("FATAL ERROR during robot initialization: " + e.getMessage());
      DriverStation.reportError("Robot failed to initialize: " + e.getMessage(), true);
    }
  }

  // Setup Limelight camera stream for dashboards
  private void setupLimelightStream() {
    try {
      HttpCamera limelightCamera = new HttpCamera(
          "Limelight", 
          "http://limelight-front.local:5800/stream.mjpg");
      
      MjpegServer server = CameraServer.addServer("Limelight Stream", 1181);
      server.setSource(limelightCamera);
      
      SmartLogger.logConsole("Limelight stream: http://limelight-front.local:5800 (port 1181)");
    } catch (Exception e) {
      SmartLogger.logConsoleError("Failed to setup Limelight stream: " + e.getMessage());
    }
  }

  // Runs EVERY 20ms in ALL modes - robot heartbeat
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    // Read battery once per loop
    cachedBatteryVoltage = RobotController.getBatteryVoltage();
    
    // Check battery only when disabled (before match)
    if (!matchActive) {
      checkBatteryVoltage();
    }
    
    // Log battery and current
    Logger.recordOutput("Battery/Voltage", cachedBatteryVoltage);
    Logger.recordOutput("Battery/Current", RobotController.getInputCurrent());
    
    // Check brownout
    if (RobotController.isBrownedOut()) {
      Logger.recordOutput("Robot/BrownedOut", true);
      SmartLogger.logConsoleError("!!! BROWNOUT DETECTED !!!");
    } else {
      Logger.recordOutput("Robot/BrownedOut", false);
    }

    if (robotContainer != null) {
      robotContainer.periodic();
    }
    
    // Null check for elasticDashboard
    if (elasticDashboard != null) {
      elasticDashboard.update();
    }
    
    LogSpaceMonitor.periodic();
  }

  // Check battery with proper hysteresis (disabled mode only)
  private void checkBatteryVoltage() {
    // Critical: < 10.0V
    if (cachedBatteryVoltage < 10.0 && !criticalBatteryWarningShown) {
      SmartLogger.logConsoleError("*** CRITICAL BATTERY: " + cachedBatteryVoltage + "V - CHARGE IMMEDIATELY!");
      DriverStation.reportError("CRITICAL BATTERY: " + cachedBatteryVoltage + "V", false);
      criticalBatteryWarningShown = true;
    }
    // Reset critical when above 10.2V (0.2V hysteresis)
    if (cachedBatteryVoltage > 10.2) {
      criticalBatteryWarningShown = false;
    }
    
    // Low: < 11.4V
    if (cachedBatteryVoltage < 11.4 && !lowBatteryWarningShown) {
      SmartLogger.logConsoleError("!!! LOW BATTERY: " + cachedBatteryVoltage + "V - charge before next match");
      DriverStation.reportWarning("Low battery: " + cachedBatteryVoltage + "V", false);
      lowBatteryWarningShown = true;
    }
    // Reset low when above 11.6V (0.2V hysteresis)
    if (cachedBatteryVoltage > 11.6) {
      lowBatteryWarningShown = false;
    }
  }

  @Override
  public void disabledInit() {
    matchActive = false;
    robotState.setEnabled(false);
    robotState.setMode(RobotState.Mode.DISABLED);
    
    Logger.recordOutput("Robot/Mode", "DISABLED");
    SmartLogger.logConsole("Robot DISABLED");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    matchActive = true;
    robotState.setEnabled(true);
    robotState.setMode(RobotState.Mode.ENABLED_AUTO);
    
    Logger.recordOutput("Robot/Mode", "AUTO");
    SmartLogger.logConsole(">>> AUTONOMOUS MODE STARTED <<<", "Auto Start", 15);
    
    autonomousCommand = robotContainer.getAutonomousCommand();
    
    if (autonomousCommand != null) {
      String autoName = autonomousCommand.getName();
      SmartLogger.logConsole("Running auto: " + autoName);
      Logger.recordOutput("Auto/SelectedName", autoName);
      
      if (cachedBatteryVoltage < 10) {
        SmartLogger.logConsoleError("WARNING: Low battery for auto (" + cachedBatteryVoltage + "V)");
        DriverStation.reportWarning("Low battery for auto!", false);
      }
      
      autonomousCommand.schedule();
      SmartLogger.logConsole("Auto command scheduled");
    } else {
      SmartLogger.logConsoleError("NO AUTO SELECTED!");
      DriverStation.reportError("No auto selected!", false);
      Logger.recordOutput("Auto/SelectedName", "NONE");
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    matchActive = true;
    
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      SmartLogger.logConsole("Auto canceled - driver has control");
    }
    
    robotState.setEnabled(true);
    robotState.setMode(RobotState.Mode.ENABLED_TELEOP);
    
    Logger.recordOutput("Robot/Mode", "TELEOP");
    SmartLogger.logConsole(">>> TELEOP started - driver has control");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    robotState.setMode(RobotState.Mode.TEST);
    
    Logger.recordOutput("Robot/Mode", "TEST");
    SmartLogger.logConsole("TEST mode started");
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    Logger.recordOutput("Robot/Mode", "SIMULATION");
    SmartLogger.logConsole("Simulation mode started");
  }

  @Override
  public void simulationPeriodic() {}
}