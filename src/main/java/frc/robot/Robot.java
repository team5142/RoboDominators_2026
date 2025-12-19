package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.util.ConsoleLogger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;

// Main robot class - runs on boot, manages all modes (auto/teleop/test), 20ms loop
public class Robot extends LoggedRobot {
  
  private Command autonomousCommand; // Selected auto routine
  private RobotContainer robotContainer; // Holds all subsystems and commands
  private RobotState robotState; // Robot state tracker
  
  private boolean lowBatteryWarningShown = false; // Prevent spam
  private boolean criticalBatteryWarningShown = false;
  private boolean matchActive = false; // Track if we're in a match (auto/teleop/test)
  private ElasticDashboard elasticDashboard; // NEW

  // Runs ONCE at robot boot - setup logging and create subsystems
  @Override
  public void robotInit() {
    // AdvantageKit logging setup - records everything for replay
    Logger.recordMetadata("ProjectName", "RoboDominators_2026");
    Logger.recordMetadata("TeamNumber", "5142");
    Logger.recordMetadata("RobotName", "Lebron2");
    
    Logger.addDataReceiver(new NT4Publisher()); // Live dashboard data
    Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs")); // Persistent logs
    Logger.start();
    
    ConsoleLogger.log("========================================");
    ConsoleLogger.log("RoboDominators 5142 - Lebron2");
    ConsoleLogger.log("AdvantageKit logging: ACTIVE");
    ConsoleLogger.log("Battery: " + getBatteryVoltage() + "V");
    ConsoleLogger.log("========================================");
    try {
      robotState = new RobotState();
      robotContainer = new RobotContainer();
      ConsoleLogger.log("Robot container initialized successfully");
      ConsoleLogger.log("Robot container initialized successfully");
      
      // NEW: Setup Limelight video stream
      setupLimelightStream();
      
      // NEW: Initialize Elastic Dashboard after RobotContainer
      elasticDashboard = new ElasticDashboard(
          robotContainer.robotState,          // You'll need to expose these
          robotContainer.poseEstimator,       // as public getters in RobotContainer
          robotContainer.questNav,
          robotContainer.tagVisionSubsystem,
          robotContainer.driveSubsystem);
    } catch (Exception e) {
      ConsoleLogger.logError("FATAL ERROR during robot initialization: " + e.getMessage());
      //e.printStackTrace();
      //DriverStation.reportError("Robot failed to initialize: " + e.getMessage(), true);
    }
  }

  /**
   * Setup Limelight camera stream for dashboards
   */
  private void setupLimelightStream() {
    try {
      // Create HTTP camera source from Limelight (using mDNS hostname)
      HttpCamera limelightCamera = new HttpCamera(
          "Limelight", 
          "http://limelight-front.local:5800/stream.mjpg"); // CHANGED: Use mDNS hostname
      
      // Publish to CameraServer (makes it available to all dashboards)
      MjpegServer server = CameraServer.addServer("Limelight Stream", 1181);
      server.setSource(limelightCamera);
      
      ConsoleLogger.log("Limelight camera stream published on port 1181");
      ConsoleLogger.log("  Stream URL: http://limelight-front.local:5800");
      
    } catch (Exception e) {
      ConsoleLogger.logError("Failed to setup Limelight stream: " + e.getMessage());
    }
  }

  // Runs EVERY 20ms in ALL modes - this is the robot's heartbeat
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); // Run all active commands
    
    // Only check battery during disabled mode (before match starts)
    if (!matchActive) {
      checkBatteryVoltage();
    }
    
    Logger.recordOutput("Battery/Voltage", getBatteryVoltage());
    Logger.recordOutput("Battery/Current", RobotController.getInputCurrent());

    // NEW: Update tunable PID values
    if (robotContainer != null) {
      robotContainer.periodic();
    }
    
    // NEW: Update Elastic Dashboard
    elasticDashboard.update();
  }

  // Check battery and warn driver if low (only during disabled, prevents spam during match)
  private void checkBatteryVoltage() {
    double voltage = getBatteryVoltage();
    
    if (voltage < 10.0 && !criticalBatteryWarningShown) { // CRITICAL: Robot might shut off
      ConsoleLogger.logError("*** CRITICAL BATTERY: " + voltage + "V - CHARGE IMMEDIATELY!");
      //DriverStation.reportError("CRITICAL BATTERY: " + voltage + "V", false);
      criticalBatteryWarningShown = true;
    } else if (voltage < 11.9 && !lowBatteryWarningShown) { // WARNING: Charge soon
      ConsoleLogger.logError("!!! LOW BATTERY: " + voltage + "V - charge before next match");
      //DriverStation.reportWarning("Low battery: " + voltage + "V", false);
      lowBatteryWarningShown = true;
    } else if (voltage > 12.0) { // Reset flags when voltage recovers
      lowBatteryWarningShown = false;
      criticalBatteryWarningShown = false;
    }
  }

  private double getBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  @Override
  public void disabledInit() {
    matchActive = false; // Re-enable battery warnings
    robotState.setEnabled(false);
    robotState.setMode(RobotState.Mode.DISABLED); // FIXED: Use DISABLED
    ConsoleLogger.log("Robot DISABLED");
  }

  @Override
  public void disabledPeriodic() {} // Nothing needed - LED updates in their periodic()

  @Override
  public void disabledExit() {
    // Nothing needed - AdvantageKit handles logging automatically
  }

  @Override
  public void autonomousInit() {
    matchActive = true; // Disable battery warnings during match
    robotState.setEnabled(true);
    robotState.setMode(RobotState.Mode.ENABLED_AUTO); // FIXED: Use ENABLED_AUTO
    
    ConsoleLogger.log("========================================");
    
    ConsoleLogger.log("========================================");
    ConsoleLogger.log(">>> AUTONOMOUS MODE STARTED <<<");
    
    autonomousCommand = robotContainer.getAutonomousCommand();
    
    if (autonomousCommand != null) {
      String autoName = autonomousCommand.getName();
      ConsoleLogger.log("Running auto: " + autoName);
      Logger.recordOutput("Auto/SelectedName", autoName);
      
      if (getBatteryVoltage() < 10) { // Warn if battery low for auto
        ConsoleLogger.logError("WARNING: Low battery for auto (" + getBatteryVoltage() + "V)");
        //DriverStation.reportWarning("Low battery for auto!", false);
      }
      
      autonomousCommand.schedule();
      ConsoleLogger.log("Auto command scheduled");
    } else {
      ConsoleLogger.logError("NO AUTO SELECTED!");
      //DriverStation.reportError("No auto selected!", false);
      Logger.recordOutput("Auto/SelectedName", "NONE");
    }
    
    ConsoleLogger.log("========================================");
  }

  @Override
  public void autonomousPeriodic() {} // CommandScheduler (robotPeriodic) handles auto commands

  @Override
  public void teleopInit() {
    matchActive = true; // Disable battery warnings during match
    
    if (autonomousCommand != null) { // Stop auto if still running
      autonomousCommand.cancel();
      ConsoleLogger.log("Auto command canceled - driver in control");
      robotState.setEnabled(true);
      robotState.setMode(RobotState.Mode.ENABLED_TELEOP); // FIXED: Use ENABLED_TELEOP
      
      ConsoleLogger.log(">>> TELEOP started - driver has control");
    }
  }

  @Override
  public void teleopPeriodic() {} // Controller input handled by button bindings

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll(); // Stop all commands in test mode
    robotState.setMode(RobotState.Mode.TEST); // TEST is still valid
    ConsoleLogger.log("TEST mode started");
  }

  @Override
  public void testPeriodic() {} // Add test mode controls here if needed

  @Override
  public void simulationInit() {
    ConsoleLogger.log("Simulation mode started");
  }

  @Override
  public void simulationPeriodic() {} // Update simulated sensors/motors here if using physics sim
}