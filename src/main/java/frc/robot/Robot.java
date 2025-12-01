package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

// Main robot class - runs on boot, manages all modes (auto/teleop/test), 20ms loop
public class Robot extends LoggedRobot {
  
  private Command autonomousCommand; // Selected auto routine
  private RobotContainer robotContainer; // Holds all subsystems and commands
  private RobotState robotState; // Robot state tracker
  
  private boolean lowBatteryWarningShown = false; // Prevent spam
  private boolean criticalBatteryWarningShown = false;
  private boolean matchActive = false; // Track if we're in a match (auto/teleop/test)

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
    
    System.out.println("========================================");
    System.out.println("RoboDominators 5142 - Lebron2");
    System.out.println("AdvantageKit logging: ACTIVE");
    System.out.println("Battery: " + getBatteryVoltage() + "V");
    System.out.println("========================================");
    try {
      robotState = new RobotState();
      robotContainer = new RobotContainer();
      System.out.println("Robot container initialized successfully");
      System.out.println("Robot container initialized successfully");
    } catch (Exception e) {
      System.err.println("FATAL ERROR during robot initialization:");
      e.printStackTrace();
      DriverStation.reportError("Robot failed to initialize: " + e.getMessage(), true);
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
  }

  // Check battery and warn driver if low (only during disabled, prevents spam during match)
  private void checkBatteryVoltage() {
    double voltage = getBatteryVoltage();
    
    if (voltage < 10.0 && !criticalBatteryWarningShown) { // CRITICAL: Robot might shut off
      System.err.println("*** CRITICAL BATTERY: " + voltage + "V - CHARGE IMMEDIATELY!");
      DriverStation.reportError("CRITICAL BATTERY: " + voltage + "V", false);
      criticalBatteryWarningShown = true;
    } else if (voltage < 11.9 && !lowBatteryWarningShown) { // WARNING: Charge soon
      System.err.println("!!! LOW BATTERY: " + voltage + "V - charge before next match");
      DriverStation.reportWarning("Low battery: " + voltage + "V", false);
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
    System.out.println("Robot DISABLED");
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
    
    System.out.println("========================================");
    
    System.out.println("========================================");
    System.out.println(">>> AUTONOMOUS MODE STARTED <<<");
    
    autonomousCommand = robotContainer.getAutonomousCommand();
    
    if (autonomousCommand != null) {
      String autoName = autonomousCommand.getName();
      System.out.println("Running auto: " + autoName);
      Logger.recordOutput("Auto/SelectedName", autoName);
      
      if (getBatteryVoltage() < 10) { // Warn if battery low for auto
        System.err.println("WARNING: Low battery for auto (" + getBatteryVoltage() + "V)");
        DriverStation.reportWarning("Low battery for auto!", false);
      }
      
      autonomousCommand.schedule();
      System.out.println("Auto command scheduled");
    } else {
      System.err.println("NO AUTO SELECTED!");
      DriverStation.reportError("No auto selected!", false);
      Logger.recordOutput("Auto/SelectedName", "NONE");
    }
    
    System.out.println("========================================");
  }

  @Override
  public void autonomousPeriodic() {} // CommandScheduler (robotPeriodic) handles auto commands

  @Override
  public void teleopInit() {
    matchActive = true; // Disable battery warnings during match
    
    if (autonomousCommand != null) { // Stop auto if still running
      autonomousCommand.cancel();
      System.out.println("Auto command canceled - driver in control");
      robotState.setEnabled(true);
      robotState.setMode(RobotState.Mode.ENABLED_TELEOP); // FIXED: Use ENABLED_TELEOP
      
      System.out.println(">>> TELEOP started - driver has control");
    }
  }

  @Override
  public void teleopPeriodic() {} // Controller input handled by button bindings

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll(); // Stop all commands in test mode
    robotState.setMode(RobotState.Mode.TEST); // TEST is still valid
    System.out.println("TEST mode started");
  }

  @Override
  public void testPeriodic() {} // Add test mode controls here if needed

  @Override
  public void simulationInit() {
    System.out.println("Simulation mode started");
  }

  @Override
  public void simulationPeriodic() {} // Update simulated sensors/motors here if using physics sim
}