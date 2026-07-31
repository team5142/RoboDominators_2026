package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.util.SmartLogger;

// Main robot class - runs on boot, manages all modes (auto/teleop/test), 20ms loop
public class Robot extends LoggedRobot {

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private RobotState robotState;

  private double autoStart;
  private boolean autoMessagePrinted;
  private boolean matchActive = false;

  private double cachedBatteryVoltage = 0.0;
  private boolean lowBatteryWarningShown = false;
  private boolean criticalBatteryWarningShown = false;

  // Runs ONCE at robot boot - set up logging and create subsystems
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "RoboDominators_2026");
    Logger.recordMetadata("TeamNumber", "5142");
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
    Logger.start();

    SmartLogger.logConsole("Robot booting... Battery: " + RobotController.getBatteryVoltage() + "V", "Robot Init");

    try {
      robotState = new RobotState();
      robotContainer = new RobotContainer(robotState);
      SmartLogger.logConsole("Robot initialized successfully");
    } catch (Exception e) {
      SmartLogger.logConsoleError("FATAL: Robot failed to initialize: " + e.getMessage());
      DriverStation.reportError("Robot failed to initialize: " + e.getMessage(), true);
    }
  }

  // Runs EVERY 20ms in ALL modes - the robot heartbeat
  @Override
  public void robotPeriodic() {
    // Run all scheduled commands - this is the core of WPILib's command-based framework
    CommandScheduler.getInstance().run();

    cachedBatteryVoltage = RobotController.getBatteryVoltage();
    Logger.recordOutput("Battery/Voltage", cachedBatteryVoltage);

    if (!matchActive) {
      checkBatteryVoltage();
    }

    if (robotContainer != null) {
      robotContainer.periodic();
    }

    // Print how long the auto took once it finishes
    if (autonomousCommand != null && !autonomousCommand.isScheduled() && !autoMessagePrinted) {
      autoMessagePrinted = true;
      double elapsed = Timer.getTimestamp() - autoStart;
      SmartLogger.logConsole(String.format("Auto finished in %.2f seconds", elapsed), "Auto");
    }
  }

  // Warn drivers when battery is getting low
  private void checkBatteryVoltage() {
    if (cachedBatteryVoltage < 10.0 && !criticalBatteryWarningShown) {
      SmartLogger.logConsoleError("CRITICAL BATTERY: " + cachedBatteryVoltage + "V - CHARGE NOW!");
      criticalBatteryWarningShown = true;
    }
    if (cachedBatteryVoltage > 10.2) criticalBatteryWarningShown = false;

    if (cachedBatteryVoltage < 11.4 && !lowBatteryWarningShown) {
      SmartLogger.logConsoleError("LOW BATTERY: " + cachedBatteryVoltage + "V");
      lowBatteryWarningShown = true;
    }
    if (cachedBatteryVoltage > 11.6) lowBatteryWarningShown = false;
  }

  @Override
  public void disabledInit() {
    matchActive = false;
    if (robotState != null) {
      robotState.setEnabled(false);
      robotState.setMode(RobotState.Mode.DISABLED);
    }
    // Stop all mechanism motors when the robot is disabled
    if (robotContainer != null) {
      /* TASK 7 - Add stopAll() Safety Calls
       * When the robot disables, all motors should stop immediately.
       * Uncomment each line below once you have added stopAll() to that subsystem.
       * This is a safety requirement - motors must not run while the robot is disabled.
       */
       if (robotContainer.spindexerSubsystem  != null) robotContainer.spindexerSubsystem.stopAll();
       if (robotContainer.singulatorSubsystem != null) robotContainer.singulatorSubsystem.stopAll();
       if (robotContainer.intakeSubsystem     != null) robotContainer.intakeSubsystem.stopAll();
    }
    Logger.recordOutput("Robot/Mode", "DISABLED");
    SmartLogger.logConsole("Robot DISABLED");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    if (robotState == null || robotContainer == null) return;
    autoStart = Timer.getTimestamp();
    autoMessagePrinted = false;
    matchActive = true;
    robotState.setEnabled(true);
    robotState.setMode(RobotState.Mode.ENABLED_AUTO);

    Logger.recordOutput("Robot/Mode", "AUTO");
    SmartLogger.logConsole(">>> AUTO STARTED <<<", "Auto");

    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      SmartLogger.logConsole("Running: " + autonomousCommand.getName(), "Auto");
      CommandScheduler.getInstance().schedule(autonomousCommand);
    } else {
      SmartLogger.logConsoleError("No auto selected!");
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (robotState == null || robotContainer == null) return;
    matchActive = true;

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotState.setEnabled(true);
    robotState.setMode(RobotState.Mode.ENABLED_TELEOP);

    Logger.recordOutput("Robot/Mode", "TELEOP");
    SmartLogger.logConsole(">>> TELEOP STARTED <<<");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    Logger.recordOutput("Robot/Mode", "TEST");
    SmartLogger.logConsole("TEST mode started");
  }

  @Override
  public void testPeriodic() {}
}