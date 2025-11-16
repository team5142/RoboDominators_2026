package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;  // ✅ Import LoggedRobot
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {  // ✅ Extend LoggedRobot instead of TimedRobot
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    // ✅ Setup AdvantageKit logging
    Logger.recordMetadata("ProjectName", "RoboDominators_2026");
    Logger.recordMetadata("TeamNumber", "5142");
    
    // Log to NetworkTables for live AdvantageScope viewing
    Logger.addDataReceiver(new NT4Publisher());
    
    // Save logs to USB drive for later review
    Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
    
    // Start logging
    Logger.start();
    
    System.out.println("AdvantageKit logging started!");

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    robotContainer.getRobotState().setEnabled(false);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    robotContainer.getRobotState().setEnabled(true);
    robotContainer.getRobotState().setMode(RobotState.Mode.AUTO);
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.getRobotState().setEnabled(true);
    robotContainer.getRobotState().setMode(RobotState.Mode.TELEOP);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.getRobotState().setMode(RobotState.Mode.TEST);
  }

  @Override
  public void testPeriodic() {}
}
