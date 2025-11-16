package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;

public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private Command autonomousCommand;

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "RoboDominators_2026");
    Logger.recordMetadata("Robot", "Offseason CTRE + Advantage template");
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
