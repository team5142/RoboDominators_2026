package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.SmartLogger;

// Cancels all currently scheduled commands immediately.
// Bind to an operator panel button or a dedicated e-stop trigger.
// Does not require any subsystem - safe to call at any time.
public class CancelAllCommand extends InstantCommand {
  @Override
  public void initialize() {
    SmartLogger.logConsole("CANCEL ALL triggered", "CancelAll");
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
