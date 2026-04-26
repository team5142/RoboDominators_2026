package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

// Registers PathPlanner named commands for use in .auto files.
// Call register() once in RobotContainer before AutoBuilder.buildAutoChooser().
public final class AutoCommands {

    private AutoCommands() {}

    public static void register(
            IntakeSubsystem intake,
            SpindexerSubsystem spindexer,
            SingulatorSubsystem singulator) {

        if (intake != null) {
            NamedCommands.registerCommand("IntakeDeploy", Commands.runOnce(
                intake::extend, intake));

            NamedCommands.registerCommand("IntakeRetract", Commands.runOnce(() -> {
                intake.stopRollers();
                intake.retract();
            }, intake));

            NamedCommands.registerCommand("IntakeRollersOn", Commands.runOnce(
                intake::spinIn, intake));

            NamedCommands.registerCommand("IntakeRollersOff", Commands.runOnce(
                intake::stopRollers, intake));

            NamedCommands.registerCommand("IntakeDeployAndSpin", Commands.runOnce(
                intake::extendAndSpin, intake));
        }

        if (spindexer != null && singulator != null) {
            NamedCommands.registerCommand("FeedStart", Commands.runOnce(() -> {
                spindexer.spinForward();
                singulator.primeAndFeed();
            }, spindexer, singulator));

            NamedCommands.registerCommand("FeedStop", Commands.runOnce(() -> {
                spindexer.stop();
                singulator.pause();
            }, spindexer, singulator));
        }
    }
}