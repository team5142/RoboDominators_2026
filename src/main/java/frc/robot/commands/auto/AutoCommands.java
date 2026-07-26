package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

/*
 * TASK 11 - Add Spindexer to the Auto Feed Commands
 * -----------------------------------------------------------------------
 * PathPlanner autos call named commands by string name (e.g. "FeedStart").
 * These are registered here so PathPlanner knows what Java code to run.
 *
 * The FeedStart and FeedStop commands below already call the singulator.
 * Add spindexer calls to them so both mechanisms run together during auto.
 *
 * Inside the FeedStart lambda: call spindexer.spinForward() alongside the singulator call.
 * Inside the FeedStop lambda:  call spindexer.stop() alongside the singulator call.
 *
 * When done: compile and move to Task 12 in Constants.java.
 * -----------------------------------------------------------------------
 */

/*
 * TASK 31 - Register a New Named Command
 * -----------------------------------------------------------------------
 * Named commands are how you trigger Java code from a PathPlanner auto path.
 * Any method call can become a named command - you are not limited to what is here.
 *
 * Add at least one new named command of your own. Some ideas:
 *   "SpindexerForward"  - just runs the spindexer
 *   "IntakeDeploy"      - extends the intake arm
 *   "IntakeRetract"     - retracts and stops rollers
 *
 * Pattern - registering a command that calls one method:
 *   NamedCommands.registerCommand("CommandName", Commands.runOnce(
 *       () -> subsystem.method(), subsystem));
 *
 * After registering it here, open PathPlanner and you will see it in the
 * named commands list. Drop it into an auto path and test it.
 *
 * When done: move to Task 32 in PathPlanner (no Java changes needed).
 * -----------------------------------------------------------------------
 */

// Registers PathPlanner named commands. Call register() once in RobotContainer.
public final class AutoCommands {

    private AutoCommands() {}

    public static void register(
            IntakeSubsystem intake,
            SpindexerSubsystem spindexer,
            SingulatorSubsystem singulator) {

        if (intake != null) {
            // Task 24 unlock: uncomment each line below once you add the method to IntakeSubsystem

            // NamedCommands.registerCommand("IntakeDeploy", Commands.runOnce(
            //     intake::extend, intake));

            // NamedCommands.registerCommand("IntakeRetract", Commands.runOnce(() -> {
            //     intake.stopRollers();
            //     intake.retract();
            // }, intake));

            // NamedCommands.registerCommand("IntakeRollersOn", Commands.runOnce(
            //     intake::spinIn, intake));

            // NamedCommands.registerCommand("IntakeRollersOff", Commands.runOnce(
            //     intake::stopRollers, intake));
        }

        if (spindexer != null && singulator != null) {
            NamedCommands.registerCommand("FeedStart", Commands.runOnce(
                () -> {spindexer.motorForward();}
                // Task 16 unlock: singulator.spinFeed();
                , spindexer, singulator)); }

            NamedCommands.registerCommand("FeedStop", Commands.runOnce(() -> {
                // Task 11 unlock: spindexer.stop();
                // Task 16 unlock: singulator.pause();
            }, spindexer, singulator));
        }
    }

