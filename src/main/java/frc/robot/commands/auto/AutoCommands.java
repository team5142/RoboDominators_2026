package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

// Registers PathPlanner named commands for use in .auto files.
// Call register() once in RobotContainer before AutoBuilder.buildAutoChooser().
// Any null subsystem argument silently skips that group of commands.
public final class AutoCommands {

    private AutoCommands() {}

    public static void register(
            IntakeSubsystem intake,
            TurretSubsystem turret,
            SpindexerSubsystem spindexer,
            SingulatorSubsystem singulator,
            ClimberSubsystem climber) {

        // --- Intake ---
        if (intake != null) {
            // Deploy: extend arm and start pulling game pieces in
            NamedCommands.registerCommand("IntakeDeploy", Commands.runOnce(() -> {
                intake.extend();
                intake.spinIn();
            }, intake));

            // Only spin if arm is fully extended — avoids ejecting inside the frame
            NamedCommands.registerCommand("IntakeReverse", Commands.runOnce(() -> {
                if (intake.isExtended()) intake.spinOut();
            }, intake));

            // Only spin if arm is fully extended — avoids intaking while retracting
            NamedCommands.registerCommand("IntakeForward", Commands.runOnce(() -> {
                if (intake.isExtended()) intake.spinIn();
            }, intake));

            // Retract: stop rollers first, then bring arm back inside frame
            NamedCommands.registerCommand("IntakeRetract", Commands.runOnce(() -> {
                intake.stopRollers();
                intake.retract();
            }, intake));
        }

        // --- Shoot ---
        // ShootStart enables the fire interlock and starts the feed chain.
        // ShootStop disables the interlock and drains the feed chain.
        // Flywheel / hood / turret aim run continuously in TurretSubsystem — no command needed.
        if (turret != null && spindexer != null && singulator != null) {
            NamedCommands.registerCommand("ShootStart", Commands.runOnce(() -> {
                turret.enableFire();
                spindexer.spinForward();
                singulator.primeAndFeed();
            }, turret, spindexer, singulator));

            NamedCommands.registerCommand("ShootStop", Commands.runOnce(() -> {
                turret.disableFire();
                spindexer.stop();
                singulator.pause();
            }, turret, spindexer, singulator));
        }

        // --- Climber ---
        if (climber != null) {
            // Deploy: pivot the arm out to the cage
            NamedCommands.registerCommand("ClimberDeploy", Commands.runOnce(() ->
                climber.setRotationPercent(Constants.Climber.ROTATION_SPEED), climber));

            // Pull: retract the hook to lift the robot
            NamedCommands.registerCommand("ClimberPull", Commands.runOnce(() ->
                climber.setPullPercent(Constants.Climber.PULL_SPEED), climber));

            // Stop all climber motion
            NamedCommands.registerCommand("ClimberStop", Commands.runOnce(
                climber::stopAll, climber));
        }
    }
}
