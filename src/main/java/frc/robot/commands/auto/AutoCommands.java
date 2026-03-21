package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
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
            ClimberSubsystem climber,
            RobotState robotState) {

        // --- Intake ---
        if (intake != null) {
            // Deploy: extend arm only — rollers stay off until explicitly started
            NamedCommands.registerCommand("IntakeDeploy", Commands.runOnce(() -> {
                intake.extendOnly();
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

            // Start rollers only — arm position unchanged
            NamedCommands.registerCommand("IntakeRollersOn", Commands.runOnce(
                intake::spinIn, intake));

            // Stop rollers only — arm position unchanged
            NamedCommands.registerCommand("IntakeRollersOff", Commands.runOnce(
                intake::stopRollers, intake));

            // Agitate: brief reverse pulse to free a stuck ball — arm position unchanged
            NamedCommands.registerCommand("IntakeAgitate", Commands.runOnce(
                intake::agitate, intake));
        }

        // ShootStart/ShootStop do not require turret — enableFire/disableFire are state flags only
        // and must not interrupt the tracking default command.
        if (turret != null && spindexer != null && singulator != null) {
            NamedCommands.registerCommand("ShootStart", Commands.runOnce(() -> {
                turret.enableFire();
                spindexer.spinForward();
                singulator.primeAndFeed();
            }, spindexer, singulator));

            NamedCommands.registerCommand("ShootStop", Commands.runOnce(() -> {
                turret.disableFire();
                spindexer.stop();
                singulator.pause();
            }, spindexer, singulator));
        }

        // Flywheel spin-up / spin-down — call FlywheelsOn before ShootStart, FlywheelsOff after ShootStop.
        // Uses HUBCLOSE RPS targets as initial speed; aim pipeline overrides dynamically once tracking enabled.
        // TrackingEnable allows the aim pipeline to run — call once at auto start after turret is homed.
        if (turret != null) {
            NamedCommands.registerCommand("TrackingEnable", Commands.runOnce(
                turret::enableTracking, turret));

            NamedCommands.registerCommand("FlywheelsOn", Commands.runOnce(() -> {
                turret.setFlywheelFrontRps(Constants.TurretTargets.HUBCLOSE_FRONT_RPS);
                turret.setFlywheelBackRps(Constants.TurretTargets.HUBCLOSE_BACK_RPS);
            }, turret));

            NamedCommands.registerCommand("FlywheelsOff", Commands.runOnce(() ->
                turret.setFlywheelPercent(0.0), turret));
        }

        // --- Meta commands (combinations for simple autos) ---
        // AutoInit: set flywheelOn flag (default command picks it up next loop) + extend intake.
        // Does not require turret — avoids interrupting the tracking default command.
        if (turret != null && intake != null) {
            NamedCommands.registerCommand("AutoInit", Commands.runOnce(() -> {
                robotState.setFlywheelOn(true);
                intake.extendOnly();
            }, intake));
        } else if (turret != null) {
            NamedCommands.registerCommand("AutoInit", Commands.runOnce(() -> {
                robotState.setFlywheelOn(true);
            }));
        }

        // AutoShootEnd: stop fire interlock, feed chain, and flywheels in one call.
        // Does not require turret — avoids interrupting the tracking default command.
        if (turret != null && spindexer != null && singulator != null) {
            NamedCommands.registerCommand("AutoShootEnd", Commands.runOnce(() -> {
                turret.disableFire();
                robotState.setFlywheelOn(false);
                spindexer.stop();
                singulator.pause();
            }, spindexer, singulator));
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
