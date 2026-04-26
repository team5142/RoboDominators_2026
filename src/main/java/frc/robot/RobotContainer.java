package frc.robot;

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.Auto.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.subsystems.*;
import frc.robot.util.SmartLogger;
import java.util.Optional;

// Wires up robot hardware, controllers, and commands.
// Call register() once before AutoBuilder.buildAutoChooser().
public class RobotContainer {

  // === CONFIGURATION FLAGS ===
  public static final boolean COMPETITION_MODE = false;
  private static final boolean ENABLE_INTAKE     = true;
  private static final boolean ENABLE_SPINDEXER  = true;
  private static final boolean ENABLE_SINGULATOR = true;

  private static Alliance cachedAlliance = Alliance.Blue;

  // Controllers
  private final XboxController driverController   = new XboxController(DRIVER_CONTROLLER_PORT);
  private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  // === SUBSYSTEMS ===
  // These are kept as reference for students  the complex ones are present but not wired to buttons.
  final RobotState robotState;
  final GyroSubsystem gyro;
  final QuestNavSubsystem questNav;
  final DriveSubsystem driveSubsystem;
  final PoseEstimatorSubsystem poseEstimator;

  // Student subsystems  these are the ones students will implement
  IntakeSubsystem intakeSubsystem;
  SpindexerSubsystem spindexerSubsystem;
  SingulatorSubsystem singulatorSubsystem;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer(RobotState robotState) {
    this.robotState = robotState;

    SmartLogger.configure(!COMPETITION_MODE);

    // Initialize subsystems
    gyro           = new GyroSubsystem();
    questNav       = new QuestNavSubsystem();
    driveSubsystem = new DriveSubsystem(robotState, gyro);
    poseEstimator  = new PoseEstimatorSubsystem(driveSubsystem, robotState, questNav);

    intakeSubsystem     = ENABLE_INTAKE     ? new IntakeSubsystem(robotState)     : null;
    spindexerSubsystem  = ENABLE_SPINDEXER  ? new SpindexerSubsystem(robotState)  : null;
    singulatorSubsystem = ENABLE_SINGULATOR ? new SingulatorSubsystem(robotState) : null;

    updateAllianceFromDriverStation();
    robotState.setAlliance(cachedAlliance);

    configurePathPlanner();
    AutoCommands.register(intakeSubsystem, spindexerSubsystem, singulatorSubsystem);
    configureDefaultCommands();
    configureButtonBindings();

    // Auto chooser  add more options here as autos are built
    autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    poseEstimator.setAutoChooser(autoChooser);

    SmartLogger.logConsole("RobotContainer initialized", "Init");
  }

  // Connects PathPlanner to the drivetrain for path following
  private void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          poseEstimator::getEstimatedPose,
          (pose) -> poseEstimator.resetPose(pose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions()),
          driveSubsystem::getRobotRelativeSpeeds,
          (speeds, ff) -> driveSubsystem.driveRobotRelative(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD),
              new PIDConstants(ROTATION_KP, ROTATION_KI, ROTATION_KD)),
          config,
          () -> isRedAlliance(),
          driveSubsystem);
      SmartLogger.logConsole("PathPlanner configured", "PathPlanner");
    } catch (Exception e) {
      SmartLogger.logConsoleError("PathPlanner config failed: " + e.getMessage());
    }
  }

  // Default commands run when no other command requires a subsystem
  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(
        new DriveWithJoysticks(
            driveSubsystem,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> true,
            () -> false));
  }

  // Button bindings  map controller buttons to commands
  private void configureButtonBindings() {
    // DRIVER: Back button resets field orientation
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(driveSubsystem.createOrientToFieldCommand(robotState));

    // OPERATOR: Y  toggle intake arm extend/retract
    new JoystickButton(operatorController, XboxController.Button.kY.value)
        .onTrue(Commands.runOnce(() -> {
          if (intakeSubsystem == null) return;
          if (intakeSubsystem.isExtended()) {
            intakeSubsystem.stopRollers();
            intakeSubsystem.retract();
          } else {
            intakeSubsystem.extend();
          }
        }));

    // OPERATOR: B (hold)  run intake rollers in, stop on release
    new JoystickButton(operatorController, XboxController.Button.kB.value)
        .whileTrue(Commands.startEnd(
            () -> { if (intakeSubsystem != null) intakeSubsystem.spinIn(); },
            () -> { if (intakeSubsystem != null) intakeSubsystem.stopRollers(); }));

    // OPERATOR: X (hold)  reverse intake rollers, stop on release
    new JoystickButton(operatorController, XboxController.Button.kX.value)
        .whileTrue(Commands.startEnd(
            () -> { if (intakeSubsystem != null) intakeSubsystem.spinOut(); },
            () -> { if (intakeSubsystem != null) intakeSubsystem.stopRollers(); }));

    // OPERATOR: Right trigger (hold)  run spindexer + singulator to feed/shoot
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(Commands.startEnd(
            () -> {
              if (spindexerSubsystem  != null) spindexerSubsystem.spinForward();
              if (singulatorSubsystem != null) singulatorSubsystem.primeAndFeed();
            },
            () -> {
              if (spindexerSubsystem  != null) spindexerSubsystem.stop();
              if (singulatorSubsystem != null) singulatorSubsystem.pause();
            }));

    // OPERATOR: Right stick down (> 0.9)  reverse spindexer + singulator to clear a jam
    new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(Commands.startEnd(
            () -> {
              if (spindexerSubsystem  != null) spindexerSubsystem.spinReverse();
              if (singulatorSubsystem != null) singulatorSubsystem.spinReverse();
            },
            () -> {
              if (spindexerSubsystem  != null) spindexerSubsystem.stop();
              if (singulatorSubsystem != null) singulatorSubsystem.pause();
            }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void periodic() {
    updateAllianceFromDriverStation();
    robotState.setAlliance(cachedAlliance);
    SmartDashboard.putBoolean("Robot/IsRedAlliance", cachedAlliance == Alliance.Red);
  }

  public static boolean isRedAlliance() {
    return cachedAlliance == Alliance.Red;
  }

  private void updateAllianceFromDriverStation() {
    Optional<Alliance> fmsAlliance = DriverStation.getAlliance();
    if (fmsAlliance.isPresent()) {
      cachedAlliance = fmsAlliance.get();
    }
  }
}
