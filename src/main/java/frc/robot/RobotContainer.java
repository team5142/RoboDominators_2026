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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Intake;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.subsystems.*;
import frc.robot.util.SmartLogger;
import java.util.Optional;

// Wires up all subsystems, controllers, and commands.
// This is the central file where hardware meets code - every button binding lives here.
public class RobotContainer {

  public static final boolean COMPETITION_MODE = false;
  private static final boolean ENABLE_INTAKE     = true;
  private static final boolean ENABLE_SPINDEXER  = true;
  private static final boolean ENABLE_SINGULATOR = true;

  private static Alliance cachedAlliance = Alliance.Blue;

  // Controllers - do not modify
  private final XboxController driverController   = new XboxController(DRIVER_CONTROLLER_PORT);
  private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  // Complex subsystems - pre-built, do not modify
  final RobotState robotState;
  final GyroSubsystem gyro;
  final QuestNavSubsystem questNav;
  final DriveSubsystem driveSubsystem;
  final PoseEstimatorSubsystem poseEstimator;

  // Student-built subsystems
  IntakeSubsystem intakeSubsystem;
  SpindexerSubsystem spindexerSubsystem;
  SingulatorSubsystem singulatorSubsystem;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer(RobotState robotState) {
    this.robotState = robotState;

    SmartLogger.configure(!COMPETITION_MODE);

    gyro           = new GyroSubsystem();
    questNav       = new QuestNavSubsystem();
    driveSubsystem = new DriveSubsystem(robotState, gyro);
    poseEstimator  = new PoseEstimatorSubsystem(driveSubsystem, robotState, questNav);

    // Task 9: update SpindexerSubsystem() to pass robotState once you add it to the constructor
    // Task 14: update SingulatorSubsystem() similarly
    intakeSubsystem     = ENABLE_INTAKE     ? new IntakeSubsystem(robotState) : null;
    spindexerSubsystem  = ENABLE_SPINDEXER  ? new SpindexerSubsystem(robotState)        : null;
    singulatorSubsystem = ENABLE_SINGULATOR ? new SingulatorSubsystem(robotState)       : null;

    updateAllianceFromDriverStation();
    robotState.setAlliance(cachedAlliance);

    configurePathPlanner();
    AutoCommands.register(intakeSubsystem, spindexerSubsystem, singulatorSubsystem);
    configureDefaultCommands();
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    poseEstimator.setAutoChooser(autoChooser);

    SmartLogger.logConsole("RobotContainer initialized", "Init");
  }

  // PathPlanner configuration - pre-built, do not modify
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

  // Default commands run whenever no other command requires a subsystem.
  // The drivetrain default is DriveWithJoysticks - pre-built, do not modify.
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

  private void configureButtonBindings() {

    // DRIVER: Back button resets field orientation - pre-built, do not modify
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(driveSubsystem.createOrientToFieldCommand(robotState));

    /*
     * TASK 4 - Wire the Right Bumper to spinForward() and stop()
     * -----------------------------------------------------------------------
     * whileTrue() with Commands.startEnd() is how you run something while a
     * button is held and clean up when it is released.
     *
     * The startEnd pattern takes two lambdas:
     *   - First lambda runs when the button is pressed
     *   - Second lambda runs when the button is released
     * Followed by the subsystems it requires (for safety scheduling).
     *
     * Pattern to use:
     *   new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
     *       .whileTrue(Commands.startEnd(
     *           () -> { // pressed - call spinForward here ; },
     *           () -> { // released - call stop here ; },
     *           spindexerSubsystem));
     *
     * Always null-check before calling: if (spindexerSubsystem != null) ...
     *
     * When done: compile and deploy.
     * [ROBOT OPTIONAL] Hold Right Bumper - the spindexer should spin.
     */
     new JoystickButton(operatorController,XboxController.Button.kRightBumper.value)
      .whileTrue(Commands.startEnd(
        () -> {spindexerSubsystem.motorForward();
               System.out.println("Spindexer on");
              singulatorSubsystem.spinFeed();},
        () -> {spindexerSubsystem.motorStop();
              singulatorSubsystem.pause();
               System.out.println("Spindexer off");},
        spindexerSubsystem,
        singulatorSubsystem));
      

    /* 
     * Come back here for Task 6.
     * -----------------------------------------------------------------------
     */
 
    /*
     * TASK 6 - Wire the Left Bumper to spinReverse() and stop()
     * -----------------------------------------------------------------------
     * Same pattern as Task 4 - whileTrue + startEnd.
     * Left Bumper: XboxController.Button.kLeftBumper
     *
     * When done: compile and move to Task 7 in Robot.java.
     * -----------------------------------------------------------------------
     */
    new JoystickButton(operatorController,XboxController.Button.kLeftBumper.value)
      .whileTrue(Commands.startEnd(
        () -> {spindexerSubsystem.motorBackward();
              System.out.println("Spindexer on");},
        () -> {spindexerSubsystem.motorStop();
              System.out.println("Spindexer off");},
        spindexerSubsystem));
    new JoystickButton(operatorController,XboxController.Button.kA.value)
      .onTrue(Commands.runOnce(
    () -> {singulatorSubsystem.spinFeed();},
    singulatorSubsystem))
      .onFalse(Commands.runOnce(
    () -> {singulatorSubsystem.pause();},
      singulatorSubsystem));
      
    /*
     * TASK 16 - Combine Spindexer and Singulator on Right Bumper
     * -----------------------------------------------------------------------
     * Now that both subsystems exist, update the Right Bumper binding to call
     * both at the same time. One lambda can call multiple methods.
     *
     * Think about: when the button is pressed, what should both mechanisms do?
     * When it is released, what should both do?
     *
     * Singulator method to call when feeding: spinFeed()
     * Singulator method to call on release:   pause()
     *
     * Make sure to include both subsystems in the requirements list at the end
     * of startEnd() so WPILib knows both are being used.
     *
     * When done: compile and move to Task 17a in SingulatorSubsystem.java.
     * -----------------------------------------------------------------------
     */

    /*
     * TASK 17b - Add a Release-to-Trigger Binding (onFalse)
     * -----------------------------------------------------------------------
     * Sometimes you want an action to happen when a button is RELEASED rather
     * than pressed. onFalse() fires once when the button goes from held to released.
     *
     * Add a binding on the A button that calls singulatorSubsystem.spinFeed()
     * when pressed (onTrue), and singulatorSubsystem.pause() when released (onFalse).
     * This gives you manual single-shot control.
     *
     * Pattern hint:
     *   new JoystickButton(...)
     *       .onTrue(Commands.runOnce(() -> { ... }))
     *       .onFalse(Commands.runOnce(() -> { ... }));
     *
     * When done: move to Task 18.
     * -----------------------------------------------------------------------
     */

    /*
     * TASK 18 - Wire the Y Button: Intake Arm Toggle (Conditional Logic)
     * -----------------------------------------------------------------------
     * The Y button should extend the arm if it is retracted, and retract it
     * if it is extended. This requires a conditional check inside the lambda.
     *
     * onTrue() with runOnce() fires once on press - no cleanup needed.
     *
     * Inside the lambda, check intakeSubsystem.isExtended() to decide which
     * method to call. Also call stopRollers() before retracting.
     *
     * Pattern hint (generic, not the actual code):
     *   Commands.runOnce(() -> {
     *     if (someCondition) {
     *       doThingA();
     *     } else {
     *       doThingB();
     *     }
     *   }, theSubsystem)
     *
     * When done: move to Task 19a in RobotState.java.
     * -----------------------------------------------------------------------
     */

    /*
     * TASK 19b - Wire B and X: Roller Control While Held
     * -----------------------------------------------------------------------
     * B button (hold): run intake rollers in (spinIn), stop on release
     * X button (hold): run intake rollers out (spinOut), stop on release
     *
     * You have done this pattern twice already (Tasks 4 and 6).
     * Write these two bindings without looking back at your previous work.
     *
     * When done: compile and move to Task 20a in DriveWithJoysticks.java.
     * -----------------------------------------------------------------------
     */

    /*
     * TASK 22b - Add a Precision Mode Button (Live Supplier)
     * -----------------------------------------------------------------------
     * DriveWithJoysticks already accepts a precisionModeSupplier - a lambda
     * that is called every loop to check if precision mode is active.
     * Right now it is hardcoded to () -> false in configureDefaultCommands().
     *
     * Update configureDefaultCommands() to pass a live supplier instead:
     *   () -> driverController.getLeftBumper()
     *
     * This is a supplier - the lambda is evaluated every 20ms, not just once.
     * That is different from a button binding, which fires on an edge.
     *
     * When done: deploy and hold Left Bumper while driving.
     * [ROBOT OPTIONAL] The robot should feel noticeably slower and more precise.
     * Then move to Task 23b.
     * -----------------------------------------------------------------------
     */

    /*
     * TASK 23b - Rewrite One Binding as a Method Reference
     * -----------------------------------------------------------------------
     * A method reference is a shorthand for a lambda that only calls one method.
     *
     * Lambda:           () -> spindexerSubsystem.stop()
     * Method reference: spindexerSubsystem::stop
     *
     * Both do exactly the same thing - the second is just more concise.
     *
     * Find one of your existing bindings that only calls a single method and
     * rewrite it using the :: syntax. Compile and verify it still works.
     *
     * When done: move to Task 24 in SnapToHeadingFixed.java.
     * -----------------------------------------------------------------------
     */

    /*
     * TASK 26 - Wire D-Pad to Snap-to-Heading (Passing a Value to a Command)
     * -----------------------------------------------------------------------
     * SnapToHeadingFixed is a Command class - you create it with new, passing
     * arguments in the constructor. This is different from Commands.runOnce()
     * which runs an inline lambda.
     *
     * The D-pad on an XboxController uses POVButton instead of JoystickButton.
     * The angle argument is the D-pad direction in degrees (0 = up, 90 = right, etc).
     *   new POVButton(driverController, 0)  // up
     *   new POVButton(driverController, 90) // right
     *
     * You will need to import: edu.wpi.first.wpilibj2.command.button.POVButton
     *
     * For each of the four D-pad directions, create a whileTrue binding that
     * starts a new SnapToHeadingFixed pointed at that cardinal angle.
     * Pass the left stick suppliers for translation, and a fixed heading value
     * as a supplier: () -> 0.0 for north, () -> 90.0 for east, etc.
     *
     * When done: move to Task 27.
     * -----------------------------------------------------------------------
     */

    /*
     * TASK 27 - Add a Toggle Mode for the Spindexer
     * -----------------------------------------------------------------------
     * toggleOnTrue() is like onTrue(), but pressing again stops the command.
     * Good for "set it and forget it" spinning without holding the button.
     *
     * Add a binding on the operatorController Start button using toggleOnTrue()
     * that runs the spindexer forward continuously until the button is pressed again.
     *
     * You will need Commands.startEnd() or Commands.run() - think about which
     * one makes more sense for a toggle vs a hold.
     *
     * When done: move to Task 28 (vision).
     * -----------------------------------------------------------------------
     */

    /*
     * TASK 28 - Gate a Binding on Vision Detection (.and() trigger chaining)
     * -----------------------------------------------------------------------
     * Trigger conditions can be combined with .and() so a binding only fires
     * when multiple conditions are true at the same time.
     *
     * Add a binding that runs spinIn() on the intake only while the B button
     * is held AND the object detection camera currently sees a target.
     *
     * You will need a reference to the object vision subsystem - check
     * RobotContainer fields to see if it is already available.
     *
     * Pattern hint (generic):
     *   new JoystickButton(controller, button)
     *       .and(() -> someConditionIsTrue())
     *       .whileTrue(Commands.runOnce(() -> { ... }));
     *
     * When done: move to Task 29.
     * -----------------------------------------------------------------------
     */

    /*
     * TASK 29 - Display Vision Status on the Dashboard
     * -----------------------------------------------------------------------
     * SmartDashboard.putBoolean(key, value) sends a boolean to the driver
     * dashboard and to AdvantageScope. Call it in the periodic() method below.
     *
     * Add a line that posts whether the limelight currently sees any target.
     * You will need to find the right method on the vision subsystem to call.
     *
     * When done: move to Task 30.
     * -----------------------------------------------------------------------
     */

    /*
     * TASK 30 - Aim at a Detected Object with SnapToHeadingDynamic
     * -----------------------------------------------------------------------
     * SnapToHeadingDynamic takes a supplier for the target heading instead of
     * a fixed value - the heading is recalculated every loop.
     *
     * Wire the Right Stick button (kRightStick) to a whileTrue binding that
     * starts a SnapToHeadingDynamic aimed at the object detection camera target.
     *
     * You will need to figure out what supplier to pass - look at what the
     * object vision subsystem exposes and how heading relates to yaw offset.
     *
     * This is an advanced task - there is no exact answer. Experiment.
     * -----------------------------------------------------------------------
     */
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
