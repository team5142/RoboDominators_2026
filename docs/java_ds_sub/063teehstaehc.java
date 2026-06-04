// ROBODOMINATORS 2026 - TEACHER CHEATSHEET: SAMPLE ANSWERS FOR TASKS 2-6
// Summer2026ProgrammingPractice Student Scaffold
// This is NOT meant to be compiled as part of the project - it's a reference file for teachers.

// ============================================================================
// TASK 2 - Declare a Motor (SpindexerSubsystem.java)
// ============================================================================

// Add these imports to SpindexerSubsystem.java:
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

// Add this field inside SpindexerSubsystem class:
private final SparkMax motor;

// Constructor body:
public SpindexerSubsystem() {
  motor = new SparkMax(Constants.Spindexer.MOTOR_ID, MotorType.kBrushless);
}

// ============================================================================
// TASK 3 - Spin the Motor Forward and Stop It (SpindexerSubsystem.java)
// ============================================================================

public void spinForward() {
  motor.set(Constants.Spindexer.FORWARD_SPEED);
}

public void stop() {
  motor.set(0.0);
}

// ============================================================================
// TASK 4 - Wire Right Bumper to spinForward() and stop() (RobotContainer.java)
// ============================================================================

// Add this inside configureButtonBindings() after the Back button binding:
new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
    .whileTrue(Commands.startEnd(
        () -> {
          if (spindexerSubsystem != null) spindexerSubsystem.spinForward();
        },
        () -> {
          if (spindexerSubsystem != null) spindexerSubsystem.stop();
        },
        spindexerSubsystem));

// ============================================================================
// TASK 5 - Add Reverse and Safety Stop (SpindexerSubsystem.java)
// ============================================================================

public void spinReverse() {
  motor.set(Constants.Spindexer.REVERSE_SPEED);
}

public void stopAll() {
  stop();
}

// ============================================================================
// TASK 6 - Wire Left Bumper to spinReverse() and stop() (RobotContainer.java)
// ============================================================================

// Add this inside configureButtonBindings() right after the Right Bumper binding:
new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
    .whileTrue(Commands.startEnd(
        () -> {
          if (spindexerSubsystem != null) spindexerSubsystem.spinReverse();
        },
        () -> {
          if (spindexerSubsystem != null) spindexerSubsystem.stop();
        },
        spindexerSubsystem));

// ============================================================================
// END OF CHEATSHEET
// ============================================================================
