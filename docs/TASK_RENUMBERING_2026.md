# Task Renumbering - July 2026

## Summary
Tasks 12-23 have been renumbered with a/b suffixes to clarify the dual-task structure and eliminate confusion about duplicate task numbers.

## Complete Task Flow (Linear Path)

```
Task 1:  docs/GIT_GUIDE.md - Create branch
Task 2:  SpindexerSubsystem - Declare motor
Task 3:  SpindexerSubsystem - Add spinForward/stop
Task 4:  RobotContainer - Wire right bumper
Task 5:  SpindexerSubsystem - Add spinReverse
Task 6:  RobotContainer - Wire left bumper
Task 7:  Robot.java - Add stopAll safety
Task 8:  SpindexerSubsystem - Track state
Task 9:  RobotContainer - Update constructor (note)
Task 10: SpindexerSubsystem - Log data
Task 11: AutoCommands - Add spindexer

Task 12a: Constants - Read Spindexer Constants
Task 12b: SpindexerSubsystem - Configure motor
Task 13a: Constants - Add Singulator Constants
Task 13b: SpindexerSubsystem - Jam Detection

Task 14: SingulatorSubsystem - Declare motor
Task 15: SingulatorSubsystem - Add methods
Task 16: RobotContainer - Combine spindexer+singulator

Task 17a: SingulatorSubsystem - Add periodic logging
Task 17b: RobotContainer - Release-to-trigger binding

Task 18: RobotContainer - Wire Y button

Task 19a: RobotState - Add IntakePosition enum
Task 19b: RobotContainer - Wire B/X buttons

Task 20a: RobotState - Add IntakeRollerState
Task 20b: DriveWithJoysticks - Read file

Task 21a: RobotState - Add SingulatorState
Task 21b: DriveWithJoysticks - Change input curve

Task 22a: IntakeSubsystem - Declare motors
Task 22b: RobotContainer - Precision mode

Task 23a: IntakeSubsystem - Add methods
Task 23b: RobotContainer - Method reference

Task 24: SnapToHeadingFixed - Read file
Task 25: SnapToHeadingFixed - Tune PID
Task 26: RobotContainer - Wire D-Pad
Task 27: RobotContainer - Toggle mode
Task 28: RobotContainer - Gate on vision
Task 29: RobotContainer - Display vision status
Task 30: RobotContainer - Aim at object
Task 31: AutoCommands - Register named command
Task 32: PathPlanner - Use command in auto (no Java)
```

## What Changed

### Tasks 12-13: Spindexer Configuration (Now has a/b)
- **Task 12a**: Constants.java - Read Spindexer Constants → move to **Task 12b**
- **Task 12b**: SpindexerSubsystem.java - Configure the Motor → move to **Task 13a**
- **Task 13a**: Constants.java - Add Singulator Constants → move to **Task 13b**
- **Task 13b**: SpindexerSubsystem.java - Jam Detection → move to **Task 14**

### Tasks 14-17: Singulator (Task 17 now has a/b)
- **Task 14**: SingulatorSubsystem.java - Declare motor (unchanged)
- **Task 15**: SingulatorSubsystem.java - Add methods (unchanged)
- **Task 16**: RobotContainer.java - Combine spindexer+singulator → move to **Task 17a**
- **Task 17a**: SingulatorSubsystem.java - Add periodic logging → move to **Task 17b**
- **Task 17b**: RobotContainer.java - Release-to-trigger binding → move to **Task 18**

### Task 18: RobotContainer (Updated navigation)
- **Task 18**: RobotContainer.java - Wire Y button → move to **Task 19a**

### Tasks 19-21: RobotState + Bindings (Now has a/b)
- **Task 19a**: RobotState.java - Add IntakePosition enum → move to **Task 19b or 20a**
- **Task 19b**: RobotContainer.java - Wire B/X buttons → move to **Task 20a**
- **Task 20a**: RobotState.java - Add IntakeRollerState → move to **Task 20b or 21a**
- **Task 20b**: DriveWithJoysticks.java - Read file → move to **Task 21b**
- **Task 21a**: RobotState.java - Add SingulatorState → move to **Task 21b or 22a**
- **Task 21b**: DriveWithJoysticks.java - Change input curve → move to **Task 22a or 22b**

### Tasks 22-23: Intake + RobotContainer (Now has a/b)
- **Task 22a**: IntakeSubsystem.java - Declare motors → move to **Task 23a**
- **Task 22b**: RobotContainer.java - Precision mode → move to **Task 23b**
- **Task 23a**: IntakeSubsystem.java - Add methods → move to **Task 23b**
- **Task 23b**: RobotContainer.java - Method reference → move to **Task 24**

### Tasks 24-31: Advanced Features (Unchanged)
- **Task 24**: SnapToHeadingFixed.java - Read file
- **Task 25**: SnapToHeadingFixed.java - Tune PID
- **Task 26**: RobotContainer.java - Wire D-Pad
- **Task 27**: RobotContainer.java - Toggle mode
- **Task 28**: RobotContainer.java - Gate on vision
- **Task 29**: RobotContainer.java - Display vision status
- **Task 30**: RobotContainer.java - Aim at object
- **Task 31**: AutoCommands.java - Register named command

## Why This Change?

The original design had "dual tasks" where the same number appeared in two files. This was intentional (students do TWO things per task number), but it created confusion:
- Students saw "Task 17" twice and weren't sure which to do first
- Dependencies weren't clear (e.g., Task 18 needs IntakeSubsystem methods that aren't done yet)

The **a/b suffix** makes it explicit:
- **"a" tasks** are typically subsystem/core functionality
- **"b" tasks** are typically RobotContainer bindings or secondary files
- Navigation now clearly says "move to Task 19a" vs "move to Task 19b"

## For Students

When you see a task with an "a" suffix:
- Complete that task first
- You can then do the "b" task, OR continue with the next "a" task
- The instructions will guide you on the recommended path

Example flow:
```
Task 19a (RobotState) → Task 19b (RobotContainer) → Task 20a (DriveWithJoysticks)
              OR
Task 19a (RobotState) → Task 20a (RobotState) → Task 21a (RobotState) → Task 22a (IntakeSubsystem)
```

Both paths work! The "a" path keeps you focused on subsystems, the "b" path mixes in bindings.
