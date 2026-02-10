package frc.robot.subsystems;

import frc.robot.RobotState;
import frc.robot.subsystems.turret.TurretIOCTRE;

// Wrapper for legacy imports
@Deprecated
public class TurretSubsystem extends frc.robot.subsystems.turret.TurretSubsystem {
  public TurretSubsystem(RobotState robotState) {
    super(robotState, new TurretIOCTRE());
  }
}
