package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;

// Inputs for turret aim calculation
public class TurretAimInputs {
  public Pose2d robotPose = new Pose2d();
  public Pose2d targetPose = new Pose2d();
  public double robotSpeedMetersPerSecond = 0.0;
  public double robotOmegaRadPerSecond = 0.0;  // rotation rate — used to prevent latch freezing during in-place rotation
  // Field-relative velocity — used for Phase 4 lead compensation.
  public double robotFieldVxMetersPerSecond = 0.0;
  public double robotFieldVyMetersPerSecond = 0.0;
  public double targetLatencySeconds = 0.0;
}
