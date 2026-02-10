package frc.robot.subsystems.turret;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;

// Connects sensors and solver into an aim provider
public class TurretAimPipeline implements TurretAimProvider {
  private final Supplier<TurretAimInputs> inputsSupplier;
  private final TurretAimSolver solver;
  private final TurretAimInputs inputs = new TurretAimInputs();

  public TurretAimPipeline(Supplier<TurretAimInputs> inputsSupplier, TurretAimSolver solver) {
    this.inputsSupplier = inputsSupplier;
    this.solver = solver;
  }

  public TurretAimPipeline(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem,
      TurretAimSolver solver) {
    this(new TurretAimInputsFromPoseEstimator(poseEstimator, driveSubsystem), solver);
  }

  public TurretAimPipeline(
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem driveSubsystem,
      Supplier<Pose2d> targetPoseSupplier,
      TurretAimSolver solver) {
    this(new TurretAimInputsFromPoseEstimator(poseEstimator, driveSubsystem, targetPoseSupplier), solver);
  }

  @Override
  public boolean update(TurretAimGoal goal) {
    TurretAimInputs latest = inputsSupplier.get();
    if (latest == null) {
      goal.enable = false;
      return false;
    }

    inputs.robotPose = latest.robotPose;
    inputs.targetPose = latest.targetPose;
    inputs.robotSpeedMetersPerSecond = latest.robotSpeedMetersPerSecond;
    inputs.targetLatencySeconds = latest.targetLatencySeconds;

    solver.solve(inputs, goal);
    return goal.enable;
  }
}
