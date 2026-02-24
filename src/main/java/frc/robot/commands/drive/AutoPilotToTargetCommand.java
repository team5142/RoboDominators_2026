package frc.robot.commands.drive;

// TODO (next session with robot):
// 1. Remove the 3 unused constructor parameters (maxVelocity, maxAcceleration, maxJerk) -
//    they are always passed as 0, 0, 0 from SmartDriveToPosition and do nothing.
// 2. Confirm AutoPilot vendor library is tuned and atTarget() tolerance is tight enough
//    for use as a shooting position command.

import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.SmartLogger;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.MetersPerSecond;

// Precision navigation using the therekrab/autopilot vendor library.
// Called by SmartDriveToPosition for the final approach after PathPlanner gets close.
// Uses a singleton AutoPilot instance defined in Constants.AutoPilotConstants.
public class AutoPilotToTargetCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final PoseEstimatorSubsystem m_poseEstimator;
  private final Pose2d m_targetPose;

  private APTarget m_target;
  private int execCounter = 0;
  
  public AutoPilotToTargetCommand(
      Pose2d targetPose,
      DriveSubsystem driveSubsystem,
      PoseEstimatorSubsystem poseEstimator,
      double maxVelocity,      // Ignored - using singleton
      double maxAcceleration,  // Ignored - using singleton
      double maxJerk) {        // Ignored - using singleton
    
    m_targetPose = targetPose;
    m_driveSubsystem = driveSubsystem;
    m_poseEstimator = poseEstimator;
    
    addRequirements(driveSubsystem);
  }
  
  @Override
  public void initialize() {
    // FIXED: Just create target - AutoPilot instance is singleton
    m_target = new APTarget(m_targetPose);

    SmartLogger.logConsole("AutoPilot navigating to: " + SmartLogger.formatPose(m_targetPose), "AutoPilot");
    Logger.recordOutput("AutoPilot/TargetPose", m_targetPose);
    execCounter = 0;
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_poseEstimator.getEstimatedPose();
    var robotRelativeSpeeds = m_driveSubsystem.getRobotRelativeSpeeds();

    var result = Constants.AutoPilotConstants.PRECISION_AUTOPILOT.calculate(currentPose, robotRelativeSpeeds, m_target);

    double currentAngle = currentPose.getRotation().getRadians();
    double targetAngle = result.targetAngle().getRadians();

    // Shortest-arc angle wrap - keeps omega within [-pi, pi]
    double omega = MathUtil.angleModulus(targetAngle - currentAngle);

    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        result.vx().in(MetersPerSecond),
        result.vy().in(MetersPerSecond),
        omega,
        currentPose.getRotation()
    );

    m_driveSubsystem.driveRobotRelative(targetSpeeds);

    // Throttle to 10Hz - VX/VY/Omega are debug values, not needed every loop
    if (execCounter++ % 5 == 0) {
      Logger.recordOutput("AutoPilot/Active", true);
      Logger.recordOutput("AutoPilot/VX", result.vx().in(MetersPerSecond));
      Logger.recordOutput("AutoPilot/VY", result.vy().in(MetersPerSecond));
      Logger.recordOutput("AutoPilot/Omega", omega);
    }
  }
  
  @Override
  public boolean isFinished() {
    return Constants.AutoPilotConstants.PRECISION_AUTOPILOT.atTarget(m_poseEstimator.getEstimatedPose(), m_target);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    SmartLogger.logConsole("AutoPilot " + (interrupted ? "interrupted" : "complete"), "AutoPilot");
    Logger.recordOutput("AutoPilot/Active", false);
  }
}
