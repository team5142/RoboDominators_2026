package frc.robot.commands.drive;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * AutoPilot command for precision navigation - uses singleton AutoPilot
 */
public class AutoPilotToTargetCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final PoseEstimatorSubsystem m_poseEstimator;
  private final Pose2d m_targetPose;
  
  private APTarget m_target;
  
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
    
    System.out.println("[AutoPilot] Navigate to: " + formatPose(m_targetPose));
    Logger.recordOutput("AutoPilot/TargetPose", m_targetPose);
  }
  
  @Override
  public void execute() {
    Pose2d currentPose = m_poseEstimator.getEstimatedPose();
    var robotRelativeSpeeds = m_driveSubsystem.getRobotRelativeSpeeds();
    
    // Use singleton AutoPilot instance
    var result = Constants.AutoPilotConstants.PRECISION_AUTOPILOT.calculate(currentPose, robotRelativeSpeeds, m_target);
    
    // Calculate omega
    double currentAngle = currentPose.getRotation().getRadians();
    double targetAngle = result.targetAngle().getRadians();
    double omega = targetAngle - currentAngle;
    
    // Normalize omega
    while (omega > Math.PI) omega -= 2 * Math.PI;
    while (omega < -Math.PI) omega += 2 * Math.PI;
    
    // Convert to field-relative
    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        result.vx().in(MetersPerSecond),
        result.vy().in(MetersPerSecond),
        omega,
        currentPose.getRotation()
    );
    
    // Apply speeds
    m_driveSubsystem.driveRobotRelative(targetSpeeds);
    
    // Log
    Logger.recordOutput("AutoPilot/Active", true);
    Logger.recordOutput("AutoPilot/VX", result.vx().in(MetersPerSecond));
    Logger.recordOutput("AutoPilot/VY", result.vy().in(MetersPerSecond));
    Logger.recordOutput("AutoPilot/Omega", omega);
  }
  
  @Override
  public boolean isFinished() {
    return Constants.AutoPilotConstants.PRECISION_AUTOPILOT.atTarget(m_poseEstimator.getEstimatedPose(), m_target);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    System.out.println("[AutoPilot] " + (interrupted ? "INTERRUPTED" : "Complete"));
    Logger.recordOutput("AutoPilot/Active", false);
  }
  
  private static String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }
}
