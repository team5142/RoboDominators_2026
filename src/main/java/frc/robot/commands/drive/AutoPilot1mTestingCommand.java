package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.SmartLogger;
import static edu.wpi.first.units.Units.MetersPerSecond;

// D-pad 1m test - validates AutoPilot accuracy/tuning (KEEP for pre-competition validation)
// Hold D-pad direction to move 1m, release to stop - measures final accuracy
public class AutoPilot1mTestingCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final PoseEstimatorSubsystem m_poseEstimator;
  private final String m_direction;
  private com.therekrab.autopilot.APTarget m_target;
  private int m_executionCount = 0;
  
  public AutoPilot1mTestingCommand(String direction, DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimator) {
    m_direction = direction;
    m_driveSubsystem = driveSubsystem;
    m_poseEstimator = poseEstimator;
    addRequirements(driveSubsystem);
  }
  
  @Override
  public void initialize() {
    Pose2d currentPose = m_poseEstimator.getEstimatedPose();
    Pose2d targetPose = calculateTargetPose(currentPose, m_direction);
    m_target = new com.therekrab.autopilot.APTarget(targetPose);
    m_executionCount = 0;
    
    String initMsg = m_direction + " test: " + formatPose(currentPose) + " → " + formatPose(targetPose);
    SmartLogger.logConsole(initMsg, "AutoPilot 1m Test");
    SmartLogger.logReplay("AutoPilotTest/StartPose", currentPose);
    SmartLogger.logReplay("AutoPilotTest/TargetPose", targetPose);
  }
  
  @Override
  public void execute() {
    m_executionCount++;
    Pose2d currentPose = m_poseEstimator.getEstimatedPose();
    var robotRelativeSpeeds = m_driveSubsystem.getRobotRelativeSpeeds();
    
    // Use singleton TEST_AUTOPILOT (5cm/2° tolerances)
    var result = Constants.AutoPilotConstants.TEST_AUTOPILOT.calculate(currentPose, robotRelativeSpeeds, m_target);
    
    // Calculate rotation command
    double currentAngle = currentPose.getRotation().getRadians();
    double targetAngle = result.targetAngle().getRadians();
    double omega = targetAngle - currentAngle;
    while (omega > Math.PI) omega -= 2 * Math.PI;
    while (omega < -Math.PI) omega += 2 * Math.PI;
    
    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        result.vx().in(MetersPerSecond), result.vy().in(MetersPerSecond), omega, currentPose.getRotation());
    
    m_driveSubsystem.driveRobotRelative(targetSpeeds);
    
    // Console progress every 10 loops (~200ms) to reduce spam
    double distanceRemaining = currentPose.getTranslation().getDistance(m_target.getReference().getTranslation());
    if (m_executionCount % 10 == 0) {
      SmartLogger.logConsole(String.format("[%3d] %.3fm | VX:%+.3f VY:%+.3f W:%+.3f",
          m_executionCount, distanceRemaining, result.vx().in(MetersPerSecond), result.vy().in(MetersPerSecond), omega));
    }
    
    // Essential replay logging (every loop for velocity graphs)
    SmartLogger.logReplay("AutoPilot/DistanceRemaining", distanceRemaining);
    SmartLogger.logReplay("AutoPilot/VX", result.vx().in(MetersPerSecond));
    SmartLogger.logReplay("AutoPilot/VY", result.vy().in(MetersPerSecond));
    SmartLogger.logReplay("AutoPilot/Omega", omega);
  }
  
  @Override
  public boolean isFinished() {
    return Constants.AutoPilotConstants.TEST_AUTOPILOT.atTarget(m_poseEstimator.getEstimatedPose(), m_target);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    
    Pose2d finalPose = m_poseEstimator.getEstimatedPose();
    double finalDistance = finalPose.getTranslation().getDistance(m_target.getReference().getTranslation());
    
    String result = String.format("%s: %d loops, %.1fcm error",
        interrupted ? "INTERRUPTED" : "SUCCESS", m_executionCount, finalDistance * 100);
    SmartLogger.logConsole(result, "Test Complete");
    
    SmartLogger.logReplay("AutoPilotTest/FinalPose", finalPose);
    SmartLogger.logReplay("AutoPilotTest/FinalDistance", finalDistance);
  }
  
  // Calculate 1m target in requested direction (maintains rotation toward target)
  private static Pose2d calculateTargetPose(Pose2d current, String direction) {
    switch (direction.toLowerCase()) {
      case "forward":  return new Pose2d(current.getX() + 1.0, current.getY(), edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0.0));
      case "backward": return new Pose2d(current.getX() - 1.0, current.getY(), edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180.0));
      case "left":     return new Pose2d(current.getX(), current.getY() + 1.0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90.0));
      case "right":    return new Pose2d(current.getX(), current.getY() - 1.0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-90.0));
      default: throw new IllegalArgumentException("Invalid direction: " + direction);
    }
  }
  
  private static String formatPose(Pose2d pose) {
    return String.format("(%.2f,%.2f,%.0f°)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }
}
