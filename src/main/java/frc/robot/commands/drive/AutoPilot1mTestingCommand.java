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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * AutoPilot 1m test command - proper Command class matching AutoPilot examples
 */
public class AutoPilot1mTestingCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final PoseEstimatorSubsystem m_poseEstimator;
  private final String m_direction;
  
  private APTarget m_target;
  private int m_executionCount = 0; // Track loop iterations
  
  // Test constraints
  private static final double TEST_MAX_VELOCITY = 1.5; // m/s
  private static final double TEST_MAX_ACCELERATION = 1.0; // m/s²
  private static final double TEST_MAX_JERK = 10.0; // m/s³
  private static final double ERROR_XY_METERS = 0.03; // 3cm
  private static final double ERROR_THETA_DEGREES = 1.0; // 1°
  
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
    
    // Create AutoPilot instance
    APConstraints constraints = new APConstraints(TEST_MAX_VELOCITY, TEST_MAX_ACCELERATION, TEST_MAX_JERK);
    APProfile profile = new APProfile(constraints)
        .withErrorXY(Meters.of(ERROR_XY_METERS))
        .withErrorTheta(Degrees.of(ERROR_THETA_DEGREES));
    
    m_target = new APTarget(targetPose);
    
    m_executionCount = 0; // Reset counter
    
    System.out.println("========== AUTOPILOT 1M TEST ==========");
    System.out.println("Direction: " + m_direction);
    System.out.println("Current: " + formatPose(currentPose));
    System.out.println("Target:  " + formatPose(targetPose));
    System.out.println("Distance: " + formatDistance(currentPose, targetPose));
    System.out.println("========================================");
    
    Logger.recordOutput("AutoPilotTest/StartPose", currentPose);
    Logger.recordOutput("AutoPilotTest/TargetPose", targetPose);
  }
  
  @Override
  public void execute() {
    m_executionCount++; // Increment counter
    
    Pose2d currentPose = m_poseEstimator.getEstimatedPose();
    var robotRelativeSpeeds = m_driveSubsystem.getRobotRelativeSpeeds();
    
    // Calculate AutoPilot output
    var result = Constants.AutoPilotConstants.TEST_AUTOPILOT.calculate(currentPose, robotRelativeSpeeds, m_target);
    
    // Apply velocities directly (matching the example!)
    double currentAngle = currentPose.getRotation().getRadians();
    double targetAngle = result.targetAngle().getRadians();
    double omega = targetAngle - currentAngle;
    
    // Normalize omega to [-π, π]
    while (omega > Math.PI) omega -= 2 * Math.PI;
    while (omega < -Math.PI) omega += 2 * Math.PI;
    
    // Convert to field-relative ChassisSpeeds
    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        result.vx().in(MetersPerSecond),
        result.vy().in(MetersPerSecond),
        omega,
        currentPose.getRotation()
    );
    
    // Console trace every loop
    double distanceRemaining = currentPose.getTranslation().getDistance(m_target.getReference().getTranslation());
    boolean atTarget = Constants.AutoPilotConstants.TEST_AUTOPILOT.atTarget(currentPose, m_target);
    
    System.out.printf("[Loop %3d] Dist: %.3fm | VX: %+.3f | VY: %+.3f | Omega: %+.3f | AtTarget: %s%n",
        m_executionCount,
        distanceRemaining,
        result.vx().in(MetersPerSecond),
        result.vy().in(MetersPerSecond),
        omega,
        atTarget ? "YES" : "NO");
    
    // Apply the speeds
    m_driveSubsystem.driveRobotRelative(targetSpeeds);
    
    // Log for debugging
    Logger.recordOutput("AutoPilot/ExecutionCount", m_executionCount);
    Logger.recordOutput("AutoPilot/DistanceRemaining", distanceRemaining);
    Logger.recordOutput("AutoPilot/VX", result.vx().in(MetersPerSecond));
    Logger.recordOutput("AutoPilot/VY", result.vy().in(MetersPerSecond));
    Logger.recordOutput("AutoPilot/Omega", omega);
    Logger.recordOutput("AutoPilot/TargetAngle", result.targetAngle().getDegrees());
    Logger.recordOutput("AutoPilot/AtTarget", atTarget);
  }
  
  @Override
  public boolean isFinished() {
    // Check if at target - runs after each execute()
    return Constants.AutoPilotConstants.TEST_AUTOPILOT.atTarget(m_poseEstimator.getEstimatedPose(), m_target);
  }
  
  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    
    Pose2d finalPose = m_poseEstimator.getEstimatedPose();
    double finalDistance = finalPose.getTranslation().getDistance(m_target.getReference().getTranslation());
    
    System.out.println("========== TEST COMPLETE ==========");
    System.out.println("Loops executed: " + m_executionCount);
    System.out.println("Final pose: " + formatPose(finalPose));
    System.out.println("Final distance: " + String.format("%.3fm", finalDistance));
    System.out.println(interrupted ? "INTERRUPTED" : "SUCCESS");
    System.out.println("===================================");
    
    Logger.recordOutput("AutoPilotTest/FinalPose", finalPose);
    Logger.recordOutput("AutoPilotTest/FinalDistance", finalDistance);
  }
  
  private static Pose2d calculateTargetPose(Pose2d current, String direction) {
    switch (direction.toLowerCase()) {
      case "forward":  // Face 0° (downfield)
        return new Pose2d(
            current.getX() + 1.0, 
            current.getY(), 
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0.0));
      
      case "backward": // Face 180° (back to alliance wall)
        return new Pose2d(
            current.getX() - 1.0, 
            current.getY(), 
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180.0));
      
      case "left":     // Face 90° (left side)
        return new Pose2d(
            current.getX(), 
            current.getY() + 1.0, 
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90.0));
      
      case "right":    // Face -90° or 270° (right side)
        return new Pose2d(
            current.getX(), 
            current.getY() - 1.0, 
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-90.0));
      
      default: throw new IllegalArgumentException("Invalid direction: " + direction);
    }
  }
  
  private static String formatPose(Pose2d pose) {
    return String.format("(%.2fm, %.2fm, %.1f°)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }
  
  private static String formatDistance(Pose2d from, Pose2d to) {
    double distance = from.getTranslation().getDistance(to.getTranslation());
    return String.format("%.3fm", distance);
  }
}
