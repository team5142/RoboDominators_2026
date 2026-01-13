package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallisticsCalculator;
import org.littletonrobotics.junction.Logger;

// Continuously aims turret at hub (accounts for robot position and motion)
// Future enhancement: Add lead angle for shoot-on-move
public class TrackHubCommand extends Command {
  private final TurretSubsystem turret;
  private final ShooterSubsystem shooter;
  private final PoseEstimatorSubsystem poseEstimator;
  private final DriveSubsystem drive;
  
  public TrackHubCommand(
      TurretSubsystem turret, 
      ShooterSubsystem shooter,
      PoseEstimatorSubsystem poseEstimator,
      DriveSubsystem drive) {
    
    this.turret = turret;
    this.shooter = shooter;
    this.poseEstimator = poseEstimator;
    this.drive = drive;
    
    addRequirements(turret); // Command controls turret (not shooter - allows manual RPM control)
  }
  
  @Override
  public void execute() {
    Pose2d robotPose = poseEstimator.getEstimatedPose();
    Pose2d hubPose = shooter.getAllianceHubPublic(); // Need to make getAllianceHub() public or add getter
    
    // Calculate angle from robot to hub
    Translation2d robotToHub = hubPose.getTranslation().minus(robotPose.getTranslation());
    double angleToHubRad = Math.atan2(robotToHub.getY(), robotToHub.getX());
    
    // Convert to turret-relative angle (subtract robot heading)
    double robotHeadingRad = robotPose.getRotation().getRadians();
    double turretAngleRad = angleToHubRad - robotHeadingRad;
    
    // TODO: Add lead angle calculation when implementing shoot-on-move
    // double leadAngle = BallisticsCalculator.calculateTurretLeadAngle(
    //     drive.getRobotRelativeSpeeds(), 
    //     robotToHub.getNorm(), 
    //     estimatedBallVelocity);
    // turretAngleRad += leadAngle;
    
    // Command turret to aim
    turret.setTargetAngle(new Rotation2d(turretAngleRad));
    
    // Log tracking data
    Logger.recordOutput("TrackHub/RobotPose", robotPose);
    Logger.recordOutput("TrackHub/HubPose", hubPose);
    Logger.recordOutput("TrackHub/TurretAngleDeg", Math.toDegrees(turretAngleRad));
    Logger.recordOutput("TrackHub/DistanceMeters", robotToHub.getNorm());
  }
  
  @Override
  public boolean isFinished() {
    return false; // Run continuously until interrupted
  }
  
  @Override
  public void end(boolean interrupted) {
    // Don't stop turret - leave it at last commanded angle
    Logger.recordOutput("TrackHub/Active", false);
  }
}
