package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

// Calculates required shooter parameters (RPM, turret angle) for moving shots
// Uses 2D projectile motion with robot velocity compensation
// Valid range: 2.1 to 13.0 feet (shooting zone geometry)
public class BallisticsCalculator {
  
  // Physics constants
  private static final double GRAVITY_MPS2 = 9.81; // Earth gravity
  private static final double BALL_MASS_KG = 0.227; // 0.5 lbs game piece
  private static final double AIR_RESISTANCE_COEFF = 0.0; // Ignore air resistance for now
  
  // Shooter geometry (from Constants)
  private static final double SHOOTER_HEIGHT_METERS = Units.inchesToMeters(Constants.Shooter.SHOOTER_HEIGHT_INCHES);
  private static final double SHOOTER_ANGLE_RAD = Math.toRadians(Constants.Shooter.SHOOTER_ANGLE_DEG); // 45 degrees
  private static final double HUB_HEIGHT_METERS = Units.inchesToMeters(Constants.StartingPositions.HUB_HEIGHT_INCHES); // 6ft
  
  // Flywheel conversion (6" diameter wheels)
  private static final double FLYWHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(Constants.Shooter.FLYWHEEL_DIAMETER_INCHES) * Math.PI;
  
  // Calculate required flywheel RPM for stationary shot at given distance
  public static double calculateStationaryRPM(double distanceMeters) {
    // Height difference (hub is higher than shooter)
    double deltaHeight = HUB_HEIGHT_METERS - SHOOTER_HEIGHT_METERS;
    
    // Projectile motion equations for 45 degree fixed angle
    // v^2 = (g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - deltaHeight))
    double cosTheta = Math.cos(SHOOTER_ANGLE_RAD);
    double tanTheta = Math.tan(SHOOTER_ANGLE_RAD);
    
    // Required exit velocity (m/s)
    double denominator = 2 * cosTheta * cosTheta * (distanceMeters * tanTheta - deltaHeight);
    if (denominator <= 0) {
      return 0.0; // Can't reach target with this angle
    }
    
    double exitVelocitySquared = (GRAVITY_MPS2 * distanceMeters * distanceMeters) / denominator;
    double exitVelocityMPS = Math.sqrt(Math.max(0, exitVelocitySquared));
    
    // Convert exit velocity to RPM (surface speed of flywheels)
    double rps = exitVelocityMPS / FLYWHEEL_CIRCUMFERENCE_METERS; // Rotations per second
    double rpm = rps * 60.0; // Convert to RPM
    
    // Apply safety clamps
    return Math.max(2000, Math.min(6000, rpm)); // Clamp to motor safe range
  }
  
  // Calculate RPM accounting for robot forward/backward motion (adds/subtracts from ball velocity)
  public static double calculateMovingRPM(double distanceMeters, ChassisSpeeds robotVelocity, double hubAngleRad) {
    // Get base RPM for stationary shot
    double baseRPM = calculateStationaryRPM(distanceMeters);
    
    // Robot velocity component in direction of target (positive = toward hub)
    double forwardVelocityMPS = 
        robotVelocity.vxMetersPerSecond * Math.cos(hubAngleRad) + 
        robotVelocity.vyMetersPerSecond * Math.sin(hubAngleRad);
    
    // Convert velocity to RPM adjustment (forward motion = need less RPM)
    double velocityAdjustmentRPS = -forwardVelocityMPS / FLYWHEEL_CIRCUMFERENCE_METERS;
    double velocityAdjustmentRPM = velocityAdjustmentRPS * 60.0;
    
    // Apply adjustment and clamp
    double adjustedRPM = baseRPM + velocityAdjustmentRPM;
    return Math.max(2000, Math.min(6000, adjustedRPM));
  }
  
  // Calculate flight time for ballistics (needed for motion prediction)
  public static double calculateFlightTime(double distanceMeters, double exitVelocityMPS) {
    // Time of flight = distance / (velocity * cos(angle))
    double flightTime = distanceMeters / (exitVelocityMPS * Math.cos(SHOOTER_ANGLE_RAD));
    return Math.max(0, Math.min(2.0, flightTime)); // Clamp 0-2 seconds
  }
  
  // Predict where robot will be after flight time (for lead angle calculation)
  public static Pose2d predictFutureRobotPose(Pose2d currentPose, ChassisSpeeds velocity, double flightTimeSeconds) {
    // Simple linear prediction (assumes constant velocity)
    double futureX = currentPose.getX() + velocity.vxMetersPerSecond * flightTimeSeconds;
    double futureY = currentPose.getY() + velocity.vyMetersPerSecond * flightTimeSeconds;
    double futureTheta = currentPose.getRotation().getRadians() + velocity.omegaRadiansPerSecond * flightTimeSeconds;
    
    return new Pose2d(futureX, futureY, new edu.wpi.first.math.geometry.Rotation2d(futureTheta));
  }
  
  // Calculate turret lead angle for lateral robot motion
  public static double calculateTurretLeadAngle(ChassisSpeeds robotVelocity, double distanceToHub, double ballVelocityMPS) {
    // Lateral velocity perpendicular to shot (positive = robot moving left relative to hub)
    // Lead angle compensates for robot motion during flight
    
    // Simplified: lead angle = arctan(lateral_velocity * flight_time / distance)
    double flightTime = calculateFlightTime(distanceToHub, ballVelocityMPS);
    double lateralDistance = Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond) * flightTime;
    
    // Lead angle in radians (positive = aim ahead of target)
    return Math.atan2(lateralDistance, distanceToHub);
  }
  
  // Validate shot is physically possible within shooting zone constraints
  public static boolean isShotValid(double distanceMeters, double requiredRPM) {
    // Check distance range (2.12 ft min from zone boundary, 13.0 ft max at alliance wall)
    boolean inRange = distanceMeters >= Units.feetToMeters(Constants.Shooter.MIN_SHOT_DISTANCE_FEET) &&
                      distanceMeters <= Units.feetToMeters(Constants.Shooter.MAX_SHOT_DISTANCE_FEET);
    
    // Check RPM is achievable
    boolean rpmAchievable = requiredRPM >= 2000 && requiredRPM <= 6000;
    
    return inRange && rpmAchievable;
  }
}
