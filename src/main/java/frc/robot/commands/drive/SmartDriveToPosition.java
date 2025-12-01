package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Smart two-phase navigation to a target pose with vision-guided precision.
 * 
 * PHASE 1: Fast curved approach to vision acquisition point (~60% of path)
 *   - Robot rotates to final heading
 *   - Arcs smoothly to 4ft away from target
 *   - Gets good AprilTag visibility before final approach
 * 
 * PHASE 2: Slow straight precision to target (~40% of path)
 *   - Robot drives straight toward target
 *   - Vision dominates pose corrections
 *   - Smooth final alignment
 * 
 * FEATURES:
 *   - Driver override detection (joystick cancels path)
 *   - Stuck detection (no movement for 2s → abort)
 *   - Path feasibility checks (too far, out of field)
 *   - Obstacle avoidance (reef, field edges)
 *   - Vision loss fallback (multi → single → odometry)
 *   - Complete logging for post-match analysis
 */
public class SmartDriveToPosition {
  
  // Vision acquisition distance (how far from target to switch to precision)
  private static final double IDEAL_VISION_DISTANCE_METERS = .75; // CHANGED: ~3.3 feet (was 1.2m / 4ft)
  
  // Speed multipliers for each phase
  private static final double FAST_SPEED_MULT = 0.40;      // 40% speed for approach
  private static final double PRECISION_SPEED_MULT = 0.2;  // 20% speed for precision
  
  // Field boundaries (2025 Reefscape: 54ft x 27ft)
  private static final double FIELD_LENGTH_METERS = Units.feetToMeters(54.0);
  private static final double FIELD_WIDTH_METERS = Units.feetToMeters(27.0);
  private static final double FIELD_MARGIN_METERS = 0.3; // Stay 1ft from edges
  
  // Obstacle zones (reef, barge) - TODO: Update with actual 2026 game pieces
  private static final Translation2d REEF_CENTER = new Translation2d(4.0, 4.0);
  private static final double REEF_RADIUS_METERS = 1.5;
  
  // Path validation limits
  private static final double MAX_PATH_DISTANCE_METERS = 10.0; // Reject paths >33ft
  
  /**
   * Create smart navigation to target pose
   * 
   * @param target Target pose on field
   * @param poseEstimator Pose estimator for current position
   * @param tagVision Vision subsystem for quality checks
   * @param robotState Robot state for phase tracking
   * @param driverController Controller for override detection
   */
  public static Command create(
      Pose2d target,
      PoseEstimatorSubsystem poseEstimator,
      TagVisionSubsystem tagVision,
      RobotState robotState,
      XboxController driverController,
      DriveSubsystem driveSubsystem,
      GyroSubsystem gyroSubsystem) { // ADD THIS PARAMETER
    
    Pose2d start = poseEstimator.getEstimatedPose();
    double distanceToTarget = start.getTranslation().getDistance(target.getTranslation());
    
    // ===== PRE-FLIGHT CHECKS =====
    
    // CHECK 1: Path too far (likely error in code)
    if (distanceToTarget > MAX_PATH_DISTANCE_METERS) {
      DriverStation.reportError(
          String.format("Target too far: %.1fm (max: %.1fm)", 
              distanceToTarget, MAX_PATH_DISTANCE_METERS), 
          false);
      
      return new SequentialCommandGroup(
          Commands.print("ERROR: Target unreachable - too far"),
          Commands.runOnce(() -> {
            SmartDashboard.putString("SmartDrive/Status", "ERROR: Too far");
            robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
          })
      );
    }
    
    // CHECK 2: Target outside field
    if (!isWithinField(target.getTranslation())) {
      DriverStation.reportError("Target outside field boundaries", false);
      
      return new SequentialCommandGroup(
          Commands.print("ERROR: Target outside field"),
          Commands.runOnce(() -> {
            SmartDashboard.putString("SmartDrive/Status", "ERROR: Out of bounds");
            robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
          })
      );
    }
    
    // ===== PHASE SELECTION =====
    
    // CASE 1: Already close (< 4ft) - skip vision point, go straight
    if (distanceToTarget < IDEAL_VISION_DISTANCE_METERS) {
      System.out.println("=== SMART DRIVE: CLOSE RANGE ===");
      System.out.println("Distance: " + String.format("%.2fm", distanceToTarget));
      System.out.println("Using: Direct precision approach");
      
      return new SequentialCommandGroup(
          Commands.runOnce(() -> {
            robotState.setNavigationPhase(RobotState.NavigationPhase.PRECISION);
            SmartDashboard.putString("SmartDrive/Status", "Direct precision");
            Logger.recordOutput("SmartDrive/Mode", "DirectPrecision");
          }),
          
          new VisionGuidedStraightApproach(
              target,
              poseEstimator,
              tagVision,
              robotState,
              driverController,
              driveSubsystem,
              gyroSubsystem) // ADD THIS
      ).finallyDo((interrupted) -> {
        robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
        SmartDashboard.putString("SmartDrive/Status", interrupted ? "Interrupted" : "Complete");
        Logger.recordOutput("SmartDrive/Interrupted", interrupted);
        Logger.recordOutput("SmartDrive/Complete", !interrupted);
      });
    }
    
    // CASE 2: Far away - use two-phase approach
    Pose2d visionPoint = calculateSafeVisionPoint(start, target);
    
    if (visionPoint == null) {
      // Vision point invalid (obstacle/field edge) - use direct slow path
      System.out.println("=== SMART DRIVE: DIRECT PATH (No Vision Point) ===");
      System.out.println("Reason: Vision point blocked/invalid");
      
      return new SequentialCommandGroup(
          Commands.runOnce(() -> {
            robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
            SmartDashboard.putString("SmartDrive/Status", "Direct slow path");
            Logger.recordOutput("SmartDrive/Mode", "DirectSlow");
          }),
          
          AutoBuilder.pathfindToPose(target, createPrecisionConstraints())
      ).finallyDo((interrupted) -> {
        robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
        SmartDashboard.putString("SmartDrive/Status", interrupted ? "Interrupted" : "Complete");
        Logger.recordOutput("SmartDrive/Interrupted", interrupted);
        Logger.recordOutput("SmartDrive/Complete", !interrupted);
      });
    }
    
    // CASE 3: Normal two-phase approach
    System.out.println("=== SMART DRIVE: TWO-PHASE APPROACH ===");
    System.out.println("Total distance: " + String.format("%.2fm", distanceToTarget));
    System.out.println("Vision point: " + String.format("%.2fm from target", IDEAL_VISION_DISTANCE_METERS));
    System.out.println("Phase 1: Fast hook to vision point");
    System.out.println("Phase 2: Precision straight to target");
    
    return new SequentialCommandGroup(
        // PHASE 1: Fast hook to vision acquisition point
        Commands.runOnce(() -> {
          robotState.setNavigationPhase(RobotState.NavigationPhase.FAST_APPROACH);
          SmartDashboard.putString("SmartDrive/Status", "Hook to vision point");
          SmartDashboard.putNumber("SmartDrive/Progress", 0.0);
          Logger.recordOutput("SmartDrive/Mode", "TwoPhase");
          Logger.recordOutput("SmartDrive/Phase", "FastApproach");
          Logger.recordOutput("SmartDrive/VisionPoint", visionPoint);
        }),
        
        AutoBuilder.pathfindToPose(visionPoint, createFastConstraints())
            .withTimeout(5.0), // Safety timeout for phase 1
        
        // PHASE 2: Slow precision straight to target
        Commands.runOnce(() -> {
          robotState.setNavigationPhase(RobotState.NavigationPhase.PRECISION);
          SmartDashboard.putString("SmartDrive/Status", "Precision approach");
          SmartDashboard.putNumber("SmartDrive/Progress", 0.6);
          Logger.recordOutput("SmartDrive/Phase", "Precision");
        }),
        
        new VisionGuidedStraightApproach(
            target,
            poseEstimator,
            tagVision,
            robotState,
            driverController, 
            driveSubsystem,
            gyroSubsystem) // ADD THIS
            .withTimeout(3.0) // Safety timeout for phase 2
    ).finallyDo((interrupted) -> {
      robotState.setNavigationPhase(RobotState.NavigationPhase.NONE);
      SmartDashboard.putString("SmartDrive/Status", interrupted ? "Interrupted" : "Complete");
      Logger.recordOutput("SmartDrive/Interrupted", interrupted);
      Logger.recordOutput("SmartDrive/Complete", !interrupted);
    });
  }

  /**
   * Calculate safe vision acquisition point (1.5m BEHIND target, facing target)
   * "Behind" = opposite direction of where target faces
   * This ensures robot approaches from direction where it can see AprilTags
   * 
   * Example: If target faces down (-90°), vision point is 1.5m ABOVE target
   * 
   * Returns null if no valid point can be found
   */
  private static Pose2d calculateSafeVisionPoint(Pose2d start, Pose2d target) {
    // Calculate point 1.5m BEHIND target (opposite of target's heading)
    Rotation2d targetHeading = target.getRotation();
    Rotation2d approachDirection = targetHeading.plus(Rotation2d.fromDegrees(180.0)); // Flip 180°
    
    // Vision point = target + 1.5m in OPPOSITE direction of target heading
    double visionX = target.getX() + (IDEAL_VISION_DISTANCE_METERS * approachDirection.getCos());
    double visionY = target.getY() + (IDEAL_VISION_DISTANCE_METERS * approachDirection.getSin());
    Translation2d visionLocation = new Translation2d(visionX, visionY);
    
    Logger.recordOutput("SmartDrive/Debug/TargetHeading", targetHeading.getDegrees());
    Logger.recordOutput("SmartDrive/Debug/ApproachDirection", approachDirection.getDegrees());
    Logger.recordOutput("SmartDrive/Debug/VisionPointRaw", visionLocation);
    
    // VALIDATION 1: Check if within field
    if (!isWithinField(visionLocation)) {
      Logger.recordOutput("SmartDrive/VisionPoint/OutOfField", visionLocation);
      
      // Try moving vision point closer to target until it's in field
      for (double distance = IDEAL_VISION_DISTANCE_METERS - 0.2; distance > 0.5; distance -= 0.2) {
        double adjustedX = target.getX() + (distance * approachDirection.getCos());
        double adjustedY = target.getY() + (distance * approachDirection.getSin());
        Translation2d adjustedLocation = new Translation2d(adjustedX, adjustedY);
        
        if (isWithinField(adjustedLocation)) {
          visionLocation = adjustedLocation;
          Logger.recordOutput("SmartDrive/VisionPoint/AdjustedDistance", distance);
          Logger.recordOutput("SmartDrive/VisionPoint/Adjusted", visionLocation);
          break;
        }
      }
      
      // If still outside field after adjustment, give up
      if (!isWithinField(visionLocation)) {
        Logger.recordOutput("SmartDrive/VisionPointRejected", "Can't fit in field");
        return null;
      }
    }
    
    // VALIDATION 2: Check if inside obstacle
    if (isInsideObstacle(visionLocation)) {
      Logger.recordOutput("SmartDrive/VisionPoint/InsideObstacle", visionLocation);
      
      // Try moving around obstacle
      visionLocation = findNearestValidPoint(visionLocation, target.getTranslation());
      
      if (visionLocation == null) {
        Logger.recordOutput("SmartDrive/VisionPointRejected", "Blocked by obstacle");
        return null;
      }
    }
    
    // Vision point is valid - robot should face target at this point
    Pose2d visionPoint = new Pose2d(visionLocation, targetHeading); // Face same direction as target
    
    Logger.recordOutput("SmartDrive/VisionPoint/Final", visionPoint);
    Logger.recordOutput("SmartDrive/VisionPoint/DistanceFromTarget", 
        visionLocation.getDistance(target.getTranslation()));
    
    return visionPoint;
  }
  
  // ===== FIELD VALIDATION HELPERS =====
  
  private static boolean isWithinField(Translation2d point) {
    return point.getX() > FIELD_MARGIN_METERS &&
           point.getX() < FIELD_LENGTH_METERS - FIELD_MARGIN_METERS &&
           point.getY() > FIELD_MARGIN_METERS &&
           point.getY() < FIELD_WIDTH_METERS - FIELD_MARGIN_METERS;
  }
  
  private static Translation2d clampToField(Translation2d point) {
    double clampedX = Math.max(FIELD_MARGIN_METERS,
                               Math.min(FIELD_LENGTH_METERS - FIELD_MARGIN_METERS, point.getX()));
    double clampedY = Math.max(FIELD_MARGIN_METERS,
                               Math.min(FIELD_WIDTH_METERS - FIELD_MARGIN_METERS, point.getY()));
    return new Translation2d(clampedX, clampedY);
  }
  
  private static boolean isInsideObstacle(Translation2d point) {
    // Check reef (center keep-out zone)
    double distToReef = point.getDistance(REEF_CENTER);
    if (distToReef < REEF_RADIUS_METERS) {
      return true;
    }
    
    // TODO: Add other obstacles (barge, etc.) when 2026 game is released
    
    return false;
  }
  
  private static Translation2d findNearestValidPoint(Translation2d invalidPoint, Translation2d target) {
    // Move point radially away from obstacle center
    Translation2d fromReef = invalidPoint.minus(REEF_CENTER);
    double distance = fromReef.getNorm();
    
    if (distance < 0.1) {
      // Point is exactly at obstacle center - can't find direction
      return null;
    }
    
    Translation2d direction = fromReef.div(distance);
    Translation2d validPoint = REEF_CENTER.plus(direction.times(REEF_RADIUS_METERS + 0.3));
    
    // Verify new point is still valid and within field
    if (!isWithinField(validPoint)) {
      return null;
    }
    
    return validPoint;
  }
  
  // ===== PATH CONSTRAINTS =====
  
  private static PathConstraints createFastConstraints() {
    return new PathConstraints(
        Constants.Swerve.MAX_TRANSLATION_SPEED_MPS * FAST_SPEED_MULT, // 40% speed
        4.0, // Max accel
        Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SEC,
        Math.PI // Max angular accel
    );
  }
  
  private static PathConstraints createPrecisionConstraints() {
    return new PathConstraints(
        Constants.Swerve.MAX_TRANSLATION_SPEED_MPS * PRECISION_SPEED_MULT, // 20% speed
        2.0, // Max accel (gentler)
        Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SEC * 0.5,
        Math.PI / 2 // Max angular accel
    );
  }
}
