package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.SmartLogger;
import java.util.function.Supplier;

// Selects hub or pass targets based on robot pose.
// Sets RobotState.shotSuppressed when the robot is in a no-shoot zone:
//   - Hub bump zone: 73in x 47in rectangle centered on each hub
//   - Hub net zone: 4ft deep x 47in wide strip on the neutral-zone side of each hub
//   - Tower shadow zone: 44in x 47in against each driver wall (behind the alliance tower)
// AutoShootCommand reads shotSuppressed to block firing; turret keeps tracking.
public class TurretTargetSelector implements Supplier<Pose2d> {
  private final PoseEstimatorSubsystem poseEstimator;
  private final RobotState robotState;

  private static final double ZONE_HYSTERESIS_METERS = 0.3;
  // Bump zone: 73in x 47in centered on hub
  private static final double BUMP_HALF_X = Units.inchesToMeters(73.0 / 2.0);
  private static final double BUMP_HALF_Y = Units.inchesToMeters(47.0 / 2.0);
  // Net zone: 4ft deep into neutral zone, same Y width as hub (47in half = 23.5in)
  private static final double NET_DEPTH   = Units.inchesToMeters(48.0);
  private static final double NET_HALF_Y  = Units.inchesToMeters(47.0 / 2.0);
  // Tower shadow zone: 44in deep from driver wall x 47in wide, centered on tower Y
  private static final double TOWER_DEPTH_X  = Units.inchesToMeters(44.0);
  private static final double TOWER_HALF_Y   = Units.inchesToMeters(47.0 / 2.0);
  private static final double BLUE_TOWER_CENTER_Y = Units.inchesToMeters(147.0); // 147in from right (low-Y) wall
  private static final double RED_TOWER_CENTER_Y  =
      Constants.Field.FIELD_WIDTH_METERS - BLUE_TOWER_CENTER_Y;

  private boolean lastInAllianceZone   = true;  // default true so robot starts tracking hub, not pass target
  private boolean lastInOpponentZone   = false;
  private boolean lastOnLeftSide       = false;

  public TurretTargetSelector(PoseEstimatorSubsystem poseEstimator, RobotState robotState) {
    this.poseEstimator = poseEstimator;
    this.robotState = robotState;
  }

  @Override
  public Pose2d get() {
    Pose2d robotPose = poseEstimator.getEstimatedPose();
    boolean isRed = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);

    // Suppress auto fire when on the bump, in the net zone, or behind the alliance tower
    Pose2d blueHub = Constants.HubCenters.BLUE_HUB_CENTER;
    Pose2d redHub  = Constants.HubCenters.RED_HUB_CENTER;
    boolean suppressed = isInsideBump(robotPose, blueHub) || isInsideBump(robotPose, redHub)
        || isInsideNetZone(robotPose, blueHub, false)
        || isInsideNetZone(robotPose, redHub,  true)
        || isInsideTowerShadow(robotPose);
    robotState.setFieldZoneSuppressed(suppressed);

    double allianceEdge = isRed
        ? Constants.Field.FIELD_LENGTH_METERS - Constants.Field.ALLIANCE_ZONE_LENGTH_METERS
        : Constants.Field.ALLIANCE_ZONE_LENGTH_METERS;
    double opponentEdge = isRed
        ? Constants.Field.ALLIANCE_ZONE_LENGTH_METERS
        : Constants.Field.FIELD_LENGTH_METERS - Constants.Field.ALLIANCE_ZONE_LENGTH_METERS;

    // Hysteresis: widen the entry threshold vs exit threshold by ZONE_HYSTERESIS_METERS each side.
    if (lastInAllianceZone) {
      lastInAllianceZone = isRed
          ? robotPose.getX() > allianceEdge - ZONE_HYSTERESIS_METERS
          : robotPose.getX() < allianceEdge + ZONE_HYSTERESIS_METERS;
    } else {
      lastInAllianceZone = isRed
          ? robotPose.getX() > allianceEdge + ZONE_HYSTERESIS_METERS
          : robotPose.getX() < allianceEdge - ZONE_HYSTERESIS_METERS;
    }

    if (lastInOpponentZone) {
      lastInOpponentZone = isRed
          ? robotPose.getX() < opponentEdge + ZONE_HYSTERESIS_METERS
          : robotPose.getX() > opponentEdge - ZONE_HYSTERESIS_METERS;
    } else {
      lastInOpponentZone = isRed
          ? robotPose.getX() < opponentEdge - ZONE_HYSTERESIS_METERS
          : robotPose.getX() > opponentEdge + ZONE_HYSTERESIS_METERS;
    }

    // Update shooting zone on RobotState for phase-aware shoot suppression.
    RobotState.ShootingZone zone = lastInAllianceZone ? RobotState.ShootingZone.ALLIANCE
        : lastInOpponentZone ? RobotState.ShootingZone.OPPONENT
        : RobotState.ShootingZone.NEUTRAL;
    robotState.setShootingZone(zone);

    if (lastInAllianceZone) {
      Pose2d target = isRed ? Constants.HubCenters.RED_HUB_CENTER : Constants.HubCenters.BLUE_HUB_CENTER;
      SmartLogger.logReplay("NeutralZonePassing/Zone", zone.toString());
      SmartLogger.logReplay("NeutralZonePassing/SelectedTarget", target);
      SmartLogger.logReplay("NeutralZonePassing/IsLeftSide", lastOnLeftSide);
      return target;
    }

    double fieldMidY = Constants.Field.FIELD_WIDTH_METERS / 2.0;
    if (lastOnLeftSide) {
      lastOnLeftSide = robotPose.getY() > fieldMidY - ZONE_HYSTERESIS_METERS;
    } else {
      lastOnLeftSide = robotPose.getY() > fieldMidY + ZONE_HYSTERESIS_METERS;
    }

    Pose2d passTarget;
    if (lastOnLeftSide) {
      passTarget = isRed ? Constants.PassTargets.RED_PASS_TARGET_RIGHT : Constants.PassTargets.BLUE_PASS_TARGET_LEFT;
    } else {
      passTarget = isRed ? Constants.PassTargets.RED_PASS_TARGET_LEFT : Constants.PassTargets.BLUE_PASS_TARGET_RIGHT;
    }

    SmartLogger.logReplay("NeutralZonePassing/Zone", zone.toString());
    SmartLogger.logReplay("NeutralZonePassing/SelectedTarget", passTarget);
    SmartLogger.logReplay("NeutralZonePassing/IsLeftSide", lastOnLeftSide);
    return passTarget;
  }

  // True when robot is inside the 73in x 47in bump rectangle centered on hubCenter
  private boolean isInsideBump(Pose2d robot, Pose2d hubCenter) {
    return Math.abs(robot.getX() - hubCenter.getX()) <= BUMP_HALF_X
        && Math.abs(robot.getY() - hubCenter.getY()) <= BUMP_HALF_Y;
  }

  // True when robot is in the 4ft-deep net zone on the neutral side of hubCenter.
  // isRed=false (Blue hub): net extends toward higher X (field center).
  // isRed=true  (Red hub):  net extends toward lower X (field center).
  private boolean isInsideNetZone(Pose2d robot, Pose2d hubCenter, boolean isRed) {
    if (Math.abs(robot.getY() - hubCenter.getY()) > NET_HALF_Y) return false;
    if (isRed) {
      return robot.getX() >= hubCenter.getX() - NET_DEPTH && robot.getX() < hubCenter.getX();
    } else {
      return robot.getX() > hubCenter.getX() && robot.getX() <= hubCenter.getX() + NET_DEPTH;
    }
  }

  // True when robot is inside the 44in x 47in tower shadow against either driver wall.
  // Blue tower: X 0-44in, Red tower: X (field - 44in) to field end.
  private boolean isInsideTowerShadow(Pose2d robot) {
    boolean inBlue = robot.getX() <= TOWER_DEPTH_X
        && Math.abs(robot.getY() - BLUE_TOWER_CENTER_Y) <= TOWER_HALF_Y;
    boolean inRed  = robot.getX() >= Constants.Field.FIELD_LENGTH_METERS - TOWER_DEPTH_X
        && Math.abs(robot.getY() - RED_TOWER_CENTER_Y) <= TOWER_HALF_Y;
    return inBlue || inRed;
  }
}
