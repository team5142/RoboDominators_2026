package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

// Robot configuration constants - organized by subsystem/function
// This file defines physical properties, limits, and tuning values
// Most values come from measurements, CAD, or Tuner X characterization
public final class Constants {
  private Constants() {} // Prevent instantiation - this is a static container

  public static final int TEAM_NUMBER = 5142; // Used for robot name and network config
  public static final int DRIVER_CONTROLLER_PORT = 0; // USB port on driver station computer

  public static final class Swerve {
    // Drivetrain physical dimensions - measured from CAD and Tuner X
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(24.75); // Left-right wheel spacing
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(24.75); // Front-back wheel spacing
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0); // 4" wheel radius

    // Speed limits - from Tuner X SysId characterization at 12V
    public static final double MAX_TRANSLATION_SPEED_MPS = 5.21; // Maximum linear speed
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI * 2.0; // Maximum rotation speed (360 deg/s)

    // Driver control scaling - allows smooth driving at different speed levels
    public static final double NORMAL_SPEED_SCALE = 0.6; // Default speed for precise control
    public static final double PRECISION_SPEED_SCALE = 0.3; // Slow mode for fine positioning
    public static final double FAST_SPEED_SCALE = 1.0; // Full speed for rapid movement

    public static final double JOYSTICK_DEADBAND = 0.10; // Ignore stick drift below 10%

    // Module positions relative to robot center - used for kinematics calculations
    public static final Translation2d FRONT_LEFT_LOCATION =
        new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d FRONT_RIGHT_LOCATION =
        new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d BACK_LEFT_LOCATION =
        new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d BACK_RIGHT_LOCATION =
        new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);

    // CAN bus configuration - all swerve devices on CANivore for faster/more reliable updates
    public static final String CAN_BUS_NAME = "Canivore";

    // Gyroscope - used as backup when QuestNav (Quest 3 VR) loses tracking
    public static final int PIGEON_CAN_ID = 14;

    // Swerve module CAN device IDs - do not change unless rewiring robot
    // Front Left module
    public static final int FRONT_LEFT_DRIVE_ID = 2;
    public static final int FRONT_LEFT_STEER_ID = 1;
    public static final int FRONT_LEFT_CANCODER_ID = 9;
    public static final double FRONT_LEFT_OFFSET_ROTATIONS = 0.484130859375; // From Tuner X bringup

    // Front Right module
    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_STEER_ID = 3;
    public static final int FRONT_RIGHT_CANCODER_ID = 10;
    public static final double FRONT_RIGHT_OFFSET_ROTATIONS = 0.20703125; // From Tuner X bringup

    // Back Left module
    public static final int BACK_LEFT_DRIVE_ID = 8;
    public static final int BACK_LEFT_STEER_ID = 7;
    public static final int BACK_LEFT_CANCODER_ID = 12;
    public static final double BACK_LEFT_OFFSET_ROTATIONS = -0.30908203125; // From Tuner X bringup

    // Back Right module
    public static final int BACK_RIGHT_DRIVE_ID = 6;
    public static final int BACK_RIGHT_STEER_ID = 5;
    public static final int BACK_RIGHT_CANCODER_ID = 11;
    public static final double BACK_RIGHT_OFFSET_ROTATIONS = 0.396240234375; // From Tuner X bringup

    // Motor direction configuration - ensures all modules spin correctly
    // Left side motors NOT inverted (false), right side motors inverted (true)
    public static final boolean FRONT_LEFT_DRIVE_INVERTED = false;
    public static final boolean FRONT_LEFT_STEER_INVERTED = true;
    public static final boolean FRONT_RIGHT_DRIVE_INVERTED = true;
    public static final boolean FRONT_RIGHT_STEER_INVERTED = true;
    public static final boolean BACK_LEFT_DRIVE_INVERTED = false;
    public static final boolean BACK_LEFT_STEER_INVERTED = true;
    public static final boolean BACK_RIGHT_DRIVE_INVERTED = true;
    public static final boolean BACK_RIGHT_STEER_INVERTED = true;

    // Mechanical gear ratios - from SDS Mk4i swerve module specifications
    public static final double DRIVE_GEAR_RATIO = 6.122448979591837; // L3 gearing
    public static final double STEER_GEAR_RATIO = 21.428571428571427; // Steering reduction
    public static final double COUPLING_RATIO = 3.5714285714285716; // Mechanical coupling between steer and drive

    // Current limiting - prevents brownouts during aggressive maneuvers
    public static final Current SLIP_CURRENT = Amps.of(120.0);

    // PID tuning for steering motors - reduced from Tuner X defaults for smoother low-speed control
    public static final class SteerGains {
      public static final double kP = 50.0; // Reduced from 100.0 for less aggressive corrections
      public static final double kI = 0.0; // No integral term needed
      public static final double kD = 1.0; // Increased from 0.5 for more damping
      public static final double kS = 0.1; // Static friction compensation
      public static final double kV = 2.66; // Velocity feedforward
      public static final double kA = 0.0; // No acceleration feedforward
    }

    // PID tuning for drive motors - from Tuner X SysId characterization
    public static final class DriveGains {
      public static final double kP = 0.1;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kS = 0.0; // Static friction (very low for Falcon 500)
      public static final double kV = 0.124; // Velocity feedforward (volts per meter/sec)
    }

    // Motor control configuration - voltage control is more predictable than duty cycle
    public static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    public static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    // Angle sensor feedback - FusedCANcoder combines motor encoder with absolute CANcoder
    public static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    // Factory methods for creating motor configurations - ensures consistency across all modules
    public static TalonFXConfiguration createDriveMotorConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = DriveGains.kP;
      config.Slot0.kI = DriveGains.kI;
      config.Slot0.kD = DriveGains.kD;
      config.Slot0.kS = DriveGains.kS;
      config.Slot0.kV = DriveGains.kV;
      config.ClosedLoopGeneral.ContinuousWrap = false; // Drive motors don't wrap around
      return config;
    }

    public static TalonFXConfiguration createSteerMotorConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = SteerGains.kP;
      config.Slot0.kI = SteerGains.kI;
      config.Slot0.kD = SteerGains.kD;
      config.Slot0.kS = SteerGains.kS;
      config.Slot0.kV = SteerGains.kV;
      config.Slot0.kA = SteerGains.kA;
      config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
      config.ClosedLoopGeneral.ContinuousWrap = true; // Steer motors wrap at 360 degrees
      config.CurrentLimits.StatorCurrentLimit = 60.0; // Prevent steer motor overheating
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      return config;
    }

    public static CANcoderConfiguration createCancoderConfig() {
      CANcoderConfiguration config = new CANcoderConfiguration();
      config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      return config;
    }
  }

  public static final class Vision {
    // Camera network names - must match hostnames in camera web interfaces
    public static final String LL_FRONT_NAME = "limelight-front"; // Limelight 3 on front
    public static final String PV_BACK_LEFT_NAME = "RLTagPV"; // PhotonVision back-left
    public static final String PV_BACK_RIGHT_NAME = "RRTagPV"; // PhotonVision back-right
    public static final String OBJ_CAMERA_NAME = "FObjPV"; // PhotonVision object detection

    // Vision quality thresholds - reject bad measurements to prevent odometry corruption
    public static final double MAX_AMBIGUITY = 0.3; // Reject tags with poor fit (0-1 scale)
    public static final double MAX_POSE_DIFFERENCE_METERS = 2.0; // Reject if too far from current estimate
    public static final int MIN_TAG_COUNT_FOR_MULTI = 2; // Need 2+ tags for multi-tag trust boost

    // Standard deviation tuning - tells Kalman filter how much to trust each source
    // Lower numbers = higher trust. Vision is most accurate, odometry drifts over time
    public static final double[] VISION_STD_DEVS_SINGLE_TAG = {0.3, 0.3, 0.5}; // Moderate trust
    public static final double[] VISION_STD_DEVS_MULTI_TAG = {0.1, 0.1, 0.2}; // High trust (2+ tags)
    public static final double[] ODOMETRY_STD_DEVS = {1.0, 1.0, 1.0}; // Low trust (wheels slip)

    // Limelight 3 physical mounting - front center of robot
    public static final double FRONT_LL_X_METERS = 0.3; // 30cm forward from center
    public static final double FRONT_LL_Y_METERS = 0.0; // Centered left-right
    public static final double FRONT_LL_Z_METERS = 0.2; // 20cm above ground
    public static final double FRONT_LL_ROLL_DEG = 0.0; // No sideways tilt
    public static final double FRONT_LL_PITCH_DEG = 0.0; // Level (not angled)
    public static final double FRONT_LL_YAW_DEG = 0.0; // Facing forward

    // PhotonVision Back Left camera mounting
    // Mounted on back-left corner, angled forward-left to see tags beside robot
    public static final double BACK_LEFT_PV_X_METERS = Units.inchesToMeters(-11.5); // 11.5" behind center
    public static final double BACK_LEFT_PV_Y_METERS = Units.inchesToMeters(10.0); // 10" left of center
    public static final double BACK_LEFT_PV_Z_METERS = Units.inchesToMeters(8.0); // 8" above ground
    public static final double BACK_LEFT_PV_ROLL_DEG = 0.0; // No sideways tilt
    public static final double BACK_LEFT_PV_PITCH_DEG = 15.0; // Tilted back to see tags above robot
    public static final double BACK_LEFT_PV_YAW_DEG = 135.0; // Facing forward-left (45deg from back)
    public static final double BACK_LEFT_PV_FOV_DEG = 100.0; // OV9281 field of view

    // PhotonVision Back Right camera mounting
    // Mounted on back-right corner, angled forward-right to see tags beside robot
    public static final double BACK_RIGHT_PV_X_METERS = Units.inchesToMeters(-11.5); // 11.5" behind center
    public static final double BACK_RIGHT_PV_Y_METERS = Units.inchesToMeters(-10.0); // 10" right of center
    public static final double BACK_RIGHT_PV_Z_METERS = Units.inchesToMeters(8.0); // 8" above ground
    public static final double BACK_RIGHT_PV_ROLL_DEG = 0.0; // No sideways tilt
    public static final double BACK_RIGHT_PV_PITCH_DEG = 15.0; // Tilted back to see tags above robot
    public static final double BACK_RIGHT_PV_YAW_DEG = 225.0; // Facing forward-right (45deg from back)
    public static final double BACK_RIGHT_PV_FOV_DEG = 100.0; // OV9281 field of view

    // PhotonVision Object Detection camera mounting - front of robot
    public static final double OBJ_CAMERA_X_METERS = Units.inchesToMeters(12.0); // 12" forward
    public static final double OBJ_CAMERA_Y_METERS = Units.inchesToMeters(5.5); // 5.5" left
    public static final double OBJ_CAMERA_Z_METERS = Units.inchesToMeters(9.75); // 9.75" high
    public static final double OBJ_CAMERA_ROLL_DEG = 0.0;
    public static final double OBJ_CAMERA_PITCH_DEG = 0.0;
    public static final double OBJ_CAMERA_YAW_DEG = 0.0;
    public static final double OBJ_CAMERA_FOV_DEG = 120.0; // OV9782 wide-angle FOV
    
    // Game piece detection heights - temporary for 2025 Reefscape (update for 2026 game)
    public static final double CORAL_HEIGHT_METERS = Units.inchesToMeters(6.0);
    public static final double ALGAE_HEIGHT_METERS = Units.inchesToMeters(3.0);
    
    // Object detection filtering - ignore noise and distant objects
    public static final double MIN_TARGET_AREA_PERCENT = 0.1; // Minimum 0.1% of image
    public static final double MAX_TARGET_DISTANCE_METERS = 5.0; // Ignore objects >5m away
  }

  public static final class QuestNav {
    // Meta Quest 3 VR headset position - used for SLAM-based odometry (backup/supplement to vision)
    // Quest provides 6DOF tracking via inside-out cameras (very accurate but can lose tracking)
    public static final double QUEST_X_METERS = Units.inchesToMeters(14.0); // 14" forward
    public static final double QUEST_Y_METERS = Units.inchesToMeters(0.5); // 0.5" left
    public static final double QUEST_Z_METERS = Units.inchesToMeters(0.0); // Ground level
    public static final double QUEST_YAW_DEG = 0.0; // Facing forward

    // Failover settings - automatically switch to Pigeon2 if Quest loses tracking
    public static final double MAX_QUESTNAV_DISCONNECT_TIME_SECONDS = 0.5; // Failover delay
    public static final double MAX_ANGULAR_RATE_DEG_PER_SEC = 720.0; // Reject unrealistic rotation speeds
    
    // Trust settings - QuestNav is very accurate, trust it more than wheel odometry
    public static final double[] QUESTNAV_STD_DEVS = {0.05, 0.05, 0.05}; // High trust (20x more than odometry)
  }

  public static final class Auto {
    // PathPlanner autonomous path following PID tuning
    // Higher kP = more aggressive correction, but can cause oscillation
    public static final double TRANSLATION_KP = 1.0; // Increased from 0.5 for faster response
    public static final double TRANSLATION_KI = 0.0; // No integral needed
    public static final double TRANSLATION_KD = 0.0; // No derivative needed
    
    public static final double ROTATION_KP = 1.0; // Increased from 0.5 for faster heading correction
    public static final double ROTATION_KI = 0.0; // No integral needed
    public static final double ROTATION_KD = 0.0; // No derivative needed
    
    // Path constraints - must not exceed swerve drivetrain capabilities
    public static final double MAX_MODULE_SPEED_MPS = Swerve.MAX_TRANSLATION_SPEED_MPS;
    
    // Starting position validation - warn if robot not at expected auto starting pose
    public static final double STARTING_POSE_TOLERANCE_METERS = 0.15; // 15cm tolerance
    public static final double STARTING_POSE_TOLERANCE_DEGREES = 5.0; // 5 degree tolerance

    // Vision initialization - wait for cameras before starting auto
    public static final double VISION_INITIALIZATION_TIMEOUT_SECONDS = 7.0;
    
    // Fallback if vision never initializes - use safe default position
    public static final Pose2d DEFAULT_FALLBACK_POSE = StartingPositions.BLUE_REEF_TAG_17;
  }

  public static final class StartingPositions {
    // Common robot starting positions on 2025 Reefscape field (Blue alliance)
    // Coordinates are in meters (X = downfield, Y = left-right, Rotation = field-relative heading)
    // All positions measured from field CAD or AprilTag layout
    
    public static final Pose2d BLUE_REEF_TAG_17 = new Pose2d(3.897, 2.957, Rotation2d.fromDegrees(60.0)); // Reef center
    public static final Pose2d BLUE_REEF_TAG_16 = new Pose2d(3.897, 4.115, Rotation2d.fromDegrees(120.0)); // Reef left
    public static final Pose2d BLUE_REEF_TAG_18 = new Pose2d(3.897, 1.799, Rotation2d.fromDegrees(0.0)); // Reef right
    public static final Pose2d BLUE_REEF_TAG_19 = new Pose2d(3.2, 2.0, Rotation2d.fromDegrees(-60.0)); // Reef lower-left
    public static final Pose2d BLUE_REEF_TAG_20 = new Pose2d(3.2, 3.9, Rotation2d.fromDegrees(180.0)); // Reef upper-left
    public static final Pose2d BLUE_REEF_TAG_21 = new Pose2d(3.5, 2.5, Rotation2d.fromDegrees(0.0)); // Reef top
    public static final Pose2d BLUE_REEF_TAG_22 = new Pose2d(3.5, 3.5, Rotation2d.fromDegrees(0.0)); // Reef top-right
    public static final Pose2d BLUE_TAG_16 = new Pose2d(5.9, 0.5, Rotation2d.fromDegrees(-90.0)); // Processor station
    public static final Pose2d BLUE_TAG_12 = new Pose2d(1.87, 0.73, Rotation2d.fromDegrees(-120)); // Coral station
    
    // Red alliance positions are mirrored by PathPlanner automatically
  }
}
