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
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Centimeters;

public final class Constants {
  private Constants() {}

  public static final int TEAM_NUMBER = 5142;
  public static final int DRIVER_CONTROLLER_PORT = 0;

  // Swerve drivetrain hardware config and PID tuning
  public static final class Swerve {
    // Robot dimensions (from Tuner X)
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(24.75);
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(24.75);
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    // Max speeds
    public static final double MAX_TRANSLATION_SPEED_MPS = 5.21;
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI * 4.0;

    // Driver speed modes (multiply by max speed)
    public static final double NORMAL_SPEED_SCALE = 0.6;
    public static final double PRECISION_SPEED_SCALE = 0.3;
    public static final double FAST_SPEED_SCALE = 1.0;
    public static final double JOYSTICK_DEADBAND = 0.10;

    // Module positions on robot frame (meters from center)
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);

    public static final String CAN_BUS_NAME = "Canivore";
    public static final int PIGEON_CAN_ID = 14;

    // Swerve module CAN IDs and CANcoder offsets (updated post-mechanical work)
    public static final int FRONT_LEFT_DRIVE_ID = 2;
    public static final int FRONT_LEFT_STEER_ID = 1;
    public static final int FRONT_LEFT_CANCODER_ID = 9;
    public static final double FRONT_LEFT_OFFSET_ROTATIONS = 0.48779296875;

    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_STEER_ID = 3;
    public static final int FRONT_RIGHT_CANCODER_ID = 10;
    public static final double FRONT_RIGHT_OFFSET_ROTATIONS = 0.209228515625;

    public static final int BACK_LEFT_DRIVE_ID = 8;
    public static final int BACK_LEFT_STEER_ID = 7;
    public static final int BACK_LEFT_CANCODER_ID = 12;
    public static final double BACK_LEFT_OFFSET_ROTATIONS = -0.296142578125;

    public static final int BACK_RIGHT_DRIVE_ID = 6;
    public static final int BACK_RIGHT_STEER_ID = 5;
    public static final int BACK_RIGHT_CANCODER_ID = 11;
    public static final double BACK_RIGHT_OFFSET_ROTATIONS = 0.3994140625;

    // Motor inversions (set by Tuner X based on motor mounting)
    public static final boolean FRONT_LEFT_DRIVE_INVERTED = false;
    public static final boolean FRONT_LEFT_STEER_INVERTED = true;
    public static final boolean FRONT_RIGHT_DRIVE_INVERTED = true;
    public static final boolean FRONT_RIGHT_STEER_INVERTED = true;
    public static final boolean BACK_LEFT_DRIVE_INVERTED = false;
    public static final boolean BACK_LEFT_STEER_INVERTED = true;
    public static final boolean BACK_RIGHT_DRIVE_INVERTED = true;
    public static final boolean BACK_RIGHT_STEER_INVERTED = true;

    // Gear ratios (drive train specifications)
    public static final double DRIVE_GEAR_RATIO = 6.122448979591837;
    public static final double STEER_GEAR_RATIO = 21.428571428571427;
    public static final double COUPLING_RATIO = 3.5714285714285716;
    public static final Current SLIP_CURRENT = Amps.of(120.0);

    // PID gains (tuned via SysId characterization tool)
    public static final class SteerGains {
      public static final double kP = 48.633;
      public static final double kI = 0.0;
      public static final double kD = 3.2691;
      public static final double kS = 0.0782;
      public static final double kV = 2.6267;
      public static final double kA = 0.11226;
    }

    public static final class DriveGains {
      public static final double kP = 0.16077;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kS = 0.17399;
      public static final double kV = 0.11894;
      public static final double kA = 0.003069;
    }

    public static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    public static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    public static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    // Motor configuration helpers (used by TunerConstants)
    public static TalonFXConfiguration createDriveMotorConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = TunableCTREGains.DRIVE_KP.get();
      config.Slot0.kI = TunableCTREGains.DRIVE_KI.get();
      config.Slot0.kD = TunableCTREGains.DRIVE_KD.get();
      config.Slot0.kS = TunableCTREGains.DRIVE_KS.get();
      config.Slot0.kV = TunableCTREGains.DRIVE_KV.get();
      config.Slot0.kA = TunableCTREGains.DRIVE_KA.get();
      config.ClosedLoopGeneral.ContinuousWrap = false;
      return config;
    }

    public static TalonFXConfiguration createSteerMotorConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = TunableCTREGains.STEER_KP.get();
      config.Slot0.kI = TunableCTREGains.STEER_KI.get();
      config.Slot0.kD = TunableCTREGains.STEER_KD.get();
      config.Slot0.kS = TunableCTREGains.STEER_KS.get();
      config.Slot0.kV = TunableCTREGains.STEER_KV.get();
      config.Slot0.kA = TunableCTREGains.STEER_KA.get();
      config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
      config.ClosedLoopGeneral.ContinuousWrap = true;
      config.CurrentLimits.StatorCurrentLimit = 60.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      return config;
    }

    public static CANcoderConfiguration createCancoderConfig() {
      CANcoderConfiguration config = new CANcoderConfiguration();
      config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      return config;
    }
  }

  // Vision cameras for AprilTag pose estimation and object detection
  public static final class Vision {
    // Camera names (NetworkTables identifiers)
    public static final String LL_FRONT_NAME = "limelight-front";
    public static final String PV_BACK_LEFT_NAME = "RLTagPV";
    public static final String PV_BACK_RIGHT_NAME = "RRTagPV";
    public static final String OBJ_CAMERA_NAME = "FObjPV";

    // Pose estimation filters
    public static final double MAX_AMBIGUITY = 0.3;
    public static final double MAX_POSE_DIFFERENCE_METERS = 0.75;
    public static final int MIN_TAG_COUNT_FOR_MULTI = 2;

    // Trust levels (lower = more trust) - Limelight > PhotonVision > QuestNav > Odometry
    public static final double[] LIMELIGHT_MULTI_TAG_STD_DEVS = {0.03, 0.03, 0.05};
    public static final double[] LIMELIGHT_SINGLE_TAG_STD_DEVS = {0.1, 0.1, 0.15};
    public static final double[] PHOTON_MULTI_TAG_STD_DEVS = {0.05, 0.05, 0.1};
    public static final double[] PHOTON_SINGLE_TAG_STD_DEVS = {0.15, 0.15, 0.25};
    public static final double[] ODOMETRY_STD_DEVS = {1.0, 1.0, 1.0};

    // Limelight 3 mounting (front center, tilted up 10 degrees)
    public static final double FRONT_LL_X_METERS = 0.3;
    public static final double FRONT_LL_Y_METERS = 0.0;
    public static final double FRONT_LL_Z_METERS = 0.2;
    public static final double FRONT_LL_ROLL_DEG = 0.0;
    public static final double FRONT_LL_PITCH_DEG = 10;
    public static final double FRONT_LL_YAW_DEG = 0.0;

    // PhotonVision Back Left (mounted on back, angled forward-left 135 degrees, tilted back 15 degrees)
    public static final double BACK_LEFT_PV_X_METERS = Units.inchesToMeters(-11.5);
    public static final double BACK_LEFT_PV_Y_METERS = Units.inchesToMeters(10.0);
    public static final double BACK_LEFT_PV_Z_METERS = Units.inchesToMeters(8.0);
    public static final double BACK_LEFT_PV_ROLL_DEG = 0.0;
    public static final double BACK_LEFT_PV_PITCH_DEG = 15.0;
    public static final double BACK_LEFT_PV_YAW_DEG = 135.0;
    public static final double BACK_LEFT_PV_FOV_DEG = 100.0;

    // PhotonVision Back Right (mounted on back, angled forward-right 225 degrees, tilted back 15 degrees)
    public static final double BACK_RIGHT_PV_X_METERS = Units.inchesToMeters(-11.5);
    public static final double BACK_RIGHT_PV_Y_METERS = Units.inchesToMeters(-10.0);
    public static final double BACK_RIGHT_PV_Z_METERS = Units.inchesToMeters(8.0);
    public static final double BACK_RIGHT_PV_ROLL_DEG = 0.0;
    public static final double BACK_RIGHT_PV_PITCH_DEG = 15.0;
    public static final double BACK_RIGHT_PV_YAW_DEG = 225.0;
    public static final double BACK_RIGHT_PV_FOV_DEG = 100.0;

    // PhotonVision Object Detection (front right corner, driver view camera)
    public static final double OBJ_CAMERA_X_METERS = Units.inchesToMeters(12.0);
    public static final double OBJ_CAMERA_Y_METERS = Units.inchesToMeters(5.5);
    public static final double OBJ_CAMERA_Z_METERS = Units.inchesToMeters(9.75);
    public static final double OBJ_CAMERA_ROLL_DEG = 0.0;
    public static final double OBJ_CAMERA_PITCH_DEG = 0.0;
    public static final double OBJ_CAMERA_YAW_DEG = 0.0;
    public static final double OBJ_CAMERA_FOV_DEG = 120.0;
    
    // 2025 game piece heights (placeholders until 2026 game reveal)
    public static final double CORAL_HEIGHT_METERS = Units.inchesToMeters(6.0);
    public static final double ALGAE_HEIGHT_METERS = Units.inchesToMeters(3.0);
    
    public static final double MIN_TARGET_AREA_PERCENT = 0.1;
    public static final double MAX_TARGET_DISTANCE_METERS = 5.0;
  }

  // QuestNav SLAM sensor (USB/Ethernet connected IMU with visual odometry)
  public static final class QuestNav {
    // Mounting position (forward 14 inches, slightly left)
    public static final double QUEST_X_METERS = Units.inchesToMeters(14.0);
    public static final double QUEST_Y_METERS = Units.inchesToMeters(0.5);
    public static final double QUEST_Z_METERS = Units.inchesToMeters(0.0);
    public static final double QUEST_YAW_DEG = 0.0;

    // Failover timing (switch to Pigeon if QuestNav disconnects)
    public static final double MAX_QUESTNAV_DISCONNECT_TIME_SECONDS = 0.5;
    public static final double MAX_ANGULAR_RATE_DEG_PER_SEC = 720.0;
    
    // Trust levels (lower = more trust)
    public static final double[] QUESTNAV_STD_DEVS = {0.08, 0.08, 0.07};
    public static final double QUESTNAV_LATENCY_MS = 5.0;
    
    // Motion gating (when to accept QuestNav data)
    public static final double QUESTNAV_MAX_LINEAR_SPEED_MPS = 3.0;
    public static final double QUESTNAV_MAX_OMEGA_RAD_PER_SEC = 2.0;
    
    // Stopped trust (higher trust when robot not moving)
    public static final double[] QUESTNAV_STD_DEVS_STOPPED = {0.02, 0.02, 0.03};
    public static final double[] QUESTNAV_STD_DEVS_INITIAL = {0.01, 0.01, 0.02};
    
    // Post-path correction (fine-tune position after PathPlanner finishes)
    public static final double POST_PATH_CORRECTION_THRESHOLD_M = 0.03;
    public static final double POST_PATH_CORRECTION_MAX_M = 0.15;
    public static final double POST_PATH_CORRECTION_TIMEOUT_S = 1.0;
    public static final double POST_PATH_SETTLE_TIME_S = 0.5;
  }

  // Autonomous path following (PathPlanner PID tuning)
  public static final class Auto {
    // Translation PID (trust your SysId drive gains!)
    public static final double TRANSLATION_KP = 3.0;  // Was 8.0 (62% reduction)
    public static final double TRANSLATION_KI = 0.0;  // Keep zero
    public static final double TRANSLATION_KD = 0.2;  // Was 0.15 (more damping)
    
    // Rotation PID (trust your SysId steer gains!)
    public static final double ROTATION_KP = 4.0;     // Was 9.0 (56% reduction)
    public static final double ROTATION_KI = 0.0;     // Keep zero
    public static final double ROTATION_KD = 0.3;     // Was 0.2 (prevent spin wobble)
    
    public static final double MAX_MODULE_SPEED_MPS = Swerve.MAX_TRANSLATION_SPEED_MPS;
    
    // Starting pose validation (how close robot must be to expected start)
    public static final double STARTING_POSE_TOLERANCE_METERS = 0.15;
    public static final double STARTING_POSE_TOLERANCE_DEGREES = 5.0;
    public static final double VISION_INITIALIZATION_TIMEOUT_SECONDS = 7.0;
    public static final Pose2d DEFAULT_FALLBACK_POSE = StartingPositions.BLUE_REEF_TAG_17;
  }

  // Field positions (all blue alliance - red is mirrored automatically)
  public static final class StartingPositions {
    // Staging poses (Phase 1: PathPlanner pathfind targets)
    public static final Pose2d BLUE_REEF_TAG_17 = new Pose2d(3.359, 2.077, Rotation2d.fromDegrees(60.0));
    public static final Pose2d BLUE_REEF_TAG_18 = new Pose2d(2.070, 4.027, Rotation2d.fromDegrees(0.0));
    public static final Pose2d BLUE_REEF_TAG_21 = new Pose2d(6.889, 4.025, Rotation2d.fromDegrees(180.0));
    public static final Pose2d BLUE_REEF_TAG_22 = new Pose2d(5.552, 2.168, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BLUE_TAG_16 = new Pose2d(5.990, 1.543, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d BLUE_TAG_12 = new Pose2d(2.300, 1.850, Rotation2d.fromDegrees(-130.0));
    public static final Pose2d BLUE_AUTO_START_POS_FAR_RIGHT = new Pose2d(6.033, 0.985, Rotation2d.fromDegrees(180.0));
    public static final Pose2d PID_TUNING_POSITION = new Pose2d(1.270, 2.230, Rotation2d.fromDegrees(0.0));
    
    // Precision poses (Phase 2: AutoPilot final targets)
    public static final Pose2d PRECISE_12_POSE = new Pose2d(1.35, 1.07, Rotation2d.fromDegrees(-130.0));
    public static final Pose2d PRECISE_16_POSE = new Pose2d(5.99, 0.56, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d PRECISE_17_POSE = new Pose2d(3.92, 2.9, Rotation2d.fromDegrees(60.0));
    public static final Pose2d PRECISE_18_POSE = new Pose2d(3.233, 4.027, Rotation2d.fromDegrees(0.0));
    public static final Pose2d PRECISE_21_POSE = new Pose2d(5.741, 4.025, Rotation2d.fromDegrees(180.0));
    public static final Pose2d PRECISE_22_POSE = new Pose2d(5.114, 2.941, Rotation2d.fromDegrees(120.0));
  }

  // AutoPilot precision navigation library (singleton instances)
  public static final class AutoPilotConstants {
    // Test profile (1m test movements)
    private static final APConstraints TEST_CONSTRAINTS = new APConstraints()
        .withAcceleration(5.0)
        .withJerk(2.0);
    
    private static final APProfile TEST_PROFILE = new APProfile(TEST_CONSTRAINTS)
        .withErrorXY(Centimeters.of(5))
        .withErrorTheta(Degrees.of(2.0))
        .withBeelineRadius(Centimeters.of(8));
    
    public static final Autopilot TEST_AUTOPILOT = new Autopilot(TEST_PROFILE);
    
    // Precision profile (SmartDrive Phase 2)
    private static final APConstraints PRECISION_CONSTRAINTS = new APConstraints()
        .withAcceleration(8.0)
        .withJerk(4.0);
    
    private static final APProfile PRECISION_PROFILE = new APProfile(PRECISION_CONSTRAINTS)
        .withErrorXY(Centimeters.of(5))
        .withErrorTheta(Degrees.of(2.0))
        .withBeelineRadius(Centimeters.of(8));
    
    public static final Autopilot PRECISION_AUTOPILOT = new Autopilot(PRECISION_PROFILE);
    
    public static final double DEFAULT_MAX_ACCELERATION = 8.0;
    public static final double DEFAULT_MAX_JERK = 4.0;
    public static final double DEFAULT_ERROR_XY_METERS = 0.05;
    public static final double DEFAULT_ERROR_THETA_DEGREES = 2.0;
  }
}