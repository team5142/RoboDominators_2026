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
  public static final int BLINKIN_PWM_PORT = 0; // Change to your actual PWM port

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
    
    // Trust levels (lower = more trust) - Like AprilTag vision, NOT odometry
    public static final double[] QUESTNAV_STD_DEVS = {0.08, 0.08, 0.07}; // Moving
    public static final double QUESTNAV_LATENCY_MS = 5.0;
    
    // 1) VELOCITY GATING (enforce "stopped robot" requirement)
    // Reject fusion during motion to prevent fighting PathPlanner
    public static final double MAX_LINEAR_SPEED_FOR_FUSION_MPS = 1.8; // ~5 in/s
    public static final double MAX_ANGULAR_SPEED_FOR_FUSION_RAD_PER_SEC = 1.5; // ~17 deg/s
    
    // Stopped trust (HIGHEST trust when robot not moving - like stationary AprilTag)
    public static final double[] QUESTNAV_STD_DEVS_STOPPED = {0.02, 0.02, 0.03}; // 2cm XY, 1.7° theta
    public static final double[] QUESTNAV_STD_DEVS_INITIAL = {0.01, 0.01, 0.02}; // 1cm XY, 1.1° theta
    
    // 3) INNOVATION GATING (tighter gates for high-accuracy sensor)
    // Base gates (measurement must be close to estimate at measurement time)
    public static final double INNOVATION_GATE_POS_BASE_METERS = 0.10; // 10cm base
    public static final double INNOVATION_GATE_ROT_BASE_DEGREES = 5.0; // 5° base
    
    // Motion-based expansion (allow for robot motion during measurement age)
    public static final double INNOVATION_GATE_POS_PER_SPEED = 0.3; // 30% of linear speed * age
    public static final double INNOVATION_GATE_ROT_PER_OMEGA = 0.2; // 20% of angular speed * age
    
    // REACQUIRE gates (when stopped, widen gates to allow convergence)
    public static final double REACQUIRE_POS_GATE_METERS = 0.65; // 65cm when stopped
    public static final double REACQUIRE_ROT_GATE_DEGREES = 25.0; // 25° when stopped
    public static final double REACQUIRE_STOPPED_LINEAR_THRESHOLD = 0.05; // m/s
    public static final double REACQUIRE_STOPPED_ANGULAR_THRESHOLD = 0.05; // rad/s

    // === HEALTH MONITORING THRESHOLDS ===
    
    // Physical limits (for teleport detection)
    public static final double MAX_PHYSICAL_SPEED_MPS = 5.5; // Max robot speed + margin
    public static final double MAX_PHYSICAL_OMEGA_RAD_PER_SEC = 13.0; // ~745 deg/s
    public static final double PHYSICAL_PLAUSIBILITY_MARGIN = 1.3; // 30% over max for burst detection
    
    // Teleport rejection (absolute backstops)
    public static final double TELEPORT_TRANSLATION_METERS = 0.6; // 60cm jump in one frame
    public static final double TELEPORT_ROTATION_RADIANS = 0.9; // ~50° jump in one frame
    
    // Divergence detection (Quest vs Odom comparison)
    public static final double DIVERGENCE_THRESHOLD_METERS = 0.20; // 20cm disagreement per cycle
    public static final int DIVERGENCE_PATIENCE_CYCLES = 15; // 15 bad cycles = 300ms @ 50Hz
    public static final int DIVERGENCE_RECOVERY_CYCLES = 25; // 25 good cycles to recover
    public static final double DIVERGENCE_ANGULAR_WEIGHT = 0.3; // Convert rad to meters (30cm per radian)
    
    // Startup validation
    public static final double VALIDATION_TOLERANCE_METERS = 0.15; // 15cm from seed pose
    public static final double VALIDATION_TIMEOUT_SEC = 0.5; // 500ms max wait
    public static final int VALIDATION_REQUIRED_STREAK = 5; // 5 consecutive good samples
    
    // Reset grace period
    public static final double POST_RESET_GRACE_SEC = 0.1; // 100ms after resetPose()
    
    // DT safety
    public static final double MIN_DT_FOR_IMPLIED_VELOCITY = 0.005; // 5ms minimum (avoid divide-by-zero)
    
    // Motion-aware trust scaling (replaces binary velocity gate for trust)
    public static final double MOVING_TRUST_DEGRADATION_FACTOR = 1.5; // 1.5x std dev when moving fast
    public static final double DEGRADED_TRUST_FACTOR = 5.0; // 5x std dev when DEGRADED
    public static final double UNHEALTHY_TRUST_FACTOR = 50.0; // 50x std dev when UNHEALTHY (almost no influence)

    // === INITIALIZATION MODE ===
    public enum InitMode {
      COMP_SEED,    // Seed Quest to known auto start (match/competition)
      SHOP_RESUME   // Use Quest's existing tracking (practice/testing)
    }
    
    // Validation mode thresholds
    public static final double COMP_VALIDATION_TOLERANCE_METERS = 0.15; // Must match seed within 15cm
    public static final double SHOP_STABILITY_TOLERANCE_METERS = 0.05; // Must show <5cm drift while stopped
    public static final int SHOP_STABILITY_REQUIRED_CYCLES = 5; // 5 cycles of stability (100ms)
  }

  // Autonomous path following (PathPlanner PID tuning)
  public static final class Auto {
    // Translation PID (trust your SysId drive gains!)
    public static final double TRANSLATION_KP = 3.0;  // Keep (translation is smooth)
    public static final double TRANSLATION_KI = 0.0;  // Keep zero
    public static final double TRANSLATION_KD = 0.2;  // Keep (more damping)
    
    // Rotation PID (trust your SysId steer gains!)
    public static final double ROTATION_KP = 8.5;  // Was 3.0 (33% reduction - gentler rotation)
    public static final double ROTATION_KI = 0.0;     // Keep zero
    public static final double ROTATION_KD = 0.0;     // Was 0.3 (more damping to prevent oscillation)
    
    public static final double MAX_MODULE_SPEED_MPS = Swerve.MAX_TRANSLATION_SPEED_MPS;
    
    // Starting pose validation (how close robot must be to expected start)
    public static final double STARTING_POSE_TOLERANCE_METERS = 0.15;
    public static final double STARTING_POSE_TOLERANCE_DEGREES = 5.0;
    public static final double VISION_INITIALIZATION_TIMEOUT_SECONDS = 7.0;
    public static final Pose2d DEFAULT_FALLBACK_POSE = StartingPositions.BLUE_REEF_TAG_17;
    
    // Post-path correction timeout (SmartDrive precision phase)
    public static final double POST_PATH_CORRECTION_TIMEOUT_S = 3.0;
  }

  // Field positions (all blue alliance - red is mirrored automatically)
  public static final class StartingPositions {
    
    // TUNING POSITION
    public static final Pose2d PID_TUNING_POSITION = new Pose2d(1.270, 2.230, Rotation2d.fromDegrees(0.0));
    // LED CALIBRATION TEST POSITION (from actual Limelight reading)
    public static final Pose2d LED_TEST_POSITION = new Pose2d(1.2748, 2.3987, Rotation2d.fromDegrees(-6.56));
    
    // Staging poses (Phase 1: PathPlanner pathfind targets)
    public static final Pose2d BLUE_TAG_12 = new Pose2d(2.300, 1.850, Rotation2d.fromDegrees(-130.0));
    public static final Pose2d BLUE_TAG_16 = new Pose2d(5.990, 1.543, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d BLUE_REEF_TAG_17 = new Pose2d(3.359, 2.077, Rotation2d.fromDegrees(60.0));
    public static final Pose2d BLUE_REEF_TAG_18 = new Pose2d(2.070, 4.027, Rotation2d.fromDegrees(0.0));
    public static final Pose2d BLUE_REEF_TAG_21 = new Pose2d(6.889, 4.025, Rotation2d.fromDegrees(180.0));
    public static final Pose2d BLUE_REEF_TAG_22 = new Pose2d(5.552, 2.168, Rotation2d.fromDegrees(120.0));
    public static final Pose2d TEST_SPOT_1 = new Pose2d(5.84, 1.850, Rotation2d.fromDegrees(133.0));
    // AUTO RESET POSE STAGING
    public static final Pose2d BLUE_AUTO_START_POS_FAR_RIGHT = new Pose2d(5.533, 1.185, Rotation2d.fromDegrees(180.0));
    
    // Precision poses (Phase 2: AutoPilot final targets)
    public static final Pose2d PRECISE_12_POSE = new Pose2d(1.35, 1.07, Rotation2d.fromDegrees(-130.0));
    public static final Pose2d PRECISE_16_POSE = new Pose2d(5.99, 0.56, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d PRECISE_17_POSE = new Pose2d(3.92, 2.9, Rotation2d.fromDegrees(60.0));
    public static final Pose2d PRECISE_18_POSE = new Pose2d(3.233, 4.027, Rotation2d.fromDegrees(0.0));
    public static final Pose2d PRECISE_21_POSE = new Pose2d(5.741, 4.025, Rotation2d.fromDegrees(180.0));
    public static final Pose2d PRECISE_22_POSE = new Pose2d(5.114, 2.941, Rotation2d.fromDegrees(120.0));
    public static final Pose2d PRECISE_TEST_SPOT_1 = new Pose2d(6.66, 2.470, Rotation2d.fromDegrees(133.0));
    // AUTO RESET POSE PRECISE
    public static final Pose2d PRECISE_BLUE_AUTO_START_POS_FAR_RIGHT = new Pose2d(6.033, 0.985, Rotation2d.fromDegrees(180.0));
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
        .withAcceleration(10.0)//8 to 12
        .withJerk(5.0);//4 to 6
    
    private static final APProfile PRECISION_PROFILE = new APProfile(PRECISION_CONSTRAINTS)
        .withErrorXY(Centimeters.of(6))//5 to 8
        .withErrorTheta(Degrees.of(2.5))//2 to 3
        .withBeelineRadius(Centimeters.of(10));//8 to 12
    
    public static final Autopilot PRECISION_AUTOPILOT = new Autopilot(PRECISION_PROFILE);
    
    public static final double DEFAULT_MAX_ACCELERATION = 8.0;
    public static final double DEFAULT_MAX_JERK = 4.0;
    public static final double DEFAULT_ERROR_XY_METERS = 0.05;
    public static final double DEFAULT_ERROR_THETA_DEGREES = 2.0;
  }
}