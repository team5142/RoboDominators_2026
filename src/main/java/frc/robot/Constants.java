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

public final class Constants {
  private Constants() {}

  public static final int TEAM_NUMBER = 5142;
  public static final int DRIVER_CONTROLLER_PORT = 0;

  public static final class Swerve {
    // Physical dimensions from Tuner X
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(24.75); // 12.375 * 2
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(24.75); // 12.375 * 2
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    // Speed limits from Tuner X
    public static final double MAX_TRANSLATION_SPEED_MPS = 5.21; // kSpeedAt12Volts
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI * 4.0;

    public static final double NORMAL_SPEED_SCALE = 0.6;
    public static final double PRECISION_SPEED_SCALE = 0.3;
    public static final double FAST_SPEED_SCALE = 1.0;

    public static final double JOYSTICK_DEADBAND = 0.10;

    public static final Translation2d FRONT_LEFT_LOCATION =
        new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d FRONT_RIGHT_LOCATION =
        new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d BACK_LEFT_LOCATION =
        new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d BACK_RIGHT_LOCATION =
        new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);

    // CAN Bus
    public static final String CAN_BUS_NAME = "Canivore";

    // Pigeon 2
    public static final int PIGEON_CAN_ID = 14;

    // Module CAN IDs and offsets from Tuner X - UPDATED POST-MECHANICAL WORK
    // Front Left
    public static final int FRONT_LEFT_DRIVE_ID = 2;
    public static final int FRONT_LEFT_STEER_ID = 1;
    public static final int FRONT_LEFT_CANCODER_ID = 9;
    public static final double FRONT_LEFT_OFFSET_ROTATIONS = 0.48779296875;  // UPDATED (was 0.484130859375)

    // Front Right
    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_STEER_ID = 3;
    public static final int FRONT_RIGHT_CANCODER_ID = 10;
    public static final double FRONT_RIGHT_OFFSET_ROTATIONS = 0.209228515625;  // UPDATED (was 0.20703125)

    // Back Left
    public static final int BACK_LEFT_DRIVE_ID = 8;
    public static final int BACK_LEFT_STEER_ID = 7;
    public static final int BACK_LEFT_CANCODER_ID = 12;
    public static final double BACK_LEFT_OFFSET_ROTATIONS = -0.296142578125;  // UPDATED (was -0.30908203125)

    // Back Right
    public static final int BACK_RIGHT_DRIVE_ID = 6;
    public static final int BACK_RIGHT_STEER_ID = 5;
    public static final int BACK_RIGHT_CANCODER_ID = 11;
    public static final double BACK_RIGHT_OFFSET_ROTATIONS = 0.3994140625;  // UPDATED (was 0.396240234375)

    // Motor inversions from Tuner X
    public static final boolean FRONT_LEFT_DRIVE_INVERTED = false; // kInvertLeftSide
    public static final boolean FRONT_LEFT_STEER_INVERTED = true;  //  Back to true
    public static final boolean FRONT_RIGHT_DRIVE_INVERTED = true; // kInvertRightSide
    public static final boolean FRONT_RIGHT_STEER_INVERTED = true;
    public static final boolean BACK_LEFT_DRIVE_INVERTED = false;
    public static final boolean BACK_LEFT_STEER_INVERTED = true;   //  Back to true
    public static final boolean BACK_RIGHT_DRIVE_INVERTED = true;
    public static final boolean BACK_RIGHT_STEER_INVERTED = true;

    // Gear ratios from Tuner X
    public static final double DRIVE_GEAR_RATIO = 6.122448979591837;
    public static final double STEER_GEAR_RATIO = 21.428571428571427;
    public static final double COUPLING_RATIO = 3.5714285714285716;

    // Slip current
    public static final Current SLIP_CURRENT = Amps.of(120.0);

    // PID Gains - UPDATED WITH SYSID CHARACTERIZATION
    public static final class SteerGains {
      // SysId-tuned values from test project (post-mechanical maintenance)
      public static final double kP = 48.633;   // Was 60.0
      public static final double kI = 0.0;
      public static final double kD = 3.2691;   // Was 1.5
      public static final double kS = 0.0782;   // Was 0.1
      public static final double kV = 2.6267;   // Was 2.66
      public static final double kA = 0.11226;  // Was 0.0 (NEW!)
    }

    public static final class DriveGains {
      // SysId-tuned values from test project (post-mechanical maintenance)
      public static final double kP = 0.16077;  // Was 0.1
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kS = 0.17399;  // Was 0.0 (NEW!)
      public static final double kV = 0.11894;  // Was 0.124
      public static final double kA = 0.003069; // NEW constant (add to createDriveMotorConfig())
    }

    // Control output types
    public static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    public static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    // Feedback type
    public static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    /**
     * Creates a standard TalonFX configuration for drive motors
     * NOW USES TUNABLE VALUES from AdvantageScope
     */
    public static TalonFXConfiguration createDriveMotorConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      
      config.Slot0.kP = TunableCTREGains.DRIVE_KP.get();
      config.Slot0.kI = TunableCTREGains.DRIVE_KI.get();
      config.Slot0.kD = TunableCTREGains.DRIVE_KD.get();
      config.Slot0.kS = TunableCTREGains.DRIVE_KS.get();
      config.Slot0.kV = TunableCTREGains.DRIVE_KV.get();
      config.Slot0.kA = TunableCTREGains.DRIVE_KA.get();  // NEW: Added kA support

      config.ClosedLoopGeneral.ContinuousWrap = false;
      
      return config;
    }

    /**
     * Creates a standard TalonFX configuration for steer motors
     * NOW USES TUNABLE VALUES from AdvantageScope
     */
    public static TalonFXConfiguration createSteerMotorConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      
      // Use tunable values instead of constants
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

    /**
     * Creates a standard CANcoder configuration
     */
    public static CANcoderConfiguration createCancoderConfig() {
      CANcoderConfiguration config = new CANcoderConfiguration();
      config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      return config;
    }
  }

  public static final class Vision {
    // AprilTag cameras for pose estimation
    
    // Limelight 3 - Front camera (existing)
    public static final String LL_FRONT_NAME = "limelight-front";
    
    // PhotonVision - Back cameras (UPDATED NAMES)
    public static final String PV_BACK_LEFT_NAME = "RLTagPV";     // Back Left Tag camera
    public static final String PV_BACK_RIGHT_NAME = "RRTagPV";    // Back Right Tag camera
    
    // PhotonVision - Front object detection camera (UPDATED NAME)
    public static final String OBJ_CAMERA_NAME = "FObjPV";        // Front Object Detection camera

    // Vision pose estimation tuning
    public static final double MAX_AMBIGUITY = 0.3;
    public static final double MAX_POSE_DIFFERENCE_METERS = 0.75;  // CHANGED: 75cm (was 2.0m - too loose!)
    public static final int MIN_TAG_COUNT_FOR_MULTI = 2;

    // Standard deviations for pose estimation (lower = more trust)
    // TRUST HIERARCHY: Multi-tag vision > Single-tag vision > QuestNav > Odometry
    
    // Limelight 3 (BEST - pre-calibrated from factory)
    public static final double[] LIMELIGHT_MULTI_TAG_STD_DEVS = {0.03, 0.03, 0.05};   // HIGHEST TRUST (3cm!)
    public static final double[] LIMELIGHT_SINGLE_TAG_STD_DEVS = {0.1, 0.1, 0.15};     // HIGH TRUST
    
    // PhotonVision cameras (GOOD - calibrated by us)
    public static final double[] PHOTON_MULTI_TAG_STD_DEVS = {0.05, 0.05, 0.1};   // VERY HIGH TRUST (5cm)
    public static final double[] PHOTON_SINGLE_TAG_STD_DEVS = {0.15, 0.15, 0.25}; // MEDIUM TRUST
    
    // Legacy constants (deprecated)
    @Deprecated
    public static final double[] VISION_STD_DEVS_MULTI_TAG = PHOTON_MULTI_TAG_STD_DEVS;
    @Deprecated
    public static final double[] VISION_STD_DEVS_SINGLE_TAG = PHOTON_SINGLE_TAG_STD_DEVS;
    
    public static final double[] ODOMETRY_STD_DEVS = {1.0, 1.0, 1.0}; // LOWEST TRUST

    // ===================================================================
    // LIMELIGHT 3 - FRONT CAMERA
    // ===================================================================
    public static final double FRONT_LL_X_METERS = 0.3;
    public static final double FRONT_LL_Y_METERS = 0.0;
    public static final double FRONT_LL_Z_METERS = 0.2;
    public static final double FRONT_LL_ROLL_DEG = 0.0;
    public static final double FRONT_LL_PITCH_DEG = 10;  // CHANGED: Tilted back 7.5° looking up (was 0.0)
    public static final double FRONT_LL_YAW_DEG = 0.0;

    // ===================================================================
    // PHOTONVISION - BACK LEFT TAG CAMERA (RLTagPV)
    // ===================================================================
    // Back 11.5", Left 10", Height 8"
    // MOUNTED ON BACK but POINTING FORWARD-LEFT (add 180° to forward angle)
    // Tilted backward 15° to see tags above robot
    public static final double BACK_LEFT_PV_X_METERS = Units.inchesToMeters(-11.5);
    public static final double BACK_LEFT_PV_Y_METERS = Units.inchesToMeters(10.0);
    public static final double BACK_LEFT_PV_Z_METERS = Units.inchesToMeters(8.0);
    public static final double BACK_LEFT_PV_ROLL_DEG = 0.0;
    public static final double BACK_LEFT_PV_PITCH_DEG = 15.0;  // FIXED: Tilted backward to see up (was 0.0)
    public static final double BACK_LEFT_PV_YAW_DEG = 135.0;  // Mounted on back, pointing forward-left
    public static final double BACK_LEFT_PV_FOV_DEG = 100.0;

    // ===================================================================
    // PHOTONVISION - BACK RIGHT TAG CAMERA (RRTagPV)
    // ===================================================================
    // Back 11.5", Right 10", Height 8"
    // MOUNTED ON BACK but POINTING FORWARD-RIGHT (add 180° to forward angle)
    // Tilted backward 15° to see tags above robot
    public static final double BACK_RIGHT_PV_X_METERS = Units.inchesToMeters(-11.5);
    public static final double BACK_RIGHT_PV_Y_METERS = Units.inchesToMeters(-10.0);
    public static final double BACK_RIGHT_PV_Z_METERS = Units.inchesToMeters(8.0);
    public static final double BACK_RIGHT_PV_ROLL_DEG = 0.0;
    public static final double BACK_RIGHT_PV_PITCH_DEG = 15.0;  // Tilted backward to see up 
    public static final double BACK_RIGHT_PV_YAW_DEG = 225.0;  // Mounted on back, pointing forward-right
    public static final double BACK_RIGHT_PV_FOV_DEG = 100.0;

    // ===================================================================
    // PHOTONVISION - FRONT OBJECT DETECTION CAMERA (FObjPV)
    // ===================================================================
    public static final double OBJ_CAMERA_X_METERS = Units.inchesToMeters(12.0);
    public static final double OBJ_CAMERA_Y_METERS = Units.inchesToMeters(5.5);
    public static final double OBJ_CAMERA_Z_METERS = Units.inchesToMeters(9.75);
    public static final double OBJ_CAMERA_ROLL_DEG = 0.0;
    public static final double OBJ_CAMERA_PITCH_DEG = 0.0;
    public static final double OBJ_CAMERA_YAW_DEG = 0.0;
    public static final double OBJ_CAMERA_FOV_DEG = 120.0;  // OV9782 has ~120° diagonal FOV
    
    // 2025 game piece heights (temporary for testing until 2026 game)
    public static final double CORAL_HEIGHT_METERS = Units.inchesToMeters(6.0);
    public static final double ALGAE_HEIGHT_METERS = Units.inchesToMeters(3.0);
    
    // Detection filtering
    public static final double MIN_TARGET_AREA_PERCENT = 0.1;
    public static final double MAX_TARGET_DISTANCE_METERS = 5.0;
  }

  public static final class QuestNav {
    // QuestNav mounting position relative to robot center
    // Robot coordinate system: +X = forward, +Y = left, +Z = up
    public static final double QUEST_X_METERS = Units.inchesToMeters(14.0);   // 14" forward
    public static final double QUEST_Y_METERS = Units.inchesToMeters(0.5);    // 0.5" left
    public static final double QUEST_Z_METERS = Units.inchesToMeters(0.0);    // On ground level
    public static final double QUEST_YAW_DEG = 0.0;  // Facing forward

    // Failover settings
    public static final double MAX_QUESTNAV_DISCONNECT_TIME_SECONDS = 0.5;  // Switch to Pigeon after 0.5s
    public static final double MAX_ANGULAR_RATE_DEG_PER_SEC = 720.0;  // Reject unrealistic rates
    
    // Trust settings (for future pose estimation)
    // QuestNav trust - BELOW vision (SLAM < physical measurements)
    // Still better than odometry, but not as good as AprilTags
    public static final double[] QUESTNAV_STD_DEVS = {
        0.08,  // was larger (e.g., 0.10–0.12)
        0.08,
        0.07   // radians (~4 deg)
    };

    // ===== LATENCY COMPENSATION =====
    // MEASURED: ActualAge averages 2-5ms (Quest frames arrive nearly instantly via USB/Ethernet)
    // Quest SLAM processing happens BEFORE timestamp, so no additional compensation needed
    public static final double QUESTNAV_LATENCY_MS = 5.0; // Was 60ms - FIXED based on ActualAge measurements
    
    // ===== VELOCITY GATING (PERMISSIVE - Rely on timestamp validation instead) =====
    // Accept QuestNav during most motion, let timestamp validation reject stale data
    public static final double QUESTNAV_MAX_LINEAR_SPEED_MPS = 3.0;    // Accept up to moderate speed (was 0.05)
    public static final double QUESTNAV_MAX_OMEGA_RAD_PER_SEC = 2.0;   // Accept during most turns (was 0.05)
    
    // ===== TRUST LEVELS (Standard Deviations) =====
    // Lower value = higher trust
    
    // When robot is completely stopped (used for post-path correction)
    public static final double[] QUESTNAV_STD_DEVS_STOPPED = {
        0.02,  // X: 2cm (high trust - QuestNav excels when stationary)
        0.02,  // Y: 2cm (high trust)
        0.03   // Theta: ~2 deg (high trust - QuestNav theta is excellent when stopped)
    };
    
    // Initial alignment at auto start (very high trust - one-time)
    public static final double[] QUESTNAV_STD_DEVS_INITIAL = {
        0.01,  // X: 1cm (very high trust)
        0.01,  // Y: 1cm (very high trust)
        0.02   // Theta: ~1 deg (very high trust - QuestNav is best static theta source)
    };
    
    // ===== POST-PATH CORRECTION =====
    public static final double POST_PATH_CORRECTION_THRESHOLD_M = 0.03;   // 3cm - trigger correction
    public static final double POST_PATH_CORRECTION_MAX_M = 0.15;         // 15cm - don't correct if too far off
    public static final double POST_PATH_CORRECTION_TIMEOUT_S = 1.0;      // 1s max for correction move
    public static final double POST_PATH_SETTLE_TIME_S = 0.5;             // Wait 0.5s after path for settling
  }

  public static final class Auto {
    // PathPlanner PID constants for holonomic drive controller
    
    // Slightly sharper translation, a bit less damping
    public static final double TRANSLATION_KP = 8;  // was 1.2
    public static final double TRANSLATION_KI = 0.0;
    public static final double TRANSLATION_KD = 0.15; // was 0.25
    
    // Keep rotation where it is for now
    public static final double ROTATION_KP = 9.0;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.2;
    
    // Path following constraints (should match Swerve max speeds)
    public static final double MAX_MODULE_SPEED_MPS = Swerve.MAX_TRANSLATION_SPEED_MPS;
    
    // Starting pose validation tolerance
    public static final double STARTING_POSE_TOLERANCE_METERS = 0.15;  // 15cm
    public static final double STARTING_POSE_TOLERANCE_DEGREES = 5.0;   // 5 degrees

    // Vision initialization timeout
    public static final double VISION_INITIALIZATION_TIMEOUT_SECONDS = 7.0;
    
    // Default fallback position if vision never initializes
    // Uses Blue Reef Tag 17 as safe default
    public static final Pose2d DEFAULT_FALLBACK_POSE = StartingPositions.BLUE_REEF_TAG_17;
  }

  public static final class StartingPositions {
    // Blue alliance starting positions (all in meters, degrees)
    // Updated to match PathPlanner GUI STAGING POSES (Phase 1 pathfind targets)
    
    /** Blue Reef Back - Tag 17 - Staging pose from Stage17toPrecise17 */
    public static final Pose2d BLUE_REEF_TAG_17 = new Pose2d(
        3.359, 2.077, Rotation2d.fromDegrees(60.0));
    
    /** Blue Reef Right - Tag 18 - Staging pose from Stage18toPrecise18 */
    public static final Pose2d BLUE_REEF_TAG_18 = new Pose2d(
        2.070, 4.027, Rotation2d.fromDegrees(0.0));
    
    /** Blue Reef Tag 21 - Staging pose from Stage21toPrecise21 */
    public static final Pose2d BLUE_REEF_TAG_21 = new Pose2d(
        6.889, 4.025, Rotation2d.fromDegrees(180.0));
    
    /** Blue Reef Tag 22 - Staging pose from Stage22toPrecise22 */
    public static final Pose2d BLUE_REEF_TAG_22 = new Pose2d(
        5.552, 2.168, Rotation2d.fromDegrees(120.0));
    
    /** Blue Processor Station - Tag 16 - Staging pose from Stage16toPrecise16 */
    public static final Pose2d BLUE_TAG_16 = new Pose2d(
        5.990, 1.543, Rotation2d.fromDegrees(-90.0));
    
    /** Blue Coral Station - Tag 12 - Staging pose from Stage12toPrecise12 */
    public static final Pose2d BLUE_TAG_12 = new Pose2d(
        2.300, 1.850, Rotation2d.fromDegrees(-130.0));

    /** Auto Start Position Blue far right - Staging pose from StageAutoRightToPreciseAutoRight */
    public static final Pose2d BLUE_AUTO_START_POS_FAR_RIGHT = new Pose2d(
        6.033, 0.985, Rotation2d.fromDegrees(180.0));

    /** PID tuning Position */
    public static final Pose2d PID_TUNING_POSITION = new Pose2d(1.270, 2.230, Rotation2d.fromDegrees(0.0));
    public static final Pose2d PID_TUNING_POSITIONLeft = new Pose2d(1.270, 2.230, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d PID_TUNING_POSITIONRight = new Pose2d(1.270, 2.230, Rotation2d.fromDegrees(90.0));
    
    // ===== PRECISION TARGET POSES (AutoPilot Phase 2 final targets) =====
    // These are the END poses extracted from PathPlanner precision paths
    
    /** Precise12 - Final target from Stage12toPrecise12.path */
    public static final Pose2d PRECISE_12_POSE = new Pose2d(
        1.35, 1.07, Rotation2d.fromDegrees(-130.0));
    
    /** Precise16 - Final target from Stage16toPrecise16.path */
    public static final Pose2d PRECISE_16_POSE = new Pose2d(
        5.99, 0.56, Rotation2d.fromDegrees(-90.0));
    
    /** Precise17 - Final target from Stage17toPrecise17.path */
    public static final Pose2d PRECISE_17_POSE = new Pose2d(
        3.92, 2.9, Rotation2d.fromDegrees(60.0));
    
    // Red alliance positions (mirrored from blue)
    // PathPlanner will handle flipping automatically if needed
  }
}