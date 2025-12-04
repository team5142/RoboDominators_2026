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
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI * 2.0;

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

    // Module CAN IDs and offsets from Tuner X - UPDATED from latest TunerConstants
    // Front Left
    public static final int FRONT_LEFT_DRIVE_ID = 2;
    public static final int FRONT_LEFT_STEER_ID = 1;
    public static final int FRONT_LEFT_CANCODER_ID = 9;
    public static final double FRONT_LEFT_OFFSET_ROTATIONS = 0.484130859375;  //  From Tuner X

    // Front Right
    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_STEER_ID = 3;
    public static final int FRONT_RIGHT_CANCODER_ID = 10;
    public static final double FRONT_RIGHT_OFFSET_ROTATIONS = 0.20703125;  //  UPDATED (was 0.21533203125)

    // Back Left
    public static final int BACK_LEFT_DRIVE_ID = 8;
    public static final int BACK_LEFT_STEER_ID = 7;
    public static final int BACK_LEFT_CANCODER_ID = 12;
    public static final double BACK_LEFT_OFFSET_ROTATIONS = -0.30908203125;  //  From Tuner X

    // Back Right
    public static final int BACK_RIGHT_DRIVE_ID = 6;
    public static final int BACK_RIGHT_STEER_ID = 5;
    public static final int BACK_RIGHT_CANCODER_ID = 11;
    public static final double BACK_RIGHT_OFFSET_ROTATIONS = 0.396240234375;  //  UPDATED (was 0.389892578125)

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

    // PID Gains from Tuner X
    public static final class SteerGains {
      // BEFORE (twitchy at slow speeds):
      // public static final double kP = 100.0;
      // public static final double kD = 0.5;
      
      // AFTER (smoother, less twitchy):
      public static final double kP = 50.0;  // REDUCED from 100.0 - less aggressive steering correction
      public static final double kI = 0.0;
      public static final double kD = 1.0;   // INCREASED from 0.5 - more damping to prevent oscillation
      public static final double kS = 0.1;
      public static final double kV = 2.66;
      public static final double kA = 0.0;
    }

    public static final class DriveGains {
      public static final double kP = 0.1;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kS = 0.0;
      public static final double kV = 0.124;
    }

    // Control output types
    public static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    public static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    // Feedback type
    public static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    /**
     * Creates a standard TalonFX configuration for drive motors
     */
    public static TalonFXConfiguration createDriveMotorConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      
      config.Slot0.kP = DriveGains.kP;
      config.Slot0.kI = DriveGains.kI;
      config.Slot0.kD = DriveGains.kD;
      config.Slot0.kS = DriveGains.kS;
      config.Slot0.kV = DriveGains.kV;

      config.ClosedLoopGeneral.ContinuousWrap = false;
      
      return config;
    }

    /**
     * Creates a standard TalonFX configuration for steer motors
     */
    public static TalonFXConfiguration createSteerMotorConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      
      config.Slot0.kP = SteerGains.kP;
      config.Slot0.kI = SteerGains.kI;
      config.Slot0.kD = SteerGains.kD;
      config.Slot0.kS = SteerGains.kS;
      config.Slot0.kV = SteerGains.kV;
      config.Slot0.kA = SteerGains.kA;
      config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      config.ClosedLoopGeneral.ContinuousWrap = true;

      // Current limiting for steer motors
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
    public static final double[] QUESTNAV_STD_DEVS = {0.12, 0.12, 0.2};  // MEDIUM-HIGH TRUST
  }

  public static final class Auto {
    // PathPlanner PID constants for holonomic drive controller
    
    // CHANGED: Further reduce gains - 1.5 was still too aggressive
    public static final double TRANSLATION_KP = 0.8;  // REDUCED from 1.5 - much gentler
    public static final double TRANSLATION_KI = 0.0;
    public static final double TRANSLATION_KD = 0.3;  // INCREASED from 0.2 - more damping
    
    public static final double ROTATION_KP = 0.8;     // REDUCED from 1.5
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.3;     // INCREASED from 0.2
    
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
    
    /** Blue Reef Back - Tag 17 - Centered, 1" from reef wall */
    public static final Pose2d BLUE_REEF_TAG_17 = new Pose2d(
        3.84, 3.18, Rotation2d.fromDegrees(60.0));  // CORRECTED from 60° (was 180° wrong!)
    
    /** Blue Reef Right - Tag 18 - 1" from wall, 30mm Y buffer */
    public static final Pose2d BLUE_REEF_TAG_18 = new Pose2d(
        3.318, 3.996, Rotation2d.fromDegrees(0.0));  // CORRECTED: Facing 0° (downfield), not 180°
    
    /** Blue Reef Tag 21 - Top of reef */
    public static final Pose2d BLUE_REEF_TAG_21 = new Pose2d(5.661, 4.026, Rotation2d.fromDegrees(180.0));
    
    /** Blue Reef Tag 22 - Top right of reef */
    public static final Pose2d BLUE_REEF_TAG_22 = new Pose2d(5.075, 2.896, Rotation2d.fromDegrees(120.0));
    
    /** Blue Processor Station - Tag 16 */
    public static final Pose2d BLUE_TAG_16 = new Pose2d(5.87, 0.31, Rotation2d.fromDegrees(-90.0));
    
    /** Blue Coral Station - Tag 12 (same as former INTAKE_POS) */
    public static final Pose2d BLUE_TAG_12 = new Pose2d(1.0, .07, Rotation2d.fromDegrees(-130.0));

    /** Auto Start Position Blue far right */
    public static final Pose2d BLUE_AUTO_START_POS_FAR_RIGHT = new Pose2d(6.78, .43, Rotation2d.fromDegrees(180.0));

    
    
    // Red alliance positions (mirrored from blue)
    // PathPlanner will handle flipping automatically if needed  }

  }
}