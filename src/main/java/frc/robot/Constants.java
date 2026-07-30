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

public final class Constants {
  private Constants() {}

  public static final int TEAM_NUMBER = 5142;
  public static final int DRIVER_CONTROLLER_PORT   = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1; // Operator USB slot, one above driver
  public static final int BLINKIN_PWM_PORT = 0; // Change to your actual PWM port

  // Swerve drivetrain hardware config and PID tuning
  public static final class Swerve {
    // Robot dimensions (from Tuner X)
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(19.5);
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(19.5);
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    // Max speeds
    public static final double MAX_TRANSLATION_SPEED_MPS = 5.21;
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI * 4.0;

    // Driver speed modes (multiply by max speed)
    public static final double NORMAL_SPEED_SCALE         = 0.65; // 5.21 * 0.50 = ~2.6 m/s
    public static final double NORMAL_ROTATION_SCALE      = 0.30; // 4pi  * 0.30 = ~3.8 rad/s
    public static final double PRECISION_SPEED_SCALE      = 0.20; // 5.21 * 0.20 = ~1.0 m/s
    public static final double PRECISION_ROTATION_SCALE   = 0.10;
    public static final double FAST_SPEED_SCALE           = 1.0;
    public static final double JOYSTICK_DEADBAND     = 0.05; // slightly larger to reduce touchiness

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
    public static final double FRONT_LEFT_OFFSET_ROTATIONS = 0.488037109375;

    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_STEER_ID = 3;
    public static final int FRONT_RIGHT_CANCODER_ID = 10;
    public static final double FRONT_RIGHT_OFFSET_ROTATIONS = 0.209716796875;

    public static final int BACK_LEFT_DRIVE_ID = 8;
    public static final int BACK_LEFT_STEER_ID = 7;
    public static final int BACK_LEFT_CANCODER_ID = 12;
    public static final double BACK_LEFT_OFFSET_ROTATIONS = -0.3017578125;

    public static final int BACK_RIGHT_DRIVE_ID = 6;
    public static final int BACK_RIGHT_STEER_ID = 5;
    public static final int BACK_RIGHT_CANCODER_ID = 11;
    public static final double BACK_RIGHT_OFFSET_ROTATIONS = 0.395263671875;

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
      public static final double kP = 25.601;
      public static final double kI = 0.0;
      public static final double kD = 0.62218;
      public static final double kS = 0.12222;
      public static final double kV = 0;
      public static final double kA = 0;
    }

    public static final class DriveGains {
      public static final double kP = 0.13816;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kS = 0.17978;//0.17399;
      public static final double kV = 0.11294;//0.11894;
      public static final double kA = 0.0017;//0.003069;
    }

    public static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    public static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    public static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    // Motor configuration helpers (used by TunerConstants)
    public static TalonFXConfiguration createDriveMotorConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = DriveGains.kP;
      config.Slot0.kI = DriveGains.kI;
      config.Slot0.kD = DriveGains.kD;
      config.Slot0.kS = DriveGains.kS;
      config.Slot0.kV = DriveGains.kV;
      config.Slot0.kA = DriveGains.kA;
      config.ClosedLoopGeneral.ContinuousWrap = false;
      // Stator limit caps torque/heat inside the motor.
      // Supply limit caps battery draw — 4 motors x 35A = 140A max drive draw (reduced from 160A to help brownouts).
      config.CurrentLimits.StatorCurrentLimit = 60.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = 35.0; // reduced from 40 — 140A combined vs 160A
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      // Open-loop ramp: 100ms to full speed — staggers current rise across all 4 motors on hard acceleration.
      // Barely perceptible to driver but significantly reduces simultaneous current spike.
      config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
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
    
    // Game piece height for object detection distance calculation (update when 2026 game piece is known)
    public static final double GAME_PIECE_HEIGHT_METERS = Units.inchesToMeters(6.0);
    
    public static final double MIN_TARGET_AREA_PERCENT = 0.1;
    public static final double MAX_TARGET_DISTANCE_METERS = 5.0;
  }

  // QuestNav SLAM sensor (USB/Ethernet connected IMU with visual odometry)
  public static final class QuestNav {
    // Measured 2026-03-02: 10.79in back, 11.88in left, 20.28in up. Facing backward (yaw 180).
    public static final double QUEST_X_METERS = Units.inchesToMeters(-10.79);
    public static final double QUEST_Y_METERS = Units.inchesToMeters(10.88);
    public static final double QUEST_Z_METERS = Units.inchesToMeters(20.28);
    public static final double QUEST_YAW_DEG = 180.0;

    // Failover timing (switch to Pigeon if QuestNav disconnects)
    public static final double MAX_QUESTNAV_DISCONNECT_TIME_SECONDS = 0.5;
    public static final double MAX_ANGULAR_RATE_DEG_PER_SEC = 720.0;
    
    // Trust levels (lower = more trust) - Like AprilTag vision, NOT odometry
    public static final double[] QUESTNAV_STD_DEVS = {0.08, 0.08, 0.07}; // Moving
    public static final double QUESTNAV_LATENCY_MS = 5.0;
    
    // 1) VELOCITY GATING (enforce "stopped robot" requirement)
    // Reject fusion during motion to prevent fighting PathPlanner
    public static final double MAX_LINEAR_SPEED_FOR_FUSION_MPS = 1.8; // ~5 in/s
    // Raised from 1.5 -> 3.0 rad/s (~86->172 deg/s) to allow fusion during moderate turns.
    // Revert to 1.5 if Quest causes estimator fighting during PathPlanner rotation.
    public static final double MAX_ANGULAR_SPEED_FOR_FUSION_RAD_PER_SEC = 3.0; // ~172 deg/s
    
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
    // Raised from 0.9 -> 1.4 rad (~52->80 deg) to stop falsely flagging Quest during fast spins.
    // Revert to 0.9 if teleport detection misses real tracking glitches.
    public static final double TELEPORT_ROTATION_RADIANS = 1.4; // ~80° jump in one frame

  // Teleport gate option: only enforce when robot is moving fast
  public static final boolean TELEPORT_GATE_ONLY_WHEN_MOVING = true;

  // Teleport gate option: skip when robot is tilted on the ramp
  public static final double TELEPORT_GATE_MAX_TILT_DEGREES = 8.0;
    
    // Divergence detection (Quest vs Odom comparison)
    public static final double DIVERGENCE_THRESHOLD_METERS = 0.20; // 20cm disagreement per cycle
    // Raised from 15 -> 25 cycles to avoid false DEGRADED transitions during fast turns.
    // Revert to 15 if divergence detection feels too slow to catch real Quest drift.
    public static final int DIVERGENCE_PATIENCE_CYCLES = 25; // 25 bad cycles = 500ms @ 50Hz
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
    // Reduced from 1.5 -> 1.2 to reduce theta penalty during moderate rotation.
    // Revert to 1.5 if Quest measurements feel too "sticky" when moving.
    public static final double MOVING_TRUST_DEGRADATION_FACTOR = 1.2; // 1.2x std dev when moving fast
    public static final double DEGRADED_TRUST_FACTOR = 5.0; // 5x std dev when DEGRADED
    public static final double UNHEALTHY_TRUST_FACTOR = 50.0; // 50x std dev when UNHEALTHY (almost no influence)

    // Recovery from UNHEALTHY: if Quest is self-stable while stopped, snap estimator back to Quest.
    // Samples UNHEALTHY_RECOVERY_SAMPLES frames; if max variance < threshold, force-accept Quest pose.
    public static final int UNHEALTHY_RECOVERY_SAMPLES = 10;   // ~200ms of samples at 50Hz
    public static final double UNHEALTHY_RECOVERY_VARIANCE_METERS = 0.05; // Quest must be stable within 5cm

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

  // Turret mechanism hardware IDs and tuning constants
  public static final class Turret {
    public static final int FLYWHEEL_FRONT_MOTOR_ID = 20;
    public static final int FLYWHEEL_BACK_MOTOR_ID  = 21;
    public static final int HOOD_MOTOR_ID           = 22;
    public static final int TURRET_MOTOR_ID         = 23; // Kraken X44

    public static final int HOOD_LIMIT_SWITCH_DIO = 2;
    public static final int HALL_SENSOR_CCW_DIO   = 0; // CCW hard stop, primary home sensor

    // Motor inversions — confirmed on hardware
    public static final boolean TURRET_MOTOR_INVERTED         = false;
    public static final boolean HOOD_MOTOR_INVERTED           = true;
    public static final boolean FLYWHEEL_FRONT_MOTOR_INVERTED = true;
    public static final boolean FLYWHEEL_BACK_MOTOR_INVERTED  = true;

    // Flywheel velocity gains
    public static final double FLYWHEEL_FRONT_KS = 1.5;
    public static final double FLYWHEEL_FRONT_KV = 0.0975;
    public static final double FLYWHEEL_FRONT_KP = 0.03;
    public static final double FLYWHEEL_FRONT_KA = 0.0;
    public static final double FLYWHEEL_BACK_KS  = 1.0;
    public static final double FLYWHEEL_BACK_KV  = 0.108;
    public static final double FLYWHEEL_BACK_KP  = 0.005;
    public static final double FLYWHEEL_BACK_KD  = 0.002;
    public static final double FLYWHEEL_BACK_KA  = 0.0;

    // Turret MotionMagic gains
    public static final double TURRET_KS = 0.9;
    public static final double TURRET_KV = 0.12;
    public static final double TURRET_KP = 9.0;
    public static final double TURRET_KI = 2.0;
    public static final double TURRET_KD = 0.3;
    public static final double TURRET_CRUISE_VELOCITY_RPS = 15.0;
    public static final double TURRET_ACCELERATION_RPS2   = 30.0;
    public static final double TURRET_JERK_RPS3           = 200.0;

    // Turret homing constants
    public static final double TURRET_HOME_SPEED_FAST_PERCENT      = 0.06;
    public static final int    TURRET_HALL_DEBOUNCE_LOOPS          = 3;
    public static final double TURRET_HOMING_STALL_CURRENT_AMPS    = 28.0;
    public static final int    TURRET_HOMING_STALL_LOOP_THRESHOLD  = 10;

    // Turret soft limits and known positions (motor rotations)
    public static final double TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT  =  0.05;
    public static final double TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT =  9.46;
    // Motor encoder value when turret faces straight forward
    public static final double TURRET_FORWARD_MOTOR_ROT = 6.077637;
    public static final double TURRET_GEAR_RATIO = 10.0; // motor rot / turret rot

    // Hood MotionMagic gains
    public static final double HOOD_KS                  = 0.4;
    public static final double HOOD_KV                  = 0.12;
    public static final double HOOD_KP                  = 14.0;
    public static final double HOOD_CRUISE_VELOCITY_RPS = 9.0;
    public static final double HOOD_ACCELERATION_RPS2   = 30.0;

    // Hood homing constants
    public static final double HOOD_HOME_SPEED_PERCENT          = 0.10;
    public static final double HOOD_HOME_ROTATIONS              = 0.0;
    public static final double HOOD_HOME_BACKOFF_ROTATIONS      = 0.05;
    public static final double HOOD_HOMING_STALL_CURRENT_AMPS   = 10.0;
    public static final int    HOOD_HOMING_STALL_LOOP_THRESHOLD = 6;
    public static final double HOOD_SOFT_LIMIT_TOP_ROTATIONS    = 4.69;

    // Minimum RPM both flywheels must reach before shooting is allowed
    public static final double FLYWHEEL_SPINUP_MIN_RPM = 1500.0;
  }

  // Singulator hardware IDs and tuning constants (feeds balls one at a time into flywheels)
  public static final class Singulator {
    public static final int MOTOR_ID = 24; // REV NEO on SparkMax
    public static final int LASERCAN_ID = 28; // LaserCAN — ball present at singulator staging point
    public static final int DEAD_ZONE_LASERCAN_ID = 29; // LaserCAN — ball stuck above singulator, below flywheels
    // Ball is considered present when LaserCAN reads closer than this distance.
    // Channel is ~7-8in wide, ball is 6in diameter. Sensor shoots across the channel (~200mm).
    // 150mm catches the ball anywhere in the inner half without false-triggering on the far wall.
    public static final int LASERCAN_THRESHOLD_MM = 150; // TODO: verify on hardware
    public static final int DEAD_ZONE_LASERCAN_THRESHOLD_MM = 150; // TODO: verify on hardware

    // Minimum time between shots in auto mode — prevents back-to-back balls from interfering.
    public static final double AUTO_SHOT_GAP_SECS = 0.35; // TODO: tune on hardware
    // How long to reverse the singulator when attempting dead zone recovery.
    public static final double DEAD_ZONE_REVERSE_SECS = 0.40; // TODO: tune on hardware
    // How long before dead zone detection triggers recovery — shortened from 1.0s to react faster.
    // Previous value was 1.0s; 0.3s catches the jam before the next ball piles up behind it.
    public static final double DEAD_ZONE_DETECT_SECS = 0.30;
    // How long to push forward before giving up on dead zone clearance.
    public static final double DEAD_ZONE_PUSH_TIMEOUT_SECS = 1.0;

    // Feed recovery: alternates spindexer reverse and intake agitate every 4s when no ball
    // is seen at the staging beam break. Disabled until beam breaks are fully commissioned.
    public static final boolean ENABLE_AUTO_FEED_RECOVERY = false;
    public static final double  FEED_RECOVERY_INTERVAL_SECS = 4.0;  // time between each action
    public static final double  FEED_RECOVERY_PULSE_SECS    = 0.5;  // duration of each pulse
    public static final double  FEED_RECOVERY_GIVE_UP_SECS  = 32.0; // jam after this long

    // TODO: flip to true if motor runs backwards on first test
    public static final boolean MOTOR_INVERTED = true;

    public static final double FEED_SPEED    =  1.00; // raised from 0.85 (2026-03-11)
    public static final double REVERSE_SPEED = -0.85;

    // Pre-feed prime: briefly reverse before feeding to pull the ball back into compression.
    // The ball sits in a deadzone at the turret plane where the wheel barely contacts it —
    // reversing for ~0.1s pulls it back so the forward feed has traction from the start.
    // TODO: tune PRIME_REVERSE_SECS on hardware (start at 0.10, raise if ball slips on feed)
    public static final double PRIME_REVERSE_SECS = 0.10;

    // SparkMax smart current limit (stall protection)
    public static final int CURRENT_LIMIT_AMPS = 30;
  }

  /*
   * TASK 12a - Read the Spindexer Constants
   * -----------------------------------------------------------------------
   * Look through the Spindexer class below and understand what each constant
   * controls. Notice the pattern: hardware IDs at the top, booleans for config,
   * then speeds, then loop-based timing values.
   *
   * Constants exist so you can tune the robot from one place instead of
   * hunting through subsystem code. If you want the spindexer faster,
   * you change FORWARD_SPEED here - not in the subsystem.
   *
  * Nothing to write here. When done: move to Task 12b in SpindexerSubsystem.java.
   * -----------------------------------------------------------------------
   */

  /*
   * TASK 13a - Add Speed Constants for the Singulator
   * -----------------------------------------------------------------------
   * The Singulator class below has MOTOR_ID, LASERCAN_ID, MOTOR_INVERTED,
   * and CURRENT_LIMIT_AMPS already defined.
   *
   * It is missing the speed constants that SingulatorSubsystem.spinFeed()
   * and spinReverse() will need. Add them to the Singulator class:
   *
   *   FEED_SPEED    - a positive value between 0.0 and 1.0 (try 0.8 to start)
   *   REVERSE_SPEED - a negative value (try -0.5 to start)
   *
   * Look at how Spindexer defines FORWARD_SPEED and REVERSE_SPEED for the format.
   * Write the constants yourself - do not copy from Spindexer.
   *
  * When done: compile and move to Task 13b in SpindexerSubsystem.java.
   * -----------------------------------------------------------------------
   */

  // Spindexer hardware IDs and tuning constants (cone spinner that feeds balls into singulator)
  public static final class Spindexer {
    public static final int MOTOR_ID = 27; // REV NEO on SparkMax

    // TODO: flip to true if motor runs backwards on first test
    public static final boolean MOTOR_INVERTED = true;

    public static final double FORWARD_SPEED = 1.0;
    // Full reverse on agitate — maximum force to break jam in minimum time.
    // Previous value was -0.50 (conservative placeholder).
    public static final double REVERSE_SPEED = -0.6;

    // Agitator auto-reverse: current-based detection (replaces velocity-based).
    // Current spikes immediately when balls pile on; velocity was too slow to react.
    // If current exceeds LOAD_CURRENT_AMPS for AGITATE_LOOP_THRESHOLD consecutive loops,
    // fire a short full-reverse pulse, then resume forward.
    // Previous: velocity < STALL_VELOCITY_RPS for 15 loops (~300ms detection, 200ms pulse).
    // Now: current > LOAD_CURRENT_AMPS for 5 loops (~100ms detection, 100ms pulse).
    public static final int    AGITATE_LOOP_THRESHOLD = 5;    // ~100ms sustained overload before agitating
    public static final int    AGITATE_PULSE_LOOPS    = 5;    // ~100ms full-reverse pulse

    // SparkMax smart current limit (stall protection)
    public static final int CURRENT_LIMIT_AMPS = 45; // raised from 30 — gives NEO more torque headroom under load
    // Jam detection threshold — tune up if agitating during normal ball load, down if jams go undetected.
    // Watch Spindexer/CurrentAmps in AdvantageScope: normal load ~10-20A, jam should spike well above.
    public static final double LOAD_CURRENT_AMPS = 28.0; // tune on hardware — previous placeholder was 15.0
  }

  // Intake mechanism hardware IDs and tuning constants
  public static final class Intake {
    // Hopper test mode: skips homing and arm movement entirely.
    // Assumes the arm is already physically down. Only rollers and roller-agitation work.
    // Set false for normal match operation.
    public static final boolean HOPPER_TEST_MODE = false; // normal operation — homing runs on first enable

    public static final int INTAKE_ROLLER_MOTOR_ID         = 40; // NEO 500 - roller spin (SparkMax)
    public static final int INTAKE_EXTENSION_MOTOR_ID_LEFT  = 41; // Kraken X44 - left extension motor
    public static final int INTAKE_EXTENSION_MOTOR_ID_RIGHT = 42; // Kraken X44 - right extension motor (runs opposite to left)

    // Left motor: forward = extend. Right motor: forward = retract (set inverted in code via negation).
    public static final boolean EXTENSION_MOTOR_INVERTED = false;
    public static final boolean ROLLER_MOTOR_INVERTED    = true;

    // RoboRIO DIO port for retract limit switch (single switch — confirmed DIO 1)
    public static final int RETRACT_LIMIT_SWITCH_DIO = 1;

    // Extension arm positions — measured via Phoenix Tuner with both motors wired.
    // Left motor (ID 41) is used as primary encoder for all position logic.
    // Right motor (ID 42) mirrors but its encoder is only used for diagnostics.
    // IN  = arm fully retracted: left=0.292, right=0.369
    // OUT = arm fully extended:  left=8.362, right=8.691
    public static final double EXTENSION_HOME_ROTATIONS          = 0.292; // left motor at full retract
    public static final double EXTENSION_HOME_ROTATIONS_RIGHT    = 0.369; // right motor at full retract (diagnostics only)
    public static final double EXTENSION_TARGET_ROTATIONS        = 8.362; // left motor at full extend
    public static final double EXTENSION_TARGET_ROTATIONS_RIGHT  = 8.691; // right motor at full extend (diagnostics only)
    public static final double AGITATE_RETRACT_ROTATIONS  = 2.0;  // scaled down from 3.5 to match new travel range
    public static final double BUMP_LIFT_ROTATIONS        = EXTENSION_TARGET_ROTATIONS - (AGITATE_RETRACT_ROTATIONS / 6.0);

    // Duty cycle outputs — reduced for initial testing with new dual-motor setup.
    // Raise EXTEND_SPEED / RETRACT_SPEED gradually once direction and limits are confirmed.
    public static final double EXTEND_SPEED  =  0.25;
    public static final double RETRACT_SPEED = -0.25;

    // Two-speed approach: slow down when within this many rotations of the target.
    public static final double EXTEND_SLOW_SPEED           =  0.05;
    public static final double EXTEND_SLOW_ZONE_ROTATIONS  =  1.0;
    public static final double RETRACT_SLOW_SPEED          = -0.05;
    public static final double RETRACT_SLOW_ZONE_ROTATIONS =  1.0;

    // Limit switch sanity window — only trust the switch if the encoder is within this many
    // rotations of home. Ignores a stuck-ON switch when the arm is clearly still extended.
    public static final double LIMIT_SWITCH_VALID_WINDOW_ROTATIONS = 1.5;

    // Roller duty cycle outputs
    public static final double ROLLER_INTAKE_SPEED  =  1.00; // intaking
    public static final double ROLLER_REVERSE_SPEED = -0.50; // ejecting

    // Current limits
    // Extension (Kraken x2): stator caps torque current; supply caps battery draw.
    // Two motors x 15A supply = 30A max combined — arm is fast enough well below this.
    // Previous supply limit was 30A per motor (60A combined) which contributed to brownouts
    // when both motors spiked simultaneously during hard drivetrain acceleration.
    // Roller (NEO SparkMax): uses smart current limit only.
    public static final double EXTENSION_STATOR_LIMIT_AMPS = 40.0;
    public static final double EXTENSION_SUPPLY_LIMIT_AMPS = 15.0; // reduced from 30 — arm is still very fast
    public static final int    ROLLER_CURRENT_LIMIT_AMPS   = 40;

    // Stall detection via sustained current spike — distinguishes a true hard stop from
    // normal ball resistance. With 40-50 balls, the arm routinely draws 20-30A in travel.
    // Threshold is set well above expected ball load but below a real physical blockage.
    // Window is long enough (~600ms) to ignore momentary compression spikes.
    // Raise threshold further if still false-triggering with a full ball load.
    public static final double EXTENSION_STALL_CURRENT_AMPS  = 38.0; // just below 40A stator limit
    public static final double EXTENSION_STALL_VELOCITY_RPS  = 0.5; // kept for future use
    // Ball resistance threshold: softer limit used during RETRACTING/AGITATING only.
    // If hit, arm re-extends rather than forcing through the ball load.
    // TODO: tune — start conservative, raise if normal retraction false-triggers.
    public static final double BALL_RESISTANCE_CURRENT_AMPS  = 25.0;
    public static final int    BALL_RESISTANCE_LOOP_THRESHOLD = 5; // ~100ms sustained before reacting
    // TODO: tune after observing Intake/RollerCurrentAmps in AdvantageScope with/without balls
    public static final double ROLLER_LOAD_CURRENT_AMPS = 35.0; // log shows normal full-hopper load at 20-29A; true jam is 35A+
    // Jam recovery: brief reverse pulse duration when roller is under sustained load.
    // Log shows 0.15s was too short — ball re-jams immediately after pulse ends.
    public static final boolean ROLLER_JAM_RECOVERY_ENABLED = true;
    public static final double ROLLER_JAM_REVERSE_SEC  = 0.15; // short pulse — just enough to nudge a stuck ball loose
    public static final int    ROLLER_JAM_LOOP_THRESHOLD = 5;  // ~100ms to detect, down from 200ms
  }

  // Autonomous path following (PathPlanner PID tuning)
  public static final class Auto {
    // Translation PID — raised from 1.5 to fight ball resistance and contact at competition
    public static final double TRANSLATION_KP = 2.5;  
    public static final double TRANSLATION_KI = 0.0;
    public static final double TRANSLATION_KD = 0.0;
    
    // Rotation PID (trust your SysId steer gains!)
    public static final double ROTATION_KP = 3.5;     
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.0;
    
    public static final double MAX_MODULE_SPEED_MPS = 3.5; // PathPlanner cap — independent of teleop max
    
    // Starting pose validation (how close robot must be to expected start)
    public static final double STARTING_POSE_TOLERANCE_METERS = 0.15;
    public static final double STARTING_POSE_TOLERANCE_DEGREES = 5.0;
    public static final double VISION_INITIALIZATION_TIMEOUT_SECONDS = 7.0;
    public static final Pose2d DEFAULT_FALLBACK_POSE = StartingPositions.BLUE_REBUILT_RIGHT_CORNER;
    
    // Post-path correction timeout (SmartDrive precision phase)
    public static final double POST_PATH_CORRECTION_TIMEOUT_S = 3.0;

    // ShootInPlace auto timing — tune these to match flywheel spin-up and ball count
    public static final double SHOOT_IN_PLACE_SPINUP_SECONDS = 2.0; // time to reach target RPS
    public static final double SHOOT_IN_PLACE_SHOOT_SECONDS  = 8.0; // feed window for ~8 balls
  }

// Bump traversal staging poses (blue alliance frame, red mirrored automatically)
  public static final class BumpPoses {
    // Left bump staging poses (Y = 5.528 m, bump centerline 217.64 in from wall)
    public static final Pose2d BLUE_LEFTBUMP_ALLIANCE_STAGING = new Pose2d(2.972, 5.39, Rotation2d.fromDegrees(45.0));
    public static final Pose2d BLUE_LEFTBUMP_NEUTRAL_STAGING = new Pose2d(6.284, 5.39, Rotation2d.fromDegrees(45.0));
   // Right bump staging poses (Y = 2.508 m, bump centerline 98.76 in from wall)
    public static final Pose2d BLUE_RIGHTBUMP_ALLIANCE_STAGING = new Pose2d(2.972, 2.35, Rotation2d.fromDegrees(45.0));
    public static final Pose2d BLUE_RIGHTBUMP_NEUTRAL_STAGING = new Pose2d(6.284, 2.35, Rotation2d.fromDegrees(45.0));

    // Red alliance mirrored poses
    public static final Pose2d RED_LEFTBUMP_ALLIANCE_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_LEFTBUMP_ALLIANCE_STAGING.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_LEFTBUMP_ALLIANCE_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
    public static final Pose2d RED_LEFTBUMP_NEUTRAL_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_LEFTBUMP_NEUTRAL_STAGING.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_LEFTBUMP_NEUTRAL_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
    public static final Pose2d RED_RIGHTBUMP_ALLIANCE_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_RIGHTBUMP_ALLIANCE_STAGING.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_RIGHTBUMP_ALLIANCE_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
    public static final Pose2d RED_RIGHTBUMP_NEUTRAL_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_RIGHTBUMP_NEUTRAL_STAGING.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_RIGHTBUMP_NEUTRAL_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
  }

  // Known robot starting positions (blue alliance — red mirrored automatically)
  public static final class StartingPositions {
    public static final Pose2d BLUE_REBUILT_RIGHT_CORNER = new Pose2d(0.4826, 0.4191, Rotation2d.fromDegrees(0.0));
    public static final Pose2d RED_REBUILT_RIGHT_CORNER  = new Pose2d(16.4592, 7.5819, Rotation2d.fromDegrees(0.0));

    public static final Pose2d BLUE_ALLIANCE_LEFTBUMP  = new Pose2d(3.512, 5.489, Rotation2d.fromDegrees(0.0));
    public static final Pose2d BLUE_ALLIANCE_RIGHTBUMP = new Pose2d(3.512, 2.393, Rotation2d.fromDegrees(0.0));

    // ShootInPlace auto starting poses — PoseInitializer flips these for Red automatically
    public static final Pose2d SHOOT_IN_PLACE_START_RIGHT  = new Pose2d(3.620, 2.515, Rotation2d.fromDegrees(0.0));
    public static final Pose2d SHOOT_IN_PLACE_START_LEFT   = new Pose2d(3.620, Field.FIELD_WIDTH_METERS - 2.515, Rotation2d.fromDegrees(0.0));
    public static final Pose2d SHOOT_IN_PLACE_START_CENTER = new Pose2d(3.620, Field.FIELD_WIDTH_METERS / 2.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d SHOOT_IN_PLACE_START_RIGHT_ACCURATE = new Pose2d(3.560, 3.850, Rotation2d.fromDegrees(90.0));
  }

  // Field dimensions
  public static final class Field {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(650.12);
    public static final double FIELD_WIDTH_METERS  = Units.inchesToMeters(316.64);
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