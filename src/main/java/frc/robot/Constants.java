package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
      // Supply limit caps battery draw — 4 motors x 40A = 160A max drive draw.
      config.CurrentLimits.StatorCurrentLimit = 60.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
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
    public static final int FLYWHEEL_BACK_MOTOR_ID = 21;

    // Flywheel velocity gains — front and back motors tuned separately (different inertia/response).
    // kV back-calculated from 8-step RPM table. kP corrects residual steady-state error.
    // 2026-03-07: front was +2.5 RPS over, back was +5 RPS over — reduced kV on both, added kP to front.
    public static final double FLYWHEEL_FRONT_KS = 1.5;
    public static final double FLYWHEEL_FRONT_KV = 0.0975; // reduced from 0.1005 — was +2.5 RPS over
    public static final double FLYWHEEL_FRONT_KP = 0.03;   // added to correct residual steady-state error
    public static final double FLYWHEEL_FRONT_KA = 0.0;
    // Back: kS=1.3 pushed center +5 RPS over — reduced kS and kV to re-center.
    // Still oscillating — mechanical drag suspected, flagged for inspection.
    public static final double FLYWHEEL_BACK_KS  = 1.0;
    public static final double FLYWHEEL_BACK_KV  = 0.108;  // split between 0.103 (low) and 0.109 (high)
    public static final double FLYWHEEL_BACK_KP  = 0.005;
    public static final double FLYWHEEL_BACK_KD  = 0.002;
    public static final double FLYWHEEL_BACK_KA  = 0.0;
    public static final int HOOD_MOTOR_ID = 22;
    public static final int TURRET_MOTOR_ID = 23; // Kraken x44

    // Hood uses a limit switch at the bottom stop (85 deg) instead of a CANcoder.
    // A CANcoder can be retrofitted later — see TODO in TurretIOCTRE.java.
    public static final int HOOD_LIMIT_SWITCH_DIO = 2; // confirmed
    // TODO: if a CANcoder is added to the turret rotation axis later, add TURRET_CANCODER_ID here.
    public static final int HALL_SENSOR_CCW_DIO = 0;  // confirmed — CCW hard stop, primary home sensor

    // Gear train: Kraken X44 (1:1) → 20T pinion → 200T turret ring
    // Motor rotates 10x for every 1 turret rotation.
    public static final double TURRET_GEAR_RATIO = 10.0; // motor rot / turret rot

    // Turret pivot offset from robot center — used to compute accurate bearing to target.
    // Positive X = forward, positive Y = left (WPILib convention).
    // Measured: 4.94 in back, 7.44 in right of robot center.
    public static final double TURRET_PIVOT_OFFSET_X_METERS = Units.inchesToMeters(-4.94);
    public static final double TURRET_PIVOT_OFFSET_Y_METERS = Units.inchesToMeters(-7.44);

    // 350 degrees in 1 second = 0.972 turret rot/sec × 10 = 9.72 motor rot/sec.
    // Kraken X44 free speed ~100 rot/sec so cruise is ~10% throttle — plenty of margin.
    // Acceleration of 3× cruise gives ~0.33 sec to reach full speed from rest.
    // TODO: restore after hall sensor and soft limits are verified on hardware
    // public static final double TURRET_CRUISE_VELOCITY_RPS  = 9.72;  // motor rot/sec
    // public static final double TURRET_ACCELERATION_RPS2    = 30.0;  // motor rot/sec^2

    // Slowed for testing 2026-03-06 — raised cruise/kS to overcome energy chain drag
    public static final double TURRET_CRUISE_VELOCITY_RPS  = 15.0; // motor rot/sec (~540 deg/sec) raised 2026-03-18
    public static final double TURRET_ACCELERATION_RPS2    = 30.0; // motor rot/sec^2
    public static final double TURRET_JERK_RPS3            = 200.0; // S-curve ramp

    // Chain-jam stall recovery — if the turret hasn't moved for this long while a target
    // is active, snap the MM target to current position to stop fighting (and whistling).
    // 2.5s > worst-case full-range move (~1.7s) so healthy moves never trigger this.
    public static final double TURRET_STALL_TIMEOUT_SECS         = 2.5;
    public static final double TURRET_STALL_VELOCITY_THRESHOLD_RPS = 0.3; // motor RPS — "not moving"
    public static final double TURRET_STALL_ERROR_THRESHOLD_ROT    = 0.15; // motor rot — "not on target"

    // MotionMagic slot 0 gains (voltage control mode):
    // kS: static friction kick — raises output just enough to start moving. Tune first.
    // kV: voltage per rot/sec of setpoint velocity (~12V / 100 rot/s free speed = 0.12).
    // kP: voltage per rotation of position error. Must satisfy kP * min_error > kS to avoid stopping short.
    //     At kS=0.9 and worst-case 0.05 rot error, need kP > 18. kP=9.0 relies on kI to close the final gap.
    // kS: static friction — raised to overcome energy chain drag with new replacement part.
    // kD: damping — opposes velocity during the move to reduce overshoot.
    public static final double TURRET_KS = 0.9;  // raised from 0.5 — new chain/part adds static drag
    public static final double TURRET_KV = 0.12;
    public static final double TURRET_KP = 9.0;
    public static final double TURRET_KI = 2.0; // integral to close final gap when kP*error < kS
    public static final double TURRET_KD = 0.3;

    // Motor inversion — confirmed on hardware
    public static final boolean TURRET_MOTOR_INVERTED        = false;
    public static final boolean HOOD_MOTOR_INVERTED           = true;  // inverted 2026-03-06
    public static final boolean FLYWHEEL_FRONT_MOTOR_INVERTED = true;
    public static final boolean FLYWHEEL_BACK_MOTOR_INVERTED  = true;

    // Turret homing: leading edge debounced (3 loops), trailing edge raw (no debounce).
    // Only 0.075 rot between trailing edge and hard stop — cannot wait even one extra loop.
    public static final double TURRET_HOME_SPEED_FAST_PERCENT = 0.06; // slow enough to react to raw trailing edge
    // Hall debounce: leading edge only — 3 loops (60ms) to confirm inside sensor window before watching for exit.
    public static final int    TURRET_HALL_DEBOUNCE_LOOPS     = 3; // rising edge — 60ms
    public static final int    TURRET_HALL_DEBOUNCE_LOOPS_OFF = 1; // falling edge — used only for non-homing logging
    // Stall detection during homing — if current stays above threshold for this many loops,
    // homing aborts. Turret stator limit is 20A, so threshold is set just below that.
    // 10 loops = ~200ms — long enough to ignore startup inrush, short enough to protect the stop.
    public static final double TURRET_HOMING_STALL_CURRENT_AMPS = 28.0; // below 30A breaker; hard stop hits ~40A with tensioner
    public static final int    TURRET_HOMING_STALL_LOOP_THRESHOLD = 10;

    // Hall sensor is near the CCW hard stop (gear relocated to front-left 2026-03-06, dead zone now back-right).
    // Homing sweeps CCW and stops on the leading edge (sensor first fires) — encoder zeroed there.
    // Measured 2026-03-06 (pre-home TunerX values):
    //   Hall leading edge: -2.62 motor rot — encoder zeroed here now (was trailing edge before)
    //   Hall trailing edge: -3.54 motor rot (AScope -1089.7 deg)
    //   CCW hard stop:     -3.615 motor rot → 0.075 rot past trailing edge, 0.995 rot past leading edge
    // Set TURRET_HALL_OFFSET_MOTOR_ROT to shift zero so all downstream positions stay valid.
    // Leading edge is ~0.92 rot CW of old trailing edge zero — set to +0.92 to match old coords.
    public static final double TURRET_HALL_OFFSET_MOTOR_ROT = 0.0; // zero at trailing edge of hall sensor

    // Soft limits: 0 = CCW hard stop (set by TURRET_HALL_OFFSET_MOTOR_ROT). Full range ~9.9 rot.
    public static final double TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT  =  0.05; // CCW soft limit — small margin from hard stop (2026-03-06)
    public static final double TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT =  9.46; // CW soft limit — pulled in 0.3 rot from 9.76 to clear motor limit (2026-03-11)

    // MotionMagic tuning targets — kept 1 motor rot inside the soft limits so MM never commands
    // into the limit zone. Adjust outward once MM tracking is confirmed good.
    public static final double TURRET_MM_TARGET_LEFT_MOTOR_ROT  = TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT;  // 0.05
    public static final double TURRET_MM_TARGET_RIGHT_MOTOR_ROT = TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT; // 9.9

    // Motor encoder value when the turret is pointing straight toward the front of the robot.
    // Measured 2026-03-06: forward = TunerX 2.45, hall trailing edge = TunerX -3.54 → delta = 5.99 motor rot.
    public static final double TURRET_FORWARD_MOTOR_ROT = 6.077637; // measured 2026-03-07 after trailing-edge homing recal

    // When true: operator must hold LT+RT+A after each boot to confirm turret is facing forward.
    // Set false for tournament — turret auto-zeros to forward at boot with no confirmation required.
    public static final boolean REQUIRE_TURRET_FORWARD_CONFIRM = true;

    // Hood homing: creeps downward until the DIO limit switch fires, then zeroes encoder.
    // If the switch is already pressed at boot, homing completes immediately (no motor movement).
    // 0 rotations = bottom stop (limit switch pressed, raw encoder ~0.0067 before zeroing).
    // Positive motor output = hood moving UP. Physical top reads 4.69 motor rot after zeroing.
    public static final double HOOD_HOME_SPEED_PERCENT       = 0.10;

    // Hood MotionMagic gains — start conservative, tune kP up until no steady-state error
    public static final double HOOD_KS                   = 0.4;  // static friction override — increase if hood won't start moving
    public static final double HOOD_KV                   = 0.12; // velocity feedforward
    public static final double HOOD_KP = 14.0;
    public static final double HOOD_CRUISE_VELOCITY_RPS  = 9.0;  // motor rot/s
    public static final double HOOD_ACCELERATION_RPS2    = 30.0; // motor rot/s^2
    public static final double HOOD_HOME_ROTATIONS           = 0.0;  // encoder value at home (limit switch pressed)
    public static final double HOOD_HOME_BACKOFF_ROTATIONS   = 0.05; // small backoff after homing so motor isn't held against stop
    // Stall fallback — if limit switch never fires, stall current stops the motor at the hard stop.
    public static final double HOOD_HOMING_STALL_CURRENT_AMPS  = 10.0;
    public static final int    HOOD_HOMING_STALL_LOOP_THRESHOLD = 6;   // ~120ms at 50Hz
    public static final double HOOD_SOFT_LIMIT_TOP_ROTATIONS = 4.69; // measured 2026-03-08 with new gearing (was 4.1)

    // Phase-advance enablement — change to advance to the next phase
    // PHASE_1: fixed/manual setpoint, fire interlock only
    // PHASE_2: turret tracks target, robot must be near-stationary to fire
    // PHASE_3: turret tracks while driving, fire only when chassis slows below threshold
    // PHASE_4: full on-the-move — lead compensation offsets target by velocity * TOF so ball lands on hub while robot is moving.
    //   TOF values (~1.0s) were measured via slo-mo video: time from ball appearing at turret mouth to breaking the goal plane.
    //   Shots are parabolic (lobbed arc with gravity), not flat — TOF does not scale linearly with distance.
    //   At 3 m/s robot speed and 1.0s TOF, lead offset is ~3m — this is physically correct.
    //   At 8 balls/sec throughput target, 8 balls are simultaneously in flight during a sustained burst.
    //   No inter-ball timing is enforced in software — feed rate is determined by spindexer/singulator cycle time.
    public enum TurretPhase { PHASE_1_STATIC, PHASE_2_TRACKING, PHASE_3_DECEL_SHOOT, PHASE_4_ON_THE_MOVE }
    // PHASE_1: turret locked forward (0 rot), hood+flywheel still auto-adjust by distance.
    // Advance to PHASE_2+ once turret rotation is re-confirmed on rebuilt robot.
    public static final TurretPhase CURRENT_PHASE = TurretPhase.PHASE_4_ON_THE_MOVE;

    // When true, RT toggles shooting mode on/off instead of hold-to-shoot.
    // Flywheels stay on continuously. Singulator feeds one ball every 4 seconds,
    // gated by the singulator LaserCAN (not the dead zone). For solo driving tests.
    public static final boolean SEQUENCED_SHOOTING_TESTING_MODE = false;

    // Phase 3+: only allow firing when chassis translation is below this speed.
    // Wired into isReadyToShoot() for all phases — set to a large value to effectively disable.
    // TODO(beam-breaks): tune this once beam breaks are working so we can safely gate mid-match.
    //   Suggested starting value: 0.5 m/s. Currently disabled (999) to avoid pausing the feed
    //   chain without sensor feedback — a paused singulator mid-travel causes dead zone jams.
    public static final double CHASSIS_SPEED_FIRE_THRESHOLD_MPS = 999.0;
    public static final double CHASSIS_SPEED_LATCH_THRESHOLD_MPS = 0.1; // latch freezes below this — essentially stopped
    public static final double CHASSIS_OMEGA_LATCH_THRESHOLD_RPS = 0.5; // ~29 deg/s — only freeze latch when truly settled, not just slow

    // "Ready to shoot" tolerances — all must pass for isReadyToShoot() to return true
    public static final double TURRET_ON_TARGET_TOLERANCE_ROT  = 0.02; // ~7 degrees
    public static final double HOOD_ON_TARGET_TOLERANCE_ROT    = 0.05; // ~1% of full travel — loose enough to not block
    public static final double FLYWHEEL_ON_TARGET_TOLERANCE_PCT = 0.03; // within 3% of setpoint (legacy, unused)
    public static final double FLYWHEEL_ON_TARGET_TOLERANCE_RPS = 3.0;  // within 3 RPS of target
    // Minimum RPM both flywheels must reach before RT starts feeding when spun up from cold.
    // Throughput target: start at 4 balls/sec, ramp to 6, then 8 as feed chain is validated.
    // No software ceiling — rate is limited only by spindexer/singulator cycle time and ball supply.
    // Two high-powered motors showed minimal velocity dip during a 20-shot sustained burst at WNE.
    // Verify dip/recovery in AScope: watch Turret/FlywheelFrontRpm and Turret/FlywheelBackRpm
    // during a full burst at each throughput target before stepping up.
    public static final double FLYWHEEL_SPINUP_MIN_RPM = 1500.0; // tune to match actual spin-up curve

    // Manual flywheel RPS tuning via operator LB/RB.
    // Each press steps front RPS by this amount. Back RPS = front * FLYWHEEL_BACK_RATIO.
    // Ratio derived from shot table averages: back motor runs ~7% slower than front at all distances.
    // Starting RPS is the HUBCLOSE front value — operator steps from there during calibration.
    public static final double FLYWHEEL_MANUAL_STEP_RPS  = 1.0;
    public static final double FLYWHEEL_BACK_RATIO       = 0.93;
    public static final double FLYWHEEL_MANUAL_MIN_RPS   = 20.0;
    public static final double FLYWHEEL_MANUAL_MAX_RPS   = 100.0;
    // Global multiplier applied to all shot table RPS values before commanding the motors.
    // Use to compensate for mechanical changes (e.g. new belt/roller) without re-measuring the table.
    // 1.0 = full power, 0.95 = 5% reduction.
    public static final double FLYWHEEL_RPS_SCALE        = 0.97;  // baked into table values as of 2026-03-20
    // Fallback warmup speed used when LT spins up flywheels but the aim pipeline has no target yet.
    public static final double FLYWHEEL_WARMUP_FRONT_RPS = 60.0;
    public static final double FLYWHEEL_WARMUP_BACK_RPS  = 60.0;
    // Distance used for hood/flywheel lookup when pose is unavailable during Phase1Fallback.
    public static final double FALLBACK_DISTANCE_METERS  = 4.0;

    // Consecutive loops turret must stay within tolerance before isAimed()/isReadyToShoot() pass.
    // Prevents firing during a large slew where the turret is briefly passing through the tolerance band.
    // For Phase 4 (8 balls/sec): keep at 0 — at 125ms/ball there is no time budget for settle confirmation.
    // The lead comp is the accuracy mechanism, not settle gating.
    // For Phase 2/3 (stationary or decel): set to 3-5 loops (60-100ms) to avoid feeding mid-slew.
    public static final int TURRET_ON_TARGET_SETTLE_LOOPS = 0;

    // Shooter geometry:
    // Two hex shafts (front roller and top/back roller) separated by a fixed distance.
    // Front roller position is fixed relative to the turret plate.
    // Top roller pivots around the front roller as the hood rotates — so the ball exit
    // point shifts in both height and forward offset as the hood angle changes.
    // Hood range: ~35 deg (pointing forward, long pass) to ~85 deg (steep lob, short range).
    // The front roller is the effective origin for all distance/trajectory math.
    // Hood angle bakes the exit-point offset into each shot table row implicitly —
    // do NOT assume a fixed launch height when adding ballistic math later.
    // Measured 2026-03-07 after singulator gusset install (ball now centered on flywheel shaft).
    public static final double HOOD_SHAFT_SEPARATION_M    = Units.inchesToMeters(8.5);  // center-to-center between front and back hex shafts
    public static final double HOOD_FRONT_ROLLER_HEIGHT_M = Units.inchesToMeters(19.52); // front shaft center above ground
    public static final double HOOD_ANGLE_MIN_DEG         = 35.0; // hood all the way open — back roller forward, shallow launch angle, long range
    public static final double HOOD_ANGLE_MAX_DEG         = 85.0; // hood home position — back roller nearly above front roller, steep lob, short range
    // Each shaft has 3 flywheels (center one is weighted). All wheels are 3in diameter = 1.5in radius.
    // Ball exit speed = flywheel surface speed = RPS * 2 * pi * FLYWHEEL_RADIUS_M.
    // Ball is compressed between front and back rollers — average of both surface speeds is used for launch velocity.
    public static final double FLYWHEEL_RADIUS_M          = Units.inchesToMeters(1.5);  // 3in diameter wheels, 1.5in radius
    public static final int    FLYWHEEL_COUNT_PER_SHAFT   = 3; // 3 wheels per shaft, center wheel is weighted

    // Shot lookup table — distance (meters) -> flywheel percent -> hood rotations
    // Distances are turret-pivot to hub opening center.
    // CLOSE: robot bumper pressed against the hub face (~0.6m — robot half-length + hub clearance)
    // MID:   halfway between close and far corners (~3.1m)
    // FAR:   robot in the far corner near (0,0), farthest shot in blue alliance zone (~5.5m)
    public static final double[] SHOT_TABLE_DISTANCES_M        = { 0.6,    3.1,    5.5   };
    public static final double[] SHOT_TABLE_FLYWHEEL_FRONT_PCT = { 0.435,  0.546,  0.638 };
    public static final double[] SHOT_TABLE_FLYWHEEL_BACK_PCT  = { 0.50,   0.84,   0.90  };
    public static final double[] SHOT_TABLE_HOOD_ROTATIONS     = { 0.181,  0.153,  2.404 };

    // Measured 2026-03-07 via RPM recorder (settle 3s per step, 8-step 30-100% sweep).
    // Use these for VelocityVoltage closed-loop instead of percent. Front motor runs ~10% faster than back.
    // PCT:   30%     40%     50%     60%     70%     80%     90%     100%
    public static final double[] MEASURED_PCT_STEPS        = { 0.30,  0.40,  0.50,  0.60,  0.70,  0.80,  0.90,  1.00  };
    public static final double[] MEASURED_FRONT_RPS        = { 29.52, 39.99, 50.14, 59.56, 68.86, 78.10, 87.16, 94.87 };
    public static final double[] MEASURED_BACK_RPS         = { 26.61, 36.91, 46.38, 56.52, 65.64, 74.58, 82.11, 86.40 };
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

  // Spindexer hardware IDs and tuning constants (cone spinner that feeds balls into singulator)
  public static final class Spindexer {
    public static final int MOTOR_ID = 27; // REV NEO on SparkMax

    // TODO: flip to true if motor runs backwards on first test
    public static final boolean MOTOR_INVERTED = true;

    public static final double FORWARD_SPEED = 0.85;
    // Full reverse on agitate — maximum force to break jam in minimum time.
    // Previous value was -0.50 (conservative placeholder).
    public static final double REVERSE_SPEED = -1.0;

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
    // Extension (Kraken): stator caps torque current; supply caps battery draw.
    // Roller (NEO SparkMax): uses smart current limit only.
    public static final double EXTENSION_STATOR_LIMIT_AMPS = 40.0;
    public static final double EXTENSION_SUPPLY_LIMIT_AMPS = 30.0;
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

  // Climber mechanism hardware IDs and tuning
  public static final class Climber {
    public static final int PULL_MOTOR_ID = 50;
    public static final int ROTATION_MOTOR_ID = 51;

    // Inversion - confirm on hardware before enabling
    public static final boolean PULL_MOTOR_INVERTED     = false; // TODO: verify direction
    public static final boolean ROTATION_MOTOR_INVERTED = false; // TODO: verify direction

    // Current limits - conservative until mechanism is tested on robot
    public static final double PULL_STATOR_LIMIT_AMPS     = 40.0; // tune up if motor stalls under load
    public static final double PULL_SUPPLY_LIMIT_AMPS     = 30.0;
    public static final double ROTATION_STATOR_LIMIT_AMPS = 30.0;
    public static final double ROTATION_SUPPLY_LIMIT_AMPS = 25.0;

    // Operator control speeds - start very slow, increase once direction is confirmed
    public static final double PULL_SPEED     = 0.15; // tune on hardware
    public static final double ROTATION_SPEED = 0.15; // tune on hardware
  }

  // Autonomous path following (PathPlanner PID tuning)
  public static final class Auto {
    // Translation PID (trust your SysId drive gains!)
    public static final double TRANSLATION_KP = 1.5;  
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
    public static final Pose2d BLUE_AUTO_START_RIGHT = new Pose2d(3.657, 2.444, Rotation2d.fromDegrees(0.0));
   // Right bump staging poses (Y = 2.508 m, bump centerline 98.76 in from wall)
    public static final Pose2d BLUE_RIGHTBUMP_ALLIANCE_STAGING = new Pose2d(2.972, 2.35, Rotation2d.fromDegrees(45.0));
    public static final Pose2d BLUE_RIGHTBUMP_NEUTRAL_STAGING = new Pose2d(6.284, 2.35, Rotation2d.fromDegrees(45.0));

    // Far-side (opponent zone) staging poses (X pushed into opponent zone past neutral exit)
    public static final Pose2d BLUE_LEFTBUMP_OPPONENT_STAGING = new Pose2d(9.596, 5.39, Rotation2d.fromDegrees(45.0));
    public static final Pose2d BLUE_RIGHTBUMP_OPPONENT_STAGING = new Pose2d(9.596, 2.35, Rotation2d.fromDegrees(45.0));

    // Red alliance mirrored poses (rotational symmetry: X = FIELD_LENGTH-X, Y = FIELD_WIDTH-Y, rot+180)
    public static final Pose2d RED_LEFTBUMP_ALLIANCE_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_LEFTBUMP_ALLIANCE_STAGING.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_LEFTBUMP_ALLIANCE_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
    public static final Pose2d RED_LEFTBUMP_NEUTRAL_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_LEFTBUMP_NEUTRAL_STAGING.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_LEFTBUMP_NEUTRAL_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
    public static final Pose2d RED_LEFTBUMP_OPPONENT_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_LEFTBUMP_OPPONENT_STAGING.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_LEFTBUMP_OPPONENT_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));

    public static final Pose2d RED_RIGHTBUMP_ALLIANCE_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_RIGHTBUMP_ALLIANCE_STAGING.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_RIGHTBUMP_ALLIANCE_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
    public static final Pose2d RED_RIGHTBUMP_NEUTRAL_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_RIGHTBUMP_NEUTRAL_STAGING.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_RIGHTBUMP_NEUTRAL_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
    public static final Pose2d RED_RIGHTBUMP_OPPONENT_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_RIGHTBUMP_OPPONENT_STAGING.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_RIGHTBUMP_OPPONENT_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
  }

  // Field positions (all blue alliance - red is mirrored automatically)
  public static final class StartingPositions {
    
    
    // TUNING POSITION
    public static final Pose2d PID_TUNING_POSITION = new Pose2d(1.270, 2.230, Rotation2d.fromDegrees(0.0));
    // LED CALIBRATION TEST POSITION (from actual Limelight reading)
    public static final Pose2d LED_TEST_POSITION = new Pose2d(1.2748, 2.3987, Rotation2d.fromDegrees(-6.56));
    
    // Staging poses (Phase 1: PathPlanner pathfind targets)
    public static final Pose2d BLUE_REBUILT_HUB_RIGHT_ACCURATE = new Pose2d(3.560, 3.850, Rotation2d.fromDegrees(90.0)); 
    public static final Pose2d BLUE_REBUILT_RIGHT_CORNER = new Pose2d(0.4826, 0.4191, Rotation2d.fromDegrees(0.0)); 
    public static final Pose2d RED_REBUILT_RIGHT_CORNER = new Pose2d(16.4592, 7.5819, Rotation2d.fromDegrees(0.0)); 
    // Practice-field seed pose for Red - physically place robot at left wall (mirrored from Blue right corner Y)
    public static final Pose2d RED_PRACTICE_SEED = new Pose2d(9.27, 7.6239, Rotation2d.fromDegrees(180.0));
    public static final Pose2d BLUE_ALLIANCE_LEFTBUMP = new Pose2d(3.512, 5.489, Rotation2d.fromDegrees(0.0)); 
    public static final Pose2d PRECISE_BLUE_ALLIANCE_LEFTBUMP = new Pose2d(3.712, 5.489, Rotation2d.fromDegrees(0.0)); 
    public static final Pose2d BLUE_ALLIANCE_RIGHTBUMP = new Pose2d(3.512, 2.393, Rotation2d.fromDegrees(0.0)); 
    public static final Pose2d PRECISE_BLUE_ALLIANCE_RIGHTBUMP = new Pose2d(3.712, 2.393, Rotation2d.fromDegrees(0.0)); 
    public static final Pose2d BLUE_ALLIANCE_RIGHTOWER = new Pose2d(1.755, 3.29, Rotation2d.fromDegrees(.0)); 
    public static final Pose2d PRECISE_BLUE_ALLIANCE_RIGHTOWER = new Pose2d(1.555, 3.29, Rotation2d.fromDegrees(.0)); 

    // Straight-on shot seed poses — robot faces hub at 0deg, Y=4.022 (hub center Y).
    // X = hub_center_x(4.612) - robot_center_to_hub_distance. Use START button to seed these.
    // HUBCLOSE original measured pose (not straight-on, kept for reference):
    public static final Pose2d SHOT_SEED_HUBCLOSE   = new Pose2d(3.475, 4.005, Rotation2d.fromDegrees(0.0));
    public static final Pose2d SHOT_SEED_HUB1_7M    = new Pose2d(2.94,  4.01,  Rotation2d.fromDegrees(0.0)); // ~1.67m from hub
    public static final Pose2d SHOT_SEED_HUB_RIGHT_ACCURATE = new Pose2d(3.560, 3.850, Rotation2d.fromDegrees(90.0)); // BLUE_REBUILT_HUB_RIGHT_ACCURATE — up against hub facing left
    public static final Pose2d SHOT_SEED_RIGHT_BUMP = new Pose2d(3.620, 2.515, Rotation2d.fromDegrees(0.0)); // ShootInPlaceRight start
    public static final Pose2d SHOT_SEED_LEFT_BUMP  = new Pose2d(3.620, Field.FIELD_WIDTH_METERS - 2.515, Rotation2d.fromDegrees(0.0)); // ShootInPlaceLeft start
    public static final Pose2d SHOT_SEED_OUTPOST        = new Pose2d(0.4826, 0.4191, Rotation2d.fromDegrees(0.0)); // BLUE_REBUILT_RIGHT_CORNER / outpost start
    public static final Pose2d SHOT_SEED_BACK_WALL_RIGHT = new Pose2d(0.483, 2.500, Rotation2d.fromDegrees(0.0)); // RIGHT_CORNER measured 2026-03-17
    public static final Pose2d SHOT_SEED_RIGHT_CORNER = new Pose2d(0.483,  2.500,  Rotation2d.fromDegrees(0.0)); // RIGHT_CORNER measured 2026-03-17
    public static final Pose2d SHOT_SEED_2M   = new Pose2d(2.612, 4.022, Rotation2d.fromDegrees(0.0)); // 4.612-2.0
    public static final Pose2d SHOT_SEED_2_5M = new Pose2d(2.112, 4.022, Rotation2d.fromDegrees(0.0)); // 4.612-2.5
    public static final Pose2d SHOT_SEED_3M   = new Pose2d(1.612, 4.022, Rotation2d.fromDegrees(0.0)); // 4.612-3.0
    public static final Pose2d SHOT_SEED_4M   = new Pose2d(0.612, 4.022, Rotation2d.fromDegrees(0.0)); // 4.612-4.0
    public static final Pose2d SHOT_SEED_4_5M = new Pose2d(0.112, 4.022, Rotation2d.fromDegrees(0.0)); // 4.612-4.5

    // AUTO RESET POSE STAGING
    public static final Pose2d BLUE_AUTO_START_POS_FAR_RIGHT = new Pose2d(5.533, 1.185, Rotation2d.fromDegrees(180.0));
    
    // Precision poses (Phase 2: AutoPilot final targets)
    
    // AUTO RESET POSE PRECISE
    public static final Pose2d PRECISE_BLUE_AUTO_START_POS_FAR_RIGHT = new Pose2d(6.033, 0.985, Rotation2d.fromDegrees(180.0));

    // ShootInPlace auto starting poses — Blue side. PoseInitializer flips these for Red automatically.
    public static final Pose2d SHOOT_IN_PLACE_START_RIGHT          = new Pose2d(3.620, 2.515, Rotation2d.fromDegrees(0.0));
    public static final Pose2d SHOOT_IN_PLACE_START_RIGHT_ACCURATE = new Pose2d(3.560, 3.850, Rotation2d.fromDegrees(90.0)); // up against hub facing left
    public static final Pose2d SHOOT_IN_PLACE_START_LEFT           = new Pose2d(3.620, Field.FIELD_WIDTH_METERS - 2.515, Rotation2d.fromDegrees(0.0));
    public static final Pose2d SHOOT_IN_PLACE_START_CENTER         = new Pose2d(3.620, Field.FIELD_WIDTH_METERS / 2.0, Rotation2d.fromDegrees(0.0));
  }

  // Field constants used for fixed target mirroring
  //
  // Field X zones (blue-origin, all in meters):
  //
  //   0.000 ──────────────────────────────── 4.030  ALLIANCE ZONE (blue)
  //                                                   Robot scores into hub here.
  //                                                   Hub center at X=4.612 (181.56in from wall).
  //
  //   4.030 ──────────────────────────────── 5.208  HUB / BUMP / TRENCH DEAD ZONE
  //                                                   Hub structure (47in square) + bump + net occupy this strip.
  //                                                   Not usable for shooting. Treated as alliance zone for
  //                                                   shoot suppression (hub is still reachable from here).
  //
  //   5.208 ──────────────────────────────── 11.305  NEUTRAL ZONE
  //                                                   120in each side of center line (240in total), full width.
  //                                                   Pass back here during first 15s of opponent active period.
  //                                                   After 15s: collect only, fill hopper for next active period.
  //
  //   11.305 ─────────────────────────────── 12.481  OPPONENT HUB / BUMP / TRENCH DEAD ZONE
  //                                                   Mirror of blue dead zone.
  //
  //   12.481 ─────────────────────────────── 16.511  OPPONENT ALLIANCE ZONE (red)
  //                                                   Never shoot here.
  //
  public static final class Field {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(650.12); // 2026 AndyMark perimeter
    public static final double FIELD_WIDTH_METERS  = Units.inchesToMeters(316.64);
    // Alliance zone: 158.60in from each alliance wall.
    public static final double ALLIANCE_ZONE_LENGTH_METERS = Units.inchesToMeters(158.60);
    // Neutral zone: 120in each side of the center line (240in total), full field width.
    // The gap between ALLIANCE_ZONE_LENGTH and NEUTRAL_ZONE_LOW_X is the hub/bump/trench dead zone.
    public static final double NEUTRAL_ZONE_HALF_LENGTH_METERS = Units.inchesToMeters(120.0);
    public static final double NEUTRAL_ZONE_LOW_X_METERS  = (FIELD_LENGTH_METERS / 2.0) - NEUTRAL_ZONE_HALF_LENGTH_METERS;
    public static final double NEUTRAL_ZONE_HIGH_X_METERS = (FIELD_LENGTH_METERS / 2.0) + NEUTRAL_ZONE_HALF_LENGTH_METERS;
  }

  // Hub centers for aiming
  public static final class HubCenters {
    public static final Pose2d BLUE_HUB_CENTER = new Pose2d(4.612, 4.022, Rotation2d.fromDegrees(0.0));
    public static final Pose2d RED_HUB_CENTER = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_HUB_CENTER.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_HUB_CENTER.getY(),
        Rotation2d.fromDegrees(180.0));

    // Bump no-shoot zone: 73in x 47in rectangle centered on each hub.
    // Auto shoot is suppressed when the robot is inside this area — the tilted surface
    // makes accurate shots unreliable and the robot is too close for any shot profile.
    public static final double BUMP_HALF_WIDTH_X_METERS = Units.inchesToMeters(73.0 / 2.0);
    public static final double BUMP_HALF_WIDTH_Y_METERS = Units.inchesToMeters(47.0 / 2.0);
  }

  // Hub opening geometry
  public static final class HubOpening {
    public static final double OPENING_HEIGHT_METERS = Units.inchesToMeters(72.0);
    public static final double OPENING_WIDTH_METERS = Units.inchesToMeters(41.0);
    public static final double OPENING_HALF_WIDTH_METERS = OPENING_WIDTH_METERS / 2.0;

    public static final Pose3d BLUE_HUB_OPENING_CENTER = new Pose3d(
        HubCenters.BLUE_HUB_CENTER.getX(),
        HubCenters.BLUE_HUB_CENTER.getY(),
        OPENING_HEIGHT_METERS,
        new Rotation3d());

    public static final Pose3d RED_HUB_OPENING_CENTER = new Pose3d(
        HubCenters.RED_HUB_CENTER.getX(),
        HubCenters.RED_HUB_CENTER.getY(),
        OPENING_HEIGHT_METERS,
        new Rotation3d());
  }

  // Pass targets for alliance-zone handoff — balls lobbed from the neutral zone land here.
  // X=1.5m puts the landing spot well inside the alliance zone, away from robots near the hub.
  // Y values place targets on either side of the hub center (Y=4.022) to avoid the net.
  public static final class PassTargets {
    public static final Pose2d BLUE_PASS_TARGET_LEFT  = new Pose2d(2.5, 5.99, Rotation2d.fromDegrees(0.0));
    public static final Pose2d BLUE_PASS_TARGET_RIGHT = new Pose2d(2.5, 2.05, Rotation2d.fromDegrees(0.0));

    // Red targets are the 180deg field rotation of blue targets.
    // Note: rotation flips which Y is "left" — RED_PASS_TARGET_LEFT has low Y (mirrors BLUE_PASS_TARGET_RIGHT).
    // The selector swaps left/right on red to compensate.
    public static final Pose2d RED_PASS_TARGET_LEFT = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_PASS_TARGET_LEFT.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_PASS_TARGET_LEFT.getY(),
        Rotation2d.fromDegrees(180.0));
    public static final Pose2d RED_PASS_TARGET_RIGHT = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_PASS_TARGET_RIGHT.getX(),
        Field.FIELD_WIDTH_METERS  - BLUE_PASS_TARGET_RIGHT.getY(),
        Rotation2d.fromDegrees(180.0));
  }

  // Fixed target poses for turret aiming (blue alliance, red mirrored later)
  public static final class TurretTargets {
    //public static final Pose2d BLUE_PASS_TARGET_LEFT = new Pose2d(3.46, 5.83, Rotation2d.fromDegrees(0.0));

    // Named shot positions — turret rot, hood rot, front RPS, back RPS — measured 2026-03-07
    // Robot pose is blue alliance field coords (x, y, omega). Distance is to BLUE_HUB_CENTER (4.612, 4.022).
    // HUBCLOSE: pose=(3.475, 4.005, 0deg) — measured 2026-03-19
    public static final double HUBCLOSE_TURRET_ROT   = 5.90;
    public static final double HUBCLOSE_HOOD_ROT      = 0.2345; // measured 2026-03-19
    public static final double HUBCLOSE_FRONT_RPS     = 46.00;  // 50.0 * 0.92 (2026-03-20 mechanical adjustment)
    public static final double HUBCLOSE_BACK_RPS      = 42.78;  // 46.0 * 0.93
    public static final double HUBCLOSE_TOF_SECONDS   = 1.013;  // -10% from 1.125 (2026-03-09)

    // HUB1_7M: pose=(2.94, 4.01, 0deg), ~1.67m from hub center — measured 2026-03-19
    public static final double HUB1_7M_TURRET_ROT    = 6.006; // measured
    public static final double HUB1_7M_HOOD_ROT      = 0.469; // measured
    public static final double HUB1_7M_FRONT_RPS     = 44.16;  // 48.0 * 0.92 (2026-03-20 mechanical adjustment)
    public static final double HUB1_7M_BACK_RPS      = 41.07;  // 44.16 * 0.93
    public static final double HUB1_7M_TOF_SECONDS   = 1.013; // estimated

    // MIDRANGE: pose=(2.091, 5.912, -90.36deg) — locked in 2026-03-08
    // Turret pivot offset (-4.94in X, -7.44in Y) at -90.36deg heading -> pivot=(1.903, 6.039)
    // Pivot-to-hub dist = sqrt((4.612-1.903)^2 + (4.022-6.039)^2) = 3.38m
    // Interp fraction = (3.38-1.45)/(5.70-1.45) = 0.454 between HUBCLOSE and OUTPOST
    // Field angle to hub = atan2(-1.890, 2.521) = -36.8deg. Robot-relative = -36.8-(-90.36) = +53.56deg.
    // Turret CW = negative: (-53.56/360)*10 = -1.488 rot. TURRET_FORWARD(6.077637) - 1.488 = 4.590 (~4.582 measured).
    /*public static final double MIDRANGE_TURRET_ROT   = 4.582;
    public static final double MIDRANGE_HOOD_ROT      = 1.258;   // rescaled from 1.1 (was 1.1/4.1, now 1.1/4.1*4.69)
    public static final double MIDRANGE_FRONT_RPS     = 51.41;  // -3% from 53.0 (2026-03-08 turret mods)
    public static final double MIDRANGE_BACK_RPS      = 48.50;  // -3% from 50.0
    public static final double MIDRANGE_TOF_SECONDS   = 1.017;  // -10% from 1.13 (2026-03-09)

    // OUTPOST: pose=(0.4826, 0.4191, 0deg), ~5.48m from hub center — locked in 2026-03-07
    public static final double OUTPOST_TURRET_ROT    = 4.93;
    public static final double OUTPOST_HOOD_ROT       = 2.402; // rescaled from 2.1 (was 2.1/4.1, now 2.1/4.1*4.69)
    public static final double OUTPOST_FRONT_RPS      = 63.05;  // -3% from 65.0 (2026-03-08 turret mods)
    public static final double OUTPOST_BACK_RPS       = 60.82;  // -3% from 62.7
    public static final double OUTPOST_TOF_SECONDS    = 0.990;  // -10% from 1.1 (2026-03-09)
*/
    // RIGHT_BUMP: pose=(3.620, 2.515, 0deg) — ShootInPlaceRight auto start position
    // Pivot at (3.494, 2.326). Hub at (4.612, 4.022). Pivot-to-hub dist = 2.031m
    // Measured on field 2026-03-17.
    public static final double RIGHT_BUMP_TURRET_ROT  = 4.506; // measured
    public static final double RIGHT_BUMP_HOOD_ROT    = 0.469; // measured (was 0.618 interpolated)
    public static final double RIGHT_BUMP_FRONT_RPS   = 46.96; // 53.36 * 0.88 (2026-03-21)
    public static final double RIGHT_BUMP_BACK_RPS    = 43.67; // 46.96 * 0.93
    public static final double RIGHT_BUMP_TOF_SECONDS = 1.014; // interpolated
/*
    // LEFT_BUMP: same distance as RIGHT_BUMP but hub is 56.6deg CW from forward.
    // CW = add: 6.078 + (56.6/360)*10 = 7.650. All hood/RPS/TOF identical to RIGHT_BUMP.
    public static final double LEFT_BUMP_TURRET_ROT   = 7.650; // tune on field
    public static final double LEFT_BUMP_HOOD_ROT     = 0.469; // matched to RIGHT_BUMP measured
    public static final double LEFT_BUMP_FRONT_RPS    = 54.51; // 59.25 * 0.92 (2026-03-20 mechanical adjustment)
    public static final double LEFT_BUMP_BACK_RPS     = 50.70; // 55.0 * 0.92
    public static final double LEFT_BUMP_TOF_SECONDS  = 1.014;

    // RIGHT_CORNER: pose=(0.483, 2.500, 0deg) — measured 2026-03-17
    // Pivot at (0.358, 2.311). Hub at (4.612, 4.022). Pivot-to-hub dist = 4.586m
    // t = (4.586-3.38)/(5.70-3.38) = 0.520 between MIDRANGE and OUTPOST
    // Bearing = 21.97deg CCW. Motor rot = 6.078 - (21.97/360)*10 = 5.468 (measured 5.504, close)
    // Hood 1.407 is flatter than interpolated 1.852 — real shot, trust the measurement.
    // Front RPS 59.25 and Back 55.00 align well with interpolated 57.46 / 54.91.
    public static final double RIGHT_CORNER_TURRET_ROT  = 5.504;
    public static final double RIGHT_CORNER_HOOD_ROT    = 1.407;
    public static final double RIGHT_CORNER_FRONT_RPS   = 54.51; // 59.25 * 0.92 (2026-03-20 mechanical adjustment)
    public static final double RIGHT_CORNER_BACK_RPS    = 50.70; // 55.0 * 0.92
    public static final double RIGHT_CORNER_TOF_SECONDS = 1.012; // interpolated at t=0.520
 */
    // CENTER_2_4M: pose=(2.209, 4.057, 0deg), ~2.40m from hub — measured 2026-03-19
    public static final double CENTER_2_4M_TURRET_ROT  = 6.07;
    public static final double CENTER_2_4M_HOOD_ROT    = 0.7035;
    public static final double CENTER_2_4M_FRONT_RPS   = 47.38; // 51.5 * 0.92 (2026-03-20 mechanical adjustment)
    public static final double CENTER_2_4M_BACK_RPS    = 44.06; // 47.38 * 0.93
    public static final double CENTER_2_4M_TOF_SECONDS = 1.015; // interpolated

    // LEFT_3_1M: pose=(1.595, 4.795, 0deg), ~3.12m from hub, slight left — measured 2026-03-19
    public static final double LEFT_3_1M_TURRET_ROT    = 6.3647;
    public static final double LEFT_3_1M_HOOD_ROT      = 0.938;
    public static final double LEFT_3_1M_FRONT_RPS     = 47.38; // 51.5 * 0.92 (2026-03-20 mechanical adjustment)
    public static final double LEFT_3_1M_BACK_RPS      = 44.06; // 47.38 * 0.93
    public static final double LEFT_3_1M_TOF_SECONDS   = 1.017; // interpolated

    // LEFT_4_0M: pose=(1.132, 5.966, 0deg), ~3.99m from hub, left — measured 2026-03-19
    public static final double LEFT_4_0M_TURRET_ROT    = 6.86;
    public static final double LEFT_4_0M_HOOD_ROT      = 1.407;
    public static final double LEFT_4_0M_FRONT_RPS     = 50.60; // 55.0 * 0.92 (2026-03-20 mechanical adjustment)
    public static final double LEFT_4_0M_BACK_RPS      = 47.06; // 50.60 * 0.93
    public static final double LEFT_4_0M_TOF_SECONDS   = 1.010; // interpolated

    // LEFT_3_5M: pose=(3.625, 7.375, 0deg), ~3.49m from hub, far left — measured 2026-03-19
    public static final double LEFT_3_5M_TURRET_ROT    = 8.0;
    public static final double LEFT_3_5M_HOOD_ROT      = 1.175;
    public static final double LEFT_3_5M_FRONT_RPS     = 49.04; // 53.3 * 0.92 (2026-03-20 mechanical adjustment)
    public static final double LEFT_3_5M_BACK_RPS      = 45.61; // 49.04 * 0.93
    public static final double LEFT_3_5M_TOF_SECONDS   = 1.017; // interpolated

    // LEFT_5_5M: pose=(0.493, 7.621, 0deg), ~5.48m from hub, far left — measured 2026-03-19
    public static final double LEFT_5_5M_TURRET_ROT    = 7.21;
    public static final double LEFT_5_5M_HOOD_ROT      = 1.6415;
    public static final double LEFT_5_5M_FRONT_RPS     = 65.5;
    public static final double LEFT_5_5M_BACK_RPS      = 60.92; // 65.5 * 0.93
    public static final double LEFT_5_5M_TOF_SECONDS   = 0.991; // interpolated

    // RIGHT_3_5M: pose=(1.788, 1.905, 0deg), ~3.53m from hub, right — measured 2026-03-19
    // Turret rot measured; hood/RPS/TOF interpolated from LEFT_3_5M–LEFT_4_0M at t=0.08
    public static final double RIGHT_3_5M_TURRET_ROT   = 5.07;
    public static final double RIGHT_3_5M_HOOD_ROT     = 1.194;
    public static final double RIGHT_3_5M_FRONT_RPS    = 49.16; // 53.44 * 0.92 (2026-03-20 mechanical adjustment)
    public static final double RIGHT_3_5M_BACK_RPS     = 45.72; // 49.16 * 0.93
    public static final double RIGHT_3_5M_TOF_SECONDS  = 1.016; // interpolated

    // RIGHT_4_4M: pose=(0.471, 2.495, 0deg), ~4.41m from hub, right — measured 2026-03-19
    public static final double RIGHT_4_4M_TURRET_ROT   = 5.486;
    public static final double RIGHT_4_4M_HOOD_ROT     = 1.407;
    public static final double RIGHT_4_4M_FRONT_RPS    = 55.20; // 60.0 * 0.92 (2026-03-20 mechanical adjustment)
    public static final double RIGHT_4_4M_BACK_RPS     = 51.34; // 55.20 * 0.93
    public static final double RIGHT_4_4M_TOF_SECONDS  = 1.006; // interpolated

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