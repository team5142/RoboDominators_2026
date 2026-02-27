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
      public static final double kP = 25.601;//25.601
      public static final double kI = 0.0;
      public static final double kD = 0.62218;
      public static final double kS = 0.12222;
      public static final double kV = 0;
      public static final double kA = 0;
    }

    public static final class DriveGains {
      public static final double kP = 0.13816;//0.16077;
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
      // Limits peak torque current per drive motor to prevent battery voltage sag
      // during hard acceleration. 4 motors x 60A = 240A max drive draw.
      // Tune down to 40-50A if brownouts still occur at competition.
      config.CurrentLimits.StatorCurrentLimit = 60.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
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
    public static final double QUEST_X_METERS = Units.inchesToMeters(-11.38);
    public static final double QUEST_Y_METERS = Units.inchesToMeters(1.56);
    public static final double QUEST_Z_METERS = Units.inchesToMeters(13.75);
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
    public static final int HOOD_MOTOR_ID = 22;
    public static final int TURRET_MOTOR_ID = 23;

    public static final int HOOD_CANCODER_ID = 25;
    public static final int TURRET_CANCODER_ID = 26;

    public static final int HOOD_BEAM_BREAK_DIO = 2;   // TODO: confirm DIO port
    public static final int HALL_SENSOR_LEFT_DIO = 4;  // left hard stop — primary home sensor
    public static final int HALL_SENSOR_RIGHT_DIO = 3; // straight-forward mid-point sensor (optional)

    // Gear train: Kraken X44 (1:1) → 20T pinion → 200T turret ring
    // Motor rotates 10x for every 1 turret rotation.
    public static final double TURRET_GEAR_RATIO = 10.0; // motor rot / turret rot

    // 350 degrees in 1 second = 0.972 turret rot/sec × 10 = 9.72 motor rot/sec.
    // Kraken X44 free speed ~100 rot/sec so cruise is ~10% throttle — plenty of margin.
    // Acceleration of 3× cruise gives ~0.33 sec to reach full speed from rest.
    // TODO: restore after hall sensor and soft limits are verified on hardware
    // public static final double TURRET_CRUISE_VELOCITY_RPS  = 9.72;  // motor rot/sec
    // public static final double TURRET_ACCELERATION_RPS2    = 30.0;  // motor rot/sec^2

    // Safety speed: 350 degrees in ~10 seconds for initial commissioning
    public static final double TURRET_CRUISE_VELOCITY_RPS  = 0.972; // motor rot/sec — safety
    public static final double TURRET_ACCELERATION_RPS2    = 3.0;   // motor rot/sec^2 — safety
    public static final double TURRET_JERK_RPS3            = 0.0;   // 0 = disabled (tune later)

    // Motor inversion — TODO: confirm directions on hardware
    public static final boolean TURRET_MOTOR_INVERTED   = false;
    public static final boolean HOOD_MOTOR_INVERTED     = false;
    public static final boolean FLYWHEEL_MOTOR_INVERTED = false; // both flywheels use this

    // Turret homing: slow left until left hall sensor fires, then zero the encoder
    public static final double TURRET_HOME_SPEED_PERCENT = 0.10; // slow creep for safety

    // Phase-advance enablement — change to advance to the next phase
    // PHASE_1: fixed/manual setpoint, fire interlock only
    // PHASE_2: turret tracks target, robot must be near-stationary to fire
    // PHASE_3: turret tracks while driving, fire only when chassis slows below threshold
    // PHASE_4: stub — same behavior as PHASE_3 (velocity comp math not yet implemented)
    public enum TurretPhase { PHASE_1_STATIC, PHASE_2_TRACKING, PHASE_3_DECEL_SHOOT, PHASE_4_ON_THE_MOVE }
    public static final TurretPhase CURRENT_PHASE = TurretPhase.PHASE_1_STATIC;

    // Phase 3+: only allow firing when chassis is below this speed
    public static final double CHASSIS_SPEED_FIRE_THRESHOLD_MPS = 0.15; // TODO: tune

    // "Ready to shoot" tolerances — all must pass for isReadyToShoot() to return true
    public static final double TURRET_ON_TARGET_TOLERANCE_ROT  = 0.02; // ~7 degrees
    public static final double HOOD_ON_TARGET_TOLERANCE_ROT    = 0.01; // TODO: tune in rotations
    public static final double FLYWHEEL_ON_TARGET_TOLERANCE_PCT = 0.03; // within 3% of setpoint

    // Shooter geometry:
    // Two hex shafts (front roller and top/back roller) separated by a fixed distance.
    // Front roller position is fixed relative to the turret plate.
    // Top roller pivots around the front roller as the hood rotates — so the ball exit
    // point shifts in both height and forward offset as the hood angle changes.
    // Hood range: ~35 deg (pointing forward, long pass) to ~85 deg (steep lob, short range).
    // The front roller is the effective origin for all distance/trajectory math.
    // Hood angle bakes the exit-point offset into each shot table row implicitly —
    // do NOT assume a fixed launch height when adding ballistic math later.
    // TODO: measure HOOD_SHAFT_SEPARATION_M (distance between the two hex shaft centers)
    //       and HOOD_FRONT_ROLLER_HEIGHT_M (height of front roller above ground) on hardware.
    public static final double HOOD_SHAFT_SEPARATION_M    = 0.0; // TODO: measure (meters)
    public static final double HOOD_FRONT_ROLLER_HEIGHT_M = 0.0; // TODO: measure (meters)
    public static final double HOOD_ANGLE_MIN_DEG         = 35.0; // hood all the way forward
    public static final double HOOD_ANGLE_MAX_DEG         = 85.0; // hood all the way back

    // Shot lookup table — distance (meters) -> flywheel percent -> hood rotations
    // Distance is measured from the front roller (fixed turret origin) to the target.
    // Hood rotations correspond to the CANcoder reading at the desired angle — measure on hardware.
    // Data from initial ChatGPT estimate with our flywheel build; tune each row on hardware.
    // [ ] CLOSE row: measure actual distance + verify RPM/hood at ~1.2m from target
    // [ ] MID   row: measure actual distance + verify RPM/hood at ~3.5m from target
    // [ ] FAR   row: measure actual distance + verify RPM/hood at ~5.5m from target
    // Max flywheel RPM assumed 5400 (Kraken X44 free speed ~6000, loaded ~90%) — adjust if wrong.
    // Hood rotations are placeholder — measure with CANcoder at each distance and replace.
    public static final double[] SHOT_TABLE_DISTANCES_M        = { 1.2,    3.5,    5.5   };
    public static final double[] SHOT_TABLE_FLYWHEEL_FRONT_PCT = { 0.435,  0.546,  0.638 }; // 2350/5400, 2950/5400, 3490/5400
    public static final double[] SHOT_TABLE_FLYWHEEL_BACK_PCT  = { 0.465,  0.584,  0.683 }; // 2515/5400, 3155/5400, 3690/5400
    public static final double[] SHOT_TABLE_HOOD_ROTATIONS     = { 0.181,  0.153,  0.139 }; // TODO: replace — 65/360, 55/360, 50/360 placeholder
  }

  // Singulator hardware IDs and tuning constants (feeds balls one at a time into flywheels)
  public static final class Singulator {
    public static final int MOTOR_ID = 24; // REV NEO on SparkMax
    public static final int BEAM_BREAK_DIO = 28; // beam break at ball staging point

    // TODO: flip to true if motor runs backwards on first test
    public static final boolean MOTOR_INVERTED = false;

    public static final double FEED_SPEED    =  0.50; // TODO: tune on hardware
    public static final double REVERSE_SPEED = -0.40;

    // SparkMax smart current limit (stall protection)
    public static final int CURRENT_LIMIT_AMPS = 30;
  }

  // Spindexer hardware IDs and tuning constants (cone spinner that feeds balls into singulator)
  public static final class Spindexer {
    public static final int MOTOR_ID = 27; // REV NEO on SparkMax

    // TODO: flip to true if motor runs backwards on first test
    public static final boolean MOTOR_INVERTED = false;

    public static final double FORWARD_SPEED = 0.40; // normal feed speed - TODO: tune
    public static final double REVERSE_SPEED = -0.30; // unjam pulse speed

    // Agitator auto-reverse: if velocity stays below this for AGITATE_LOOP_THRESHOLD loops,
    // fire a short reverse pulse to jostle stuck balls
    public static final double STALL_VELOCITY_RPS     = 0.5;  // TODO: tune on hardware
    public static final int    AGITATE_LOOP_THRESHOLD = 25;   // ~500ms at 50Hz
    public static final int    AGITATE_PULSE_LOOPS    = 10;   // ~200ms reverse pulse

    // SparkMax smart current limit (stall protection)
    public static final int CURRENT_LIMIT_AMPS = 30;
  }

  // Intake mechanism hardware IDs and tuning constants
  public static final class Intake {
    public static final int INTAKE_ROLLER_MOTOR_ID    = 40; // Kraken X44 - roller spin
    public static final int INTAKE_EXTENSION_MOTOR_ID = 41; // Kraken X44 - arm extend/retract

    // TODO (checklist item 3/4): flip to true if motor runs backwards on first test
    public static final boolean EXTENSION_MOTOR_INVERTED = false;
    public static final boolean ROLLER_MOTOR_INVERTED    = false;

    // RoboRIO DIO ports for retract limit switches (both must trip = fully retracted)
    public static final int RETRACT_LIMIT_SWITCH_A_DIO = 0; // TODO: confirm DIO port
    public static final int RETRACT_LIMIT_SWITCH_B_DIO = 1; // TODO: confirm DIO port

    // Extension arm target in motor rotations (set via TunerX after gearing confirmed)
    // Positive = extending out over bumper; 0 = fully retracted (limit switch home)
    public static final double EXTENSION_TARGET_ROTATIONS = 10.0; // TODO: tune on hardware

    // Duty cycle output limits for extension movement (no position control until gear ratio known)
    public static final double EXTEND_SPEED  =  0.10; // positive = extending out
    public static final double RETRACT_SPEED = -0.10; // negative = retracting in

    // Roller duty cycle outputs
    public static final double ROLLER_INTAKE_SPEED  =  0.60; // intaking
    public static final double ROLLER_REVERSE_SPEED = -0.40; // ejecting

    // Current limits
    // Extension (Kraken): stator caps torque current; supply caps battery draw.
    // Roller (NEO SparkMax): uses smart current limit only.
    public static final double EXTENSION_STATOR_LIMIT_AMPS = 20.0;
    public static final double EXTENSION_SUPPLY_LIMIT_AMPS = 15.0;
    public static final int    ROLLER_CURRENT_LIMIT_AMPS   = 40;

    // Velocity threshold for stall detection on the extension motor (rotations per second).
    // If the motor is commanded to move but velocity stays below this for ~200ms, it is stalled.
    // Current-based detection is unreliable because the stator limit clamps current before
    // it can distinguish a stall from normal load.
    public static final double EXTENSION_STALL_VELOCITY_RPS = 0.5; // tune on hardware
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
    
    public static final double MAX_MODULE_SPEED_MPS = Swerve.MAX_TRANSLATION_SPEED_MPS;
    
    // Starting pose validation (how close robot must be to expected start)
    public static final double STARTING_POSE_TOLERANCE_METERS = 0.15;
    public static final double STARTING_POSE_TOLERANCE_DEGREES = 5.0;
    public static final double VISION_INITIALIZATION_TIMEOUT_SECONDS = 7.0;
    public static final Pose2d DEFAULT_FALLBACK_POSE = StartingPositions.BLUE_REBUILT_RIGHT_CORNER;
    
    // Post-path correction timeout (SmartDrive precision phase)
    public static final double POST_PATH_CORRECTION_TIMEOUT_S = 3.0;
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

    // Red alliance mirrored poses (X_red = FIELD_LENGTH - X_blue, rotation + 180 deg)
    public static final Pose2d RED_LEFTBUMP_ALLIANCE_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_LEFTBUMP_ALLIANCE_STAGING.getX(),
        BLUE_LEFTBUMP_ALLIANCE_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
    public static final Pose2d RED_LEFTBUMP_NEUTRAL_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_LEFTBUMP_NEUTRAL_STAGING.getX(),
        BLUE_LEFTBUMP_NEUTRAL_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
    public static final Pose2d RED_LEFTBUMP_OPPONENT_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_LEFTBUMP_OPPONENT_STAGING.getX(),
        BLUE_LEFTBUMP_OPPONENT_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));

    public static final Pose2d RED_RIGHTBUMP_ALLIANCE_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_RIGHTBUMP_ALLIANCE_STAGING.getX(),
        BLUE_RIGHTBUMP_ALLIANCE_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
    public static final Pose2d RED_RIGHTBUMP_NEUTRAL_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_RIGHTBUMP_NEUTRAL_STAGING.getX(),
        BLUE_RIGHTBUMP_NEUTRAL_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
    public static final Pose2d RED_RIGHTBUMP_OPPONENT_STAGING = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_RIGHTBUMP_OPPONENT_STAGING.getX(),
        BLUE_RIGHTBUMP_OPPONENT_STAGING.getY(),
        Rotation2d.fromDegrees(225.0));
  }

  // Field positions (all blue alliance - red is mirrored automatically)
  public static final class StartingPositions {
    
    
    // TUNING POSITION
    public static final Pose2d PID_TUNING_POSITION = new Pose2d(1.270, 2.230, Rotation2d.fromDegrees(0.0));
    // LED CALIBRATION TEST POSITION (from actual Limelight reading)
    public static final Pose2d LED_TEST_POSITION = new Pose2d(1.2748, 2.3987, Rotation2d.fromDegrees(-6.56));
    
    // Staging poses (Phase 1: PathPlanner pathfind targets)
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
    
   
    // AUTO RESET POSE STAGING
    public static final Pose2d BLUE_AUTO_START_POS_FAR_RIGHT = new Pose2d(5.533, 1.185, Rotation2d.fromDegrees(180.0));
    
    // Precision poses (Phase 2: AutoPilot final targets)
    
    // AUTO RESET POSE PRECISE
    public static final Pose2d PRECISE_BLUE_AUTO_START_POS_FAR_RIGHT = new Pose2d(6.033, 0.985, Rotation2d.fromDegrees(180.0));
  }

  // Field constants used for fixed target mirroring
  public static final class Field {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(650.12); // 2026 AndyMark perimeter
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(316.64);
    public static final double ALLIANCE_ZONE_LENGTH_METERS = Units.inchesToMeters(158.60);
  }

  // Hub centers for aiming
  public static final class HubCenters {
    public static final Pose2d BLUE_HUB_CENTER = new Pose2d(4.612, 4.022, Rotation2d.fromDegrees(0.0));
    public static final Pose2d RED_HUB_CENTER = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_HUB_CENTER.getX(),
        BLUE_HUB_CENTER.getY(),
        Rotation2d.fromDegrees(180.0));
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

  // Pass targets for alliance-zone handoff
  public static final class PassTargets {
    public static final Pose2d BLUE_PASS_TARGET_LEFT = new Pose2d(3.46, 5.83, Rotation2d.fromDegrees(0.0));
    public static final Pose2d BLUE_PASS_TARGET_RIGHT = new Pose2d(3.46, 2.21, Rotation2d.fromDegrees(0.0));

    public static final Pose2d RED_PASS_TARGET_LEFT = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_PASS_TARGET_LEFT.getX(),
        BLUE_PASS_TARGET_LEFT.getY(),
        Rotation2d.fromDegrees(180.0));
    public static final Pose2d RED_PASS_TARGET_RIGHT = new Pose2d(
        Field.FIELD_LENGTH_METERS - BLUE_PASS_TARGET_RIGHT.getX(),
        BLUE_PASS_TARGET_RIGHT.getY(),
        Rotation2d.fromDegrees(180.0));
  }

  // Fixed target poses for turret aiming (blue alliance, red mirrored later)
  public static final class TurretTargets {
    public static final Pose2d BLUE_PASS_TARGET_LEFT = new Pose2d(3.46, 5.83, Rotation2d.fromDegrees(0.0));
    
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