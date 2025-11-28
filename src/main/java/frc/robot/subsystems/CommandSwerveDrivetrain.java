package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

// CTRE Phoenix 6 swerve drivetrain wrapper - extends auto-generated TunerSwerveDrivetrain
// Implements WPILib Subsystem interface for command-based programming
// This class is mostly boilerplate - CTRE handles low-level module control
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5ms simulation update rate
    private Notifier m_simNotifier = null; // Background thread for simulation
    private double m_lastSimTime;

    // Alliance perspective - defines "forward" direction for field-relative driving
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero; // Blue faces 0deg
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg; // Red faces 180deg
    private boolean m_hasAppliedOperatorPerspective = false; // Track if perspective set

    // SysId characterization requests - used to find motor PID/FF gains
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    // SysId routine for drive motors - finds kS, kV, kA for feedforward
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default ramp rate (1 V/s)
            Volts.of(4), // Max 4V to prevent brownout during characterization
            null, // Default timeout (10s)
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString()) // Log to CTRE
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    // SysId routine for steering motors - finds steering PID gains
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7), // Higher voltage for steer (lighter motors)
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    // SysId routine for rotation heading controller - finds rotation PID for FieldCentricFacingAngle
    // Note: Uses rad/s but SysId only understands "volts" so we pretend volts = rad/s
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second), // Ramp rate (actually rad/s^2)
            Volts.of(Math.PI), // Max rate (actually rad/s)
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts))); // "Volts" = rad/s
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation; // Default to drive characterization

    // Constructor - initializes swerve drivetrain from Tuner X constants
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread(); // Start physics simulation if running in sim
        }
    }

    // Constructor with custom odometry update rate (default: 250Hz CAN FD, 100Hz CAN 2.0)
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // Constructor with custom odometry and vision standard deviations for pose estimator tuning
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation, // [x, y, theta] trust for wheel odometry
        Matrix<N3, N1> visionStandardDeviation, // [x, y, theta] trust for vision measurements
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // Apply a swerve drive request (field-centric, robot-centric, point-at-target, etc.)
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    // Run SysId Quasistatic test (slow ramp) for current routine
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    // Run SysId Dynamic test (fast step) for current routine
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        // Apply alliance-specific field orientation (blue = 0deg, red = 180deg)
        // This ensures "forward" always points toward opponent alliance wall
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    // Start simulation thread for physics-based testing (runs faster than real-time for better PID behavior)
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage()); // Update physics sim
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // Add vision measurement with timestamp conversion (WPILib uses FPGA time, we use current time)
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // Add vision measurement with custom standard deviations (trust levels)
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs // [x, y, theta] - lower = more trust
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}
