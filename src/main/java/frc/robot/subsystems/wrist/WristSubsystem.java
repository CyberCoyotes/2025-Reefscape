//         encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
// encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
    // Hardware
    private final TalonFX motor;
    private final CANcoder encoder;

    // Motion Magic Control Request
    private final MotionMagicVoltage motionMagicRequest;

    // Status Signals
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;
    private final StatusSignal<Angle> motorRotorPosition;

    // Fault Signals
    private final StatusSignal<Boolean> fusedSensorOutOfSync;
    private final StatusSignal<Boolean> stickyFusedSensorOutOfSync;
    private final StatusSignal<Boolean> remoteSensorInvalid;
    private final StatusSignal<Boolean> stickyRemoteSensorInvalid;

    public WristSubsystem() {
        motor = new TalonFX(WristConstants.WRIST_ID, WristConstants.kCANBus);
        encoder = new CANcoder(WristConstants.WRIST_ENCODER_ID, WristConstants.kCANBus);

        // Configure CANcoder
        var encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        // encoderConfig.MagnetSensor.AbsoluteSensorRange =
        // AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = 0.160400390625; // Configured offset

        // Apply encoder config and wait for completion
        var encoderResult = encoder.getConfigurator().apply(encoderConfig, 0.050);
        if (!encoderResult.isOK()) {
            System.out.println("[Wrist] Failed to configure CANcoder: " + encoderResult.toString());
        }
        // Configure TalonFX
        var motorConfig = new TalonFXConfiguration();

        // Clear any existing rotor offset
        motorConfig.Feedback.FeedbackRotorOffset = 0;

        // Feedback Configuration
        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.RotorToSensorRatio = 75.0; // Your gear ratio
        motorConfig.Feedback.SensorToMechanismRatio = 1.0;
        motorConfig.Feedback.FeedbackRotorOffset = 0.0; // Ensure no rotor offset

        // Motion Magic Configuration
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 30; // Reduced from 40
        motorConfig.MotionMagic.MotionMagicAcceleration = 60; // Reduced from 80
        motorConfig.MotionMagic.MotionMagicJerk = 600; // Reduced from 800

        // Slot 0 Configuration for Motion Magic
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.Slot0.kS = 0.25; // Static friction compensation
        motorConfig.Slot0.kV = 0.12; // Velocity feedforward
        motorConfig.Slot0.kA = 0.01; // Acceleration feedforward
        motorConfig.Slot0.kP = 1.50; // Reduced from 2.00
        motorConfig.Slot0.kI = 0.00; // No integral gain needed
        motorConfig.Slot0.kD = 0.08; // Increased from 0.05 for more damping
        motorConfig.Slot0.kG = 0.70; // Gravity compensation

        // Current Limits - Enabling with higher limits
        motorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 100;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply configuration and wait for completion
        var motorResult = motor.getConfigurator().apply(motorConfig, 0.050);
        if (!motorResult.isOK()) {
            System.out.println("[Wrist] Failed to configure TalonFX: " + motorResult.toString());
        }

        // Set brake mode
        motor.setNeutralMode(NeutralModeValue.Brake);

        // Initialize Motion Magic request
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

        // Initialize all status signals
        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity();
        encoderPosition = encoder.getPosition();
        encoderVelocity = encoder.getVelocity();
        motorRotorPosition = motor.getRotorPosition();

        // Initialize fault signals
        fusedSensorOutOfSync = motor.getFault_FusedSensorOutOfSync();
        stickyFusedSensorOutOfSync = motor.getStickyFault_FusedSensorOutOfSync();
        remoteSensorInvalid = motor.getFault_RemoteSensorDataInvalid();
        stickyRemoteSensorInvalid = motor.getStickyFault_RemoteSensorDataInvalid();

        // Validate remote sensor configuration
        try {
            Thread.sleep(200); // Give devices time to configure
            BaseStatusSignal.refreshAll(encoderPosition, remoteSensorInvalid);
            if (remoteSensorInvalid.getValue()) {
                DriverStation.reportWarning(
                        "Wrist CANcoder configuration failed. Check ID: " + encoder.getDeviceID(),
                        false);
            }
        } catch (InterruptedException e) {
            System.out.println("[Wrist] Configuration validation interrupted");
        }
    }

    public void setPosition(double targetRotations) {
        targetRotations = MathUtil.clamp(targetRotations,
                WristConstants.MIN_ROTATION,
                WristConstants.MAX_ROTATION);
        motor.setControl(motionMagicRequest.withPosition(targetRotations)
                .withFeedForward(WristConstants.VOLTAGE_FEEDFORWARD));
    }

    public double getPosition() {
        return encoderPosition.refresh().getValue().in(Rotations);
    }

    public double getVelocity() {
        return motorVelocity.refresh().getValue().in(RotationsPerSecond);
    }

    public boolean atTargetPosition(double toleranceRotations) {
        return Math.abs(getPosition() - motionMagicRequest.Position) < toleranceRotations;
    }

    public boolean isSafeForElevator() {
        double currentPosition = getPosition();
        boolean isSafe = currentPosition >= WristConstants.Positions.SAFE;

        if (!isSafe) {
            DriverStation.reportWarning("Wrist position unsafe for elevator movement", false);
        }

        Logger.recordOutput("Wrist/SafetyCheck/ElevatorSafe", isSafe);
        return isSafe;
    }

    public void clearStickyFaults() {
        motor.clearStickyFaults();
        encoder.clearStickyFaults();
    }

    @Override
    public void periodic() {
        // Log essential data
        Logger.recordOutput("Wrist/Position", getPosition());
        Logger.recordOutput("Wrist/Velocity", getVelocity());
        Logger.recordOutput("Wrist/Target", motionMagicRequest.Position);
        Logger.recordOutput("Wrist/AtTarget", atTargetPosition(WristConstants.POSE_TOLERANCE));
        Logger.recordOutput("Wrist/PositionError", motionMagicRequest.Position - getPosition());
        Logger.recordOutput("Wrist/ControlOutput", motor.getMotorVoltage().getValue());

        // Log named positions for telemetry
        Logger.recordOutput("Wrist/Position/Named/ElevatorSafe", WristConstants.Positions.SAFE);
        Logger.recordOutput("Wrist/Position/Named/LoadCoral", WristConstants.Positions.LOAD_CORAL);
        Logger.recordOutput("Wrist/Position/Named/L2", WristConstants.Positions.L2);
        Logger.recordOutput("Wrist/Position/Named/L4", WristConstants.Positions.L4);

        // Log diagnostic data
        Logger.recordOutput("Wrist/Diagnostics/FusedSensorOutOfSync", fusedSensorOutOfSync.refresh().getValue());
        Logger.recordOutput("Wrist/Diagnostics/RemoteSensorInvalid", remoteSensorInvalid.refresh().getValue());
        Logger.recordOutput("Wrist/Diagnostics/RawEncoderPosition",
                encoder.getPosition().refresh().getValue().in(Rotations));
        Logger.recordOutput("Wrist/Diagnostics/MotorRotorPosition",
                motorRotorPosition.refresh().getValue().in(Rotations));
    }
}