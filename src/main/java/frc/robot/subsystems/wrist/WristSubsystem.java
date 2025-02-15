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
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(0.160400390625)); // Offset from Tuner X zeroing
        // 0.064208984375 converted; actual 0.160400390625

        // Apply encoder config and wait for completion
        var encoderResult = encoder.getConfigurator().apply(encoderConfig, 0.050);
        if (!encoderResult.isOK()) {
            System.out.println("Failed to configure CANcoder: " + encoderResult.toString());
        }

        // Configure TalonFX
        var motorConfig = new TalonFXConfiguration();

        // Feedback Configuration
        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.RotorToSensorRatio = WristConstants.GEAR_RATIO;
        motorConfig.Feedback.SensorToMechanismRatio = WristConstants.ENCODER_TO_MECHANISM_RATIO;

        // Motion Magic Configuration
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MOTION_MAGIC_VELOCITY;
        motorConfig.MotionMagic.MotionMagicAcceleration = WristConstants.MOTION_MAGIC_ACCELERATION;
        motorConfig.MotionMagic.MotionMagicJerk = WristConstants.MOTION_MAGIC_JERK;

        // Slot 0 Configuration for Motion Magic
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.Slot0.kS = WristConstants.Gains.kS;
        motorConfig.Slot0.kV = WristConstants.Gains.kV;
        motorConfig.Slot0.kA = WristConstants.Gains.kA;
        motorConfig.Slot0.kP = WristConstants.Gains.kP;
        motorConfig.Slot0.kI = WristConstants.Gains.kI;
        motorConfig.Slot0.kD = WristConstants.Gains.kD;
        motorConfig.Slot0.kG = WristConstants.Gains.kG;

        // Current Limits
        motorConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.SUPPLY_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Apply configuration and wait for completion
        var motorResult = motor.getConfigurator().apply(motorConfig, 0.050);
        if (!motorResult.isOK()) {
            System.out.println("Failed to configure TalonFX: " + motorResult.toString());
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
                System.out.println("Warning: Remote sensor configuration failed. Check CANcoder ID and bus.");
                System.out.println("Expected CANcoder ID: " + encoder.getDeviceID());
            }
        } catch (InterruptedException e) {
            System.out.println("Configuration validation interrupted");
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
    
    /**
     * Checks if wrist is in a safe position for elevator movement
     * @return true if safe for elevator movement
     */
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
        // Update SmartDashboard
        Logger.recordOutput("Wrist/Position", getPosition());
        Logger.recordOutput("Wrist/Velocity", getVelocity());
        Logger.recordOutput("Wrist/Target", motionMagicRequest.Position);
        Logger.recordOutput("Wrist/AtTarget", atTargetPosition(WristConstants.POSE_TOLERANCE));
        // SmartDashboard.putBoolean("Wrist/HasFault", anyFault);
        // Log current position and named positions
        Logger.recordOutput("Wrist/Position/Named/ElevatorSafe", WristConstants.Positions.SAFE);
        Logger.recordOutput("Wrist/Position/Named/LoadCoral", WristConstants.Positions.LOAD_CORAL);
        Logger.recordOutput("Wrist/Position/Named/L2", WristConstants.Positions.L2);
        Logger.recordOutput("Wrist/Position/Named/L4", WristConstants.Positions.L4);

        
        // Other wrist telemetry can go here
    }

}