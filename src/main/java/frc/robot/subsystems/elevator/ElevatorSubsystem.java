package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    // Subsystem Modes
    public enum ElevatorMode {
        PERFORMANCE, // High-speed, competition mode
        SAFETY // Reduced speed, testing/practice mode
    }

    // Hardware
    private final TalonFX elevatorLeader;
    private final TalonFX elevatorFollower;

    // Control Requests
    private MotionMagicVoltage motionMagicRequest;
    private final VoltageOut manualVoltageRequest;

    // Mode Tracking
    private ElevatorMode currentMode = ElevatorMode.SAFETY; // Default to safety mode
    private double targetPosition = 0.0;

    // Key Hardware Constants
    private static final int ELEVATOR_LEAD_ID = 24;
    private static final int ELEVATOR_FOLLOW_ID = 23;
    private static final String CAN_BUS_NAME = "rio";
    private static final double GEAR_RATIO = 9.0;

    // Soft Limits (in rotations)
    private static final double MAX_HEIGHT = 1.0;
    private static final double MIN_HEIGHT = 0.0;

    // Performance Mode Motion Magic Settings
    private static final double PERFORMANCE_CRUISE_VELOCITY = 160;
    private static final double PERFORMANCE_ACCELERATION = 160;
    private static final double PERFORMANCE_JERK = 500;

    // Safety Mode Motion Magic Settings
    private static final double SAFETY_CRUISE_VELOCITY = 80;
    private static final double SAFETY_ACCELERATION = 80;
    private static final double SAFETY_JERK = 200;

    // Increment Settings
    private static final double FINE_INCREMENT = 0.025; // Small adjustments
    private static final double COARSE_INCREMENT = 0.10; // Large adjustments

    // Tolerance/Deadband Settings
    private static final double POSITION_TOLERANCE = 0.02;
    private static final double DEADBAND = 0.02;

    // Preset Positions (in rotations)
    private static final double POSITION_HOME = 0.0;
    private static final double POSITION_LOW = 0.2;
    private static final double POSITION_MID = 0.5;
    private static final double POSITION_HIGH = 0.9;

    // Configure gains for each slot
    private final Slot0Configs performanceGains = new Slot0Configs()
            .withKP(4.0)
            .withKI(0.01)
            .withKD(0.10)
            .withKS(0.50)
            .withKV(0.12)
            .withKG(0.50);

    private final Slot1Configs safetyGains = new Slot1Configs()
            .withKP(1.0)
            .withKI(0.01)
            .withKD(0.10)
            .withKS(0.25)
            .withKV(0.12)
            .withKG(0.25);

    private final Slot2Configs incrementalGains = new Slot2Configs()
            .withKP(1.0)
            .withKI(0.01)
            .withKD(0.10)
            .withKS(0.25)
            .withKV(0.12)
            .withKG(0.12);

    public ElevatorSubsystem() {
        // Initialize motors
        elevatorLeader = new TalonFX(ELEVATOR_LEAD_ID, CAN_BUS_NAME);
        elevatorFollower = new TalonFX(ELEVATOR_FOLLOW_ID, CAN_BUS_NAME);

        // Initialize control requests
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(1).withEnableFOC(true);
        manualVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        configureMotors();
        setMode(ElevatorMode.SAFETY); // Start in safety mode
    }

    private void configureMotors() {
        // Create and configure leader settings
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();

        // Configure motor output
        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Configure feedback
        leaderConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        // Configure soft limits
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_HEIGHT * GEAR_RATIO;
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_HEIGHT * GEAR_RATIO;

        // Configure current limits
        leaderConfig.CurrentLimits.StatorCurrentLimit = 40;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply gains to slots
        elevatorLeader.getConfigurator().apply(performanceGains); // Slot 0
        elevatorLeader.getConfigurator().apply(safetyGains);     // Slot 1
        elevatorLeader.getConfigurator().apply(incrementalGains); // Slot 2

        // Apply leader config
        elevatorLeader.getConfigurator().apply(leaderConfig);

        // Configure follower
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.CurrentLimits.StatorCurrentLimit = 40;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        elevatorFollower.getConfigurator().apply(followerConfig);
        elevatorFollower.setControl(new Follower(ELEVATOR_LEAD_ID, true));
    }

    public void setMode(ElevatorMode mode) {
        if (mode == currentMode) return;
        
        currentMode = mode;
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        elevatorLeader.getConfigurator().refresh(config);
        
        // Update motion magic settings based on mode
        if (mode == ElevatorMode.PERFORMANCE) {
            config.MotionMagic.MotionMagicCruiseVelocity = PERFORMANCE_CRUISE_VELOCITY;
            config.MotionMagic.MotionMagicAcceleration = PERFORMANCE_ACCELERATION;
            config.MotionMagic.MotionMagicJerk = PERFORMANCE_JERK;
            motionMagicRequest = motionMagicRequest.withSlot(0);
        } else {
            config.MotionMagic.MotionMagicCruiseVelocity = SAFETY_CRUISE_VELOCITY;
            config.MotionMagic.MotionMagicAcceleration = SAFETY_ACCELERATION;
            config.MotionMagic.MotionMagicJerk = SAFETY_JERK;
            motionMagicRequest = motionMagicRequest.withSlot(1);
        }
        
        elevatorLeader.getConfigurator().apply(config);
        Logger.recordOutput("Elevator/Mode", mode.toString());
    }

    public void incrementPosition(boolean up) {
        double currentPos = getPosition();
        double increment = up ? FINE_INCREMENT : -FINE_INCREMENT;
        
        // Use slot 2 for incremental movement
        motionMagicRequest = motionMagicRequest.withSlot(2);
        
        double newTarget = MathUtil.clamp(
            currentPos + increment,
            MIN_HEIGHT,
            MAX_HEIGHT
        );
        
        setPosition(newTarget);
        
        // Reset back to current mode's slot for next movement
        motionMagicRequest = motionMagicRequest.withSlot(currentMode == ElevatorMode.PERFORMANCE ? 0 : 1);
    }

    public void setPosition(double positionRotations) {
        targetPosition = MathUtil.clamp(positionRotations, MIN_HEIGHT, MAX_HEIGHT);
        
        // Only move if change is larger than deadband
        if (Math.abs(targetPosition - getPosition()) > DEADBAND) {
            elevatorLeader.setControl(motionMagicRequest.withPosition(targetPosition * GEAR_RATIO));
        }
    }

    public double getPosition() {
        return elevatorLeader.getPosition().getValueAsDouble() / GEAR_RATIO;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public boolean isAtPosition(double targetPosition) {
        return Math.abs(getPosition() - targetPosition) < POSITION_TOLERANCE;
    }

    public void setManualOutput(double percentOutput) {
        // Apply deadband and clamp output
        if (Math.abs(percentOutput) < DEADBAND) {
            percentOutput = 0;
        }
        
        double voltage = MathUtil.clamp(percentOutput * 12.0, -12.0, 12.0);
        elevatorLeader.setControl(manualVoltageRequest.withOutput(voltage));
    }

    @Override
    public void periodic() {
        // Mode and State Information
        Logger.recordOutput("Elevator/Mode", currentMode.toString());
        Logger.recordOutput("Elevator/Position/Current", getPosition());
        Logger.recordOutput("Elevator/Position/Target", targetPosition);
        Logger.recordOutput("Elevator/AtTarget", isAtPosition(targetPosition));
        
        // Motor Telemetry
        Logger.recordOutput("Elevator/Voltage", elevatorLeader.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Elevator/Current", elevatorLeader.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Elevator/Velocity", elevatorLeader.getVelocity().getValueAsDouble() / GEAR_RATIO);
    }
}