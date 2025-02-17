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
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pabeles.concurrency.ConcurrencyOps.Reset;

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
    private static final double FINE_INCREMENT = 0.02; // Small adjustments
    
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
        elevatorLeader = new TalonFX(ElevatorConstants.ELEVATOR_LEAD_ID);
        elevatorFollower = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOW_ID);

        // Initialize control requests
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(1).withEnableFOC(true);

        // Reset the elevator position
        resetElevator();

        // Configure motors
        configureMotors();
        // configureSafetyMotors();
    }

    private void configureMotors() {
        // Create configurations for both motors
        TalonFXConfiguration leadConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        leadConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Configure leader motor
        leadConfig.Slot0.kP = ElevatorConstants.kP;
        leadConfig.Slot0.kI = ElevatorConstants.kI;
        leadConfig.Slot0.kD = ElevatorConstants.kD;
        leadConfig.Slot0.kV = ElevatorConstants.kV;
        leadConfig.Slot0.kS = ElevatorConstants.kS;
        leadConfig.Slot0.kG = ElevatorConstants.kG;

        // Configure motion magic
        leadConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.TestMode.CRUISE_VELOCITY;
        leadConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.TestMode.ACCELERATION;
        leadConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.TestMode.JERK;

        // Configure soft limits
        // leadConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // leadConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        // ElevatorConstants.MAX_HEIGHT * GEAR_RATIO;
        // leadConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // leadConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        // ElevatorConstants.MIN_HEIGHT * GEAR_RATIO;

        // Configure feedback and motor direction
        leadConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leadConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        // Inside configureMotors() method, add to both leadConfig and followerConfig:
        // leadConfig.CurrentLimits.StatorCurrentLimit = 40; // Adjust value based on
        // your motor/load
        // leadConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // followerConfig.CurrentLimits.StatorCurrentLimit = 40;
        // followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply configurations
        elevatorLeader.getConfigurator().apply(leadConfig);

        // Configure follower motor to oppose the leader
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorFollower.getConfigurator().apply(followerConfig);

        // Set up follower to follow leader with opposite direction
        elevatorFollower.setControl(new Follower(ElevatorConstants.ELEVATOR_LEAD_ID, true));
    }

    private void configureSafetyMotors() {
        // Create configurations for both motors
        TalonFXConfiguration leadConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        // Configure leader motor
        leadConfig.Slot1.kP = ElevatorConstants.TestMode.kP;
        leadConfig.Slot1.kI = ElevatorConstants.TestMode.kI;
        leadConfig.Slot1.kD = ElevatorConstants.TestMode.kD;
        leadConfig.Slot1.kV = ElevatorConstants.TestMode.kV;
        leadConfig.Slot1.kS = ElevatorConstants.TestMode.kS;
        leadConfig.Slot1.kG = ElevatorConstants.TestMode.kG;
        // Configure gravity compensation for vertical mechanism
        leadConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        // Configure motion magic
        leadConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.TestMode.CRUISE_VELOCITY;
        leadConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.TestMode.ACCELERATION;
        leadConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.TestMode.JERK;

        // Configure gravity compensation for vertical mechanism
        leadConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        // Configure soft limits
        // leadConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // leadConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        // ElevatorConstants.MAX_HEIGHT * GEAR_RATIO;
        // leadConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // leadConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        // ElevatorConstants.MIN_HEIGHT * GEAR_RATIO;

        // Configure feedback and motor direction
        leadConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leadConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        // Inside configureMotors() method, add to both leadConfig and followerConfig:
        // leadConfig.CurrentLimits.StatorCurrentLimit = 40; // Adjust value based on
        // your motor/load
        // leadConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // followerConfig.CurrentLimits.StatorCurrentLimit = 40;
        // followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply configurations
        elevatorLeader.getConfigurator().apply(leadConfig);

        // Configure follower
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.CurrentLimits.StatorCurrentLimit = 40;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        elevatorFollower.getConfigurator().apply(followerConfig);
        elevatorFollower.setControl(new Follower(ELEVATOR_LEAD_ID, true));
    }

    // FIXME: This is a temporary solution to reset the elevator position
    public void resetElevator() {
        // Reset encoder position to zero
        elevatorLeader.setPosition(0);
        Logger.recordOutput("Elevator/Reset", "Encoder reset to 0");
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

        elevatorLeader.setControl(motionMagicRequest.withPosition(positionRotations * GEAR_RATIO));
    }

    public Command setPositionCommand(double position) {
        return run(() -> setPosition(position))
                .withName("SetElevatorPosition");
    }

    public void incrementPosition(double increment) {
        double currentPos = getPosition();
        double newPos = currentPos + increment;

        // Optional: Add bounds checking if needed
        // newPos = Math.min(Math.max(newPos, ElevatorConstants.MIN_HEIGHT),
        // ElevatorConstants.MAX_HEIGHT);

        setPosition(newPos);
    }

    public Command incrementUpCommand() {
        return run(() -> incrementPosition(0.02))
                .withName("IncrementElevatorUp");
    }

    /**
     * Creates a command that incrementally moves the elevator down while the D-pad
     * down is held
     * 
     * @return A command that runs while D-pad down is held
     */
    public Command decrementDownCommand() {
        return run(() -> incrementPosition(-0.02))
                .withName("IncrementElevatorDown");
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

        // Log the applied motor voltage
        Logger.recordOutput("Elevator/AppliedVoltage", elevatorLeader.getMotorVoltage().getValueAsDouble());

        // Log the motor current draw
        Logger.recordOutput("Elevator/MotorCurrent", elevatorLeader.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput("Elevator/StatorCurrent", elevatorLeader.getStatorCurrent().getValueAsDouble());

        // Log the target position for debugging
        // Logger.recordOutput("Elevator/TargetPosition", isAtPosition();
    }
}