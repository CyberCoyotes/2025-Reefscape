package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorLeader;
    private final TalonFX elevatorFollower;

    // Control requests for different modes
    private MotionMagicVoltage motionMagicRequest;
    private final VoltageOut manualVoltageRequest;

    public ElevatorSubsystem() {
        // Initialize motors
        elevatorLeader = new TalonFX(ElevatorConstants.ELEVATOR_LEAD_ID, ElevatorConstants.kCANBus);
        elevatorFollower = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOW_ID, ElevatorConstants.kCANBus);
        configureMotors();

        // Initialize control requests
        motionMagicRequest = new MotionMagicVoltage(0)
                .withSlot(0)
                .withEnableFOC(true);

        manualVoltageRequest = new VoltageOut(0)
                .withEnableFOC(true);

        // Initialize and reset the elevator position
        initializeOnStartup();
    }

    private void configureMotors() {
        // Create configurations for both motors
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        // Configure Slot 0 - High Performance
        leaderConfig.Slot0.kP = ElevatorConstants.kP;
        leaderConfig.Slot0.kI = ElevatorConstants.kI;
        leaderConfig.Slot0.kD = ElevatorConstants.kD;
        leaderConfig.Slot0.kV = ElevatorConstants.kV;
        leaderConfig.Slot0.kS = ElevatorConstants.kS;
        leaderConfig.Slot0.kG = ElevatorConstants.kG;

        // Configure Slot 1 - Safety/Testing
        leaderConfig.Slot1.kP = ElevatorConstants.SAFETY_kP;
        leaderConfig.Slot1.kI = ElevatorConstants.SAFETY_kI;
        leaderConfig.Slot1.kD = ElevatorConstants.SAFETY_kD;
        leaderConfig.Slot1.kV = ElevatorConstants.SAFETY_kV;
        leaderConfig.Slot1.kS = ElevatorConstants.SAFETY_kS;
        leaderConfig.Slot1.kG = ElevatorConstants.SAFETY_kG;

        // Configure motion magic
        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        leaderConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;
        leaderConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.JERK;

        // Configure soft limits
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.MAX_HEIGHT
                * ElevatorConstants.GEAR_RATIO;
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.MIN_HEIGHT
                * ElevatorConstants.GEAR_RATIO;

        // Configure feedback and motor direction
        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;

        // Configure gravity compensation for vertical mechanism
        // leaderConfig.Voltage. GravityCompensation = GravityTypeValue.Elevator_Static;

        // Inside configureMotors() method, add to both leaderConfig and followerConfig:
        leaderConfig.CurrentLimits.StatorCurrentLimit = 40; // Adjust value based on your motor/load
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        followerConfig.CurrentLimits.StatorCurrentLimit = 40;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply configurations
        elevatorLeader.getConfigurator().apply(leaderConfig);

        // Configure follower motor to oppose the leader
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorFollower.getConfigurator().apply(followerConfig);

        // Set up follower to follow leader with opposite direction
        elevatorFollower.setControl(new Follower(ElevatorConstants.ELEVATOR_LEAD_ID, true));
    }

    /**
     * Resets the elevator encoder to zero.
     */
    public void resetEncoder() {
        elevatorLeader.setPosition(0);
        Logger.recordOutput("Elevator/EncoderReset", 0);
    }

    /**
     * Initializes elevator position when the robot starts.
     */
    private void initializeOnStartup() {
        resetEncoder();
        Logger.recordOutput("Elevator/Initialized", true);
    }

    /**
     * Increment elevator position by specified amount
     * @param incrementRotations Amount to change position by (in rotations)
     */
    public void incrementPosition(double incrementRotations) {
        
        /*
         * Changed getValue() to getValueAsDouble() 
         * when getting the position from the TalonFX
         */

        // Get current position
        Double currentPos = elevatorLeader.getPosition().getValueAsDouble();
        
        // Calculate new target position
        double newTarget = currentPos + incrementRotations;
        
        // Clamp to valid range
        newTarget = MathUtil.clamp(
            newTarget,
            ElevatorConstants.MIN_HEIGHT,
            ElevatorConstants.MAX_HEIGHT
        );
        
        // Set new target using motion magic
        setPosition(newTarget);
    }

    /**
     * Sets the elevator position using motion magic
     * 
     * @param positionRotations The target position in rotations
     */
    public void setPosition(double positionRotations) {
        // Apply deadband to avoid small movements
        if (Math.abs(positionRotations - getPosition()) < ElevatorConstants.DEADBAND) {
            return;
        }

        elevatorLeader.setControl(motionMagicRequest.withPosition(positionRotations * ElevatorConstants.GEAR_RATIO));
    }

    /**
     * Sets manual control voltage
     * 
     * @param percentOutput The percent output (-1 to 1)
     */
    public void setManualOutput(double percentOutput) {
        // Apply deadband to prevent small unintended movements
        if (Math.abs(percentOutput) < ElevatorConstants.DEADBAND) {
            percentOutput = 0;
        }

        // Convert percent to voltage (assuming 12V system)
        double voltage = percentOutput * 12.0;
        elevatorLeader.setControl(manualVoltageRequest.withOutput(voltage));
    }

    /**
     * Gets the current position of the elevator in rotations
     * 
     * @return The current position in rotations
     */
    public double getPosition() {

        /*
         * Changed getValue() to getValueAsDouble() when getting the position from the
         * TalonFX
         */
        return elevatorLeader.getPosition().getValueAsDouble() / ElevatorConstants.GEAR_RATIO;
    }

    /**
     * Checks if the elevator is at the target position
     * 
     * @param targetPosition The target position to check against
     * @return True if at position, false otherwise
     */
    public boolean isAtPosition(double targetPosition) {
        return Math.abs(getPosition() - targetPosition) < ElevatorConstants.POSITION_TOLERANCE;
    }

    public void setSafetyMode(boolean enabled) {
        // Update motion magic request with appropriate slot
        motionMagicRequest = new MotionMagicVoltage(0)
                .withSlot(enabled ? 1 : 0)
                .withEnableFOC(true);

        // Update motion magic configs
        TalonFXConfiguration config = new TalonFXConfiguration();
        elevatorLeader.getConfigurator().refresh(config);

        config.MotionMagic.MotionMagicCruiseVelocity = enabled ? ElevatorConstants.SAFETY_CRUISE_VELOCITY
                : ElevatorConstants.CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = enabled ? ElevatorConstants.SAFETY_ACCELERATION
                : ElevatorConstants.ACCELERATION;
        config.MotionMagic.MotionMagicJerk = enabled ? ElevatorConstants.SAFETY_JERK : ElevatorConstants.JERK;

        elevatorLeader.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        // Position information
        double currentPosition = getPosition();
        Logger.recordOutput("Elevator/Position/Current", currentPosition);
        Logger.recordOutput("Elevator/Position/Named/Base", ElevatorConstants.BASE_POSE);
        Logger.recordOutput("Elevator/Position/Named/L1", ElevatorConstants.L1_POSE);
        Logger.recordOutput("Elevator/Position/Named/L2", ElevatorConstants.L2_POSE);
        Logger.recordOutput("Elevator/Position/Named/L3", ElevatorConstants.L3_POSE);
        Logger.recordOutput("Elevator/Position/Named/L4", ElevatorConstants.L4_POSE);

        // Motor telemetry
        Logger.recordOutput("Elevator/Motor/Velocity",
                elevatorLeader.getVelocity().getValueAsDouble() / ElevatorConstants.GEAR_RATIO);
        Logger.recordOutput("Elevator/Motor/AppliedVoltage", elevatorLeader.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Elevator/Motor/SupplyCurrent", elevatorLeader.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Elevator/Motor/StatorCurrent", elevatorLeader.getStatorCurrent().getValueAsDouble());
    }

}