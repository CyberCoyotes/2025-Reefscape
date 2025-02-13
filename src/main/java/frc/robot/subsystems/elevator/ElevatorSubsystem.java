package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorLeader;
    private final TalonFX elevatorFollower;
    
    // Control requests for different modes
    private final MotionMagicVoltage motionMagicRequest;
    private final VoltageOut manualVoltageRequest;
    
    // Gear ratio (9:1)
    private static final double GEAR_RATIO = 9.0;

    public ElevatorSubsystem() {
        // Initialize motors
        elevatorLeader = new TalonFX(ElevatorConstants.ELEVATOR_LEAD_ID);
        elevatorFollower = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOW_ID);
        
        // Configure the motors
        configureMotors();
        
        // Initialize control requests
        motionMagicRequest = new MotionMagicVoltage(0)
            .withSlot(0)
            .withEnableFOC(true);
            
        manualVoltageRequest = new VoltageOut(0)
            .withEnableFOC(true);
    }
    
    private void configureMotors() {
        // Create configurations for both motors
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        
        // Configure leader motor
        leaderConfig.Slot0.kP = ElevatorConstants.kP;
        leaderConfig.Slot0.kI = ElevatorConstants.kI;
        leaderConfig.Slot0.kD = ElevatorConstants.kD;
        leaderConfig.Slot0.kV = ElevatorConstants.kV;
        leaderConfig.Slot0.kS = ElevatorConstants.kS;
        leaderConfig.Slot0.kG = ElevatorConstants.kG;
        
        // Configure motion magic
        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        leaderConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;
        leaderConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.JERK;
        
        // Configure soft limits
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.MAX_HEIGHT * GEAR_RATIO;
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.MIN_HEIGHT * GEAR_RATIO;
        
        // Configure feedback and motor direction
        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        
        // Configure gravity compensation for vertical mechanism
        // leaderConfig.Voltage. GravityCompensation = GravityTypeValue.Elevator_Static;
        
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
     * Sets the elevator position using motion magic
     * @param positionRotations The target position in rotations
     */
    public void setPosition(double positionRotations) {
        // Apply deadband to avoid small movements
        if (Math.abs(positionRotations - getPosition()) < ElevatorConstants.DEADBAND) {
            return;
        }
        
        elevatorLeader.setControl(motionMagicRequest.withPosition(positionRotations * GEAR_RATIO));
    }
    
    /**
     * Sets manual control voltage
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
     * @return The current position in rotations
     */
    public double getPosition() {

        /* Changed getValue() to getValueAsDouble() when getting the position from the TalonFX */
        return elevatorLeader.getPosition().getValueAsDouble() / GEAR_RATIO;
    }
    
    /**
     * Checks if the elevator is at the target position
     * @param targetPosition The target position to check against
     * @return True if at position, false otherwise
     */
    public boolean isAtPosition(double targetPosition) {
        return Math.abs(getPosition() - targetPosition) < ElevatorConstants.POSITION_TOLERANCE;
    }
}