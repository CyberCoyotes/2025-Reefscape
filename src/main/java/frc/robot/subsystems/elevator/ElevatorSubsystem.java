package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
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

        // Initialize control requests
        motionMagicRequest = new MotionMagicVoltage(0)
                .withSlot(0)
                .withEnableFOC(true);

        manualVoltageRequest = new VoltageOut(0)
                .withEnableFOC(true);

        // Initialize and reset the elevator position
        initializeOnStartup();
        configureMotors();
        // configureSafetyMotors();
    }

    private void configureMotors() {
        // Create configurations for both motors
        TalonFXConfiguration leadConfig = new TalonFXConfiguration();

        // TODO is this needed?
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
        // leadConfig.MotionMagic.MotionMagicCruiseVelocity =
        // ElevatorConstants.CRUISE_VELOCITY;
        // leadConfig.MotionMagic.MotionMagicAcceleration =
        // ElevatorConstants.ACCELERATION;
        // leadConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.JERK;

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

        // Configure gravity compensation for vertical mechanism
        leadConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

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
        // elevatorFollower.getConfigurator().apply(followerConfig);

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
     * Sets the elevator position using motion magic
     * 
     * @param positionRotations The target position in rotations
     */
    public void setPosition(double positionRotations) {
        // Apply deadband to avoid small movements
        if (Math.abs(positionRotations - getPosition()) < ElevatorConstants.DEADBAND) {
            return;
        }

        elevatorLeader.setControl(motionMagicRequest.withPosition(positionRotations * GEAR_RATIO));
    }

    // TODO **NEW** Incremental test 2-16-25
    /**************************************************************************** */
    /*
     * Incrementally adjusts the elevator position by the given amount
     * 
     * @param increment The amount to adjust (positive for up, negative for down)
     */
    public void incrementPosition(double increment) {
        double currentPos = getPosition();
        double newPos = currentPos + increment;

        // Optional: Add bounds checking if needed
        // newPos = Math.min(Math.max(newPos, ElevatorConstants.MIN_HEIGHT),
        // ElevatorConstants.MAX_HEIGHT);

        setPosition(newPos);
    }

    /**
     * Creates a command that incrementally moves the elevator up while the D-pad up
     * is held
     * 
     * @return A command that runs while D-pad up is held
     */
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

    /******************************************************************************* */

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
        return elevatorLeader.getPosition().getValueAsDouble() / GEAR_RATIO;
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

    @Override
    public void periodic() {
        // Log the current elevator position
        Logger.recordOutput("Elevator/Position", getPosition());

        // Log the elevator velocity
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