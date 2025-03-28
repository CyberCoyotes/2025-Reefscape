package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")

public class ElevatorSubsystem extends SubsystemBase {
    // Subsystem Modes
    public enum ElevatorMode {
        PERFORMANCE, // High-speed, competition mode
        SAFETY      // Reduced speed, testing/practice mode
    }

    public enum ElevatorPosition {
        HOME(0.00),
        L1(0.00),
        SCORE_ALGAE(0.45), // Previously .5
        L2(0.45),
        ALGAE2(1.3), // Previously was 1.1 but too low
        L3(1.85),
        INTAKE_CORAL(1.92),
        TRAVEL(1.92),
        ALGAE3(2.65), // previously was 2.45 but too low
        L4(4.2); // Previously 4.7 // 4.6 too tall at practice field

        private final double position;

        ElevatorPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    // Hardware
    private final TalonFX elevatorLeader;
    private final TalonFX elevatorFollower;

    // Control Requests
    private MotionMagicVoltage motionMagicRequest;

    // Mode Tracking
    private ElevatorMode currentMode = ElevatorMode.SAFETY; // Default to safety mode
    private double targetPosition = 0.0;

    // Increment Settings
    public static final double INCREMENT = 0.02; // Small adjustments

    // Tolerance/Deadband Settings
    private static final double POSITION_TOLERANCE = 0.02;
    private static final double DEADBAND = 0.02;

    public ElevatorSubsystem() {
        // Initialize motors
        elevatorLeader = new TalonFX(Constants.ELEVATOR_LEAD_ID, Constants.kCANBus);
        elevatorFollower = new TalonFX(Constants.ELEVATOR_FOLLOW_ID, Constants.kCANBus);

        // Initialize control requests
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

        // Reset the elevator position
        resetElevator();

        // Configure motors
        configureMotors();
    }

    private void configureMotors() {
        // Create configurations for both motors
        TalonFXConfiguration leadConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        // Configure the GravityType for elevator mechanism
        leadConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Configure leader motor PID and FF gains
        leadConfig.Slot0.kP = ElevatorConstants.kP;
        leadConfig.Slot0.kI = ElevatorConstants.kI;
        leadConfig.Slot0.kD = ElevatorConstants.kD;
        leadConfig.Slot0.kV = ElevatorConstants.kV;
        leadConfig.Slot0.kS = ElevatorConstants.kS;
        leadConfig.Slot0.kG = ElevatorConstants.kG;

        // Configure motion magic parameters - FIXED: Use proper acceleration constant
        leadConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.VELOCITY;
        leadConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION; // FIXED: Use ACCELERATION instead of CRUISE_VELOCITY
        leadConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.JERK;

        // Configure soft limits - Enable them for safety
        leadConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // FIXED: Enable soft limits
        leadConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.FORWARD_LIMIT;
        leadConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // FIXED: Enable soft limits
        leadConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.REVERSE_LIMIT;

        // Configure current limits for both motors
        leadConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Changed from false to true
        leadConfig.CurrentLimits.StatorCurrentLimit = 40;

        // Apply leader configuration
        elevatorLeader.getConfigurator().apply(leadConfig);

        // Configure follower motor with its specific settings
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.CurrentLimits.StatorCurrentLimit = 40;

        // Apply follower configuration - FIXED: Use followerConfig instead of leadConfig
        elevatorFollower.getConfigurator().apply(followerConfig);

        // Set up follower to follow leader with opposite direction
        elevatorFollower.setControl(new Follower(Constants.ELEVATOR_LEAD_ID, true));
    }

    public void resetElevator() {
        elevatorLeader.setPosition(0);
    }

    public void setPosition(double positionRotations) {
        targetPosition = positionRotations;

        // FIXED: Removed duplicated setControl calls - only execute if beyond deadband
        if (Math.abs(targetPosition - getPosition()) > DEADBAND) {
            elevatorLeader.setControl(motionMagicRequest.withPosition(targetPosition * ElevatorConstants.GEAR_RATIO));
        }
    }

    public void incrementPosition(boolean up) {
        double currentPos = getPosition();
        double increment = up ? INCREMENT : -INCREMENT;
        double newTarget = (currentPos + increment);
        
        // Make sure we don't exceed soft limits
        if (newTarget >= ElevatorConstants.REVERSE_LIMIT && newTarget <= ElevatorConstants.FORWARD_LIMIT) {
            setPosition(newTarget);
        }
    }

    // This is required for the incremental command 
    public void incrementPosition(double increment) {
        double currentPos = getPosition();
        double newPos = currentPos + increment;
        
        // Make sure we don't exceed soft limits
        // if (newPos >= ElevatorConstants.REVERSE_LIMIT && newPos <= ElevatorConstants.FORWARD_LIMIT) {
            // setPosition(newPos);
        // }
    }

    public double getPosition() {
        return elevatorLeader.getPosition().getValueAsDouble() / ElevatorConstants.GEAR_RATIO;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public boolean isAtPosition(double targetPosition) {
        return Math.abs(getPosition() - targetPosition) < POSITION_TOLERANCE;
    }

    @Override
    public void periodic() {
        // Log diagnostics
        SmartDashboard.putNumber("Elevator/Position/Current", getPosition());
        SmartDashboard.putNumber("Elevator/Position/Target", targetPosition);
        SmartDashboard.putBoolean("Elevator/AtTarget", isAtPosition(targetPosition));
        
        // Add additional diagnostics to help with debugging
        SmartDashboard.putNumber("Elevator/Leader/OutputPercent", elevatorLeader.get());
        SmartDashboard.putNumber("Elevator/Leader/SupplyCurrent", elevatorLeader.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Leader/StatorCurrent", elevatorLeader.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Follower/SupplyCurrent", elevatorFollower.getSupplyCurrent().getValueAsDouble());
    }
}