package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorLeadMotor;
    private final TalonFX elevatorFollowMotor;
    
    // Motion Magic Control Request
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    
    // Manual Control Request
    private final DutyCycleOut manualOutput = new DutyCycleOut(0);
    
    // Status signals for monitoring
    private final StatusSignal<Double> position; // Will store position in rotations
    private final StatusSignal<Double> velocity; // Will store velocity in rotations per second
    
    public ElevatorSubsystem() {
        elevatorLeadMotor = new TalonFX(ElevatorConstants.ELEVATOR_LEAD_ID);
        elevatorFollowMotor = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOW_ID);
        
        // Configure both motors
        configureMotors();
        
        // Set up status signals with correct units
        position = elevatorLeadMotor.getPosition().clone();
        velocity = elevatorLeadMotor.getVelocity().clone();
        
        // Set up periodic status frame rates
        position.setUpdateFrequency(50);
        velocity.setUpdateFrequency(50);
    }
    
    private void configureMotors() {
        // Create and configure settings for both motors
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        // Motion Magic configurations
        configs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY; // RPS
        configs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION; // RPS/S
        configs.MotionMagic.MotionMagicJerk = ElevatorConstants.JERK; // RPS/S/S
        
        // Voltage-based configuration
        configs.Voltage.PeakForwardVoltage = 12.0;
        configs.Voltage.PeakReverseVoltage = -12.0;
        
        // PID and Feed Forward configuration
        configs.Slot0.kP = ElevatorConstants.kP;
        configs.Slot0.kI = ElevatorConstants.kI;
        configs.Slot0.kD = ElevatorConstants.kD;
        configs.Slot0.kV = ElevatorConstants.kV;
        configs.Slot0.kS = ElevatorConstants.kS;
        configs.Slot0.kG = ElevatorConstants.kG; // Gravity compensation
        
        // Soft limits
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.MAX_HEIGHT;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.MIN_HEIGHT;
        
        // Apply configs to lead motor
        elevatorLeadMotor.getConfigurator().apply(configs);
        
        // Set brake mode
        elevatorLeadMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Configure follow motor to oppose the lead motor
        elevatorFollowMotor.setControl(new Follower(ElevatorConstants.ELEVATOR_LEAD_ID, true));
        elevatorFollowMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    /**
     * Sets the elevator to a specific position using Motion Magic
     * @param targetPosition The target position in rotations
     */
    public void setPosition(double targetPosition) {
        elevatorLeadMotor.setControl(motionMagic.withPosition(targetPosition));
    }
    
    /**
     * Manually control the elevator using percent output
     * @param percentOutput The percent output (-1.0 to 1.0)
     */
    public void setManualOutput(double percentOutput) {
        // Apply deadband and limit the output
        if (Math.abs(percentOutput) < ElevatorConstants.DEADBAND) {
            percentOutput = 0;
        }
        elevatorLeadMotor.setControl(manualOutput.withOutput(percentOutput));
    }
    
    /**
     * Move elevator to preset positions
     */
    public void setToGround() {
        setPosition(ElevatorConstants.GROUND_POSITION);
    }
    
    public void setToMiddle() {
        setPosition(ElevatorConstants.MIDDLE_POSITION);
    }
    
    public void setToHigh() {
        setPosition(ElevatorConstants.HIGH_POSITION);
    }
    
    /**
     * Get the current position of the elevator
     * @return Current position in rotations
     */
    public double getPosition() {
        return position.getValue();
    }
    
    /**
     * Get the current velocity of the elevator
     * @return Current velocity in rotations per second
     */
    public double getVelocity() {
        return velocity.getValue();
    }
    
    /**
     * Check if the elevator is at the target position
     * @param targetPosition The target position to check against
     * @return true if the elevator is within tolerance of the target position
     */
    public boolean isAtPosition(double targetPosition) {
        return Math.abs(getPosition() - targetPosition) < ElevatorConstants.POSITION_TOLERANCE;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Refresh our status signals
        position.refresh();
        velocity.refresh();
    }
}