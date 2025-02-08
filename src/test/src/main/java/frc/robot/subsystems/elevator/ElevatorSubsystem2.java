package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem2 extends SubsystemBase {
    private final TalonFX elevatorMotor;
    private final VoltageOut voltageRequest; // For manual control
    private final PositionVoltage positionRequest; // For position control

    // Define elevator positions (in rotations)
    public static final double BOTTOM_POSITION = 0.0;
    public static final double TOP_POSITION = 100.0; // Adjust based on your elevator

    public ElevatorSubsystem2() {
        // Initialize motor with device ID and CAN bus name
        elevatorMotor = new TalonFX(20, "canivore"); // Adjust ID as needed
        
        // Create control requests
        voltageRequest = new VoltageOut(0);
        positionRequest = new PositionVoltage(0);

        // Configure the elevator motor
        configureMotor();
    }

    private void configureMotor() {
        var config = new TalonFXConfiguration();
        
        // Set motor to brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Configure current limits to prevent brownouts
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Configure voltage compensation
        config.Voltage.PeakForwardVoltage = 11.5;
        config.Voltage.PeakReverseVoltage = -11.5;
        
        // Configure closed-loop gains for position control
        var slot0 = config.Slot0;
        slot0.kP = 24.0; // Adjust these gains for your elevator
        slot0.kI = 0;
        slot0.kD = 0.1;
        slot0.kS = 0.25; // Adjust for static friction
        slot0.kV = 0.12; // Adjust for velocity feedforward
        slot0.kG = 0.45; // Adjust for gravity compensation

        // Apply the configuration
        elevatorMotor.getConfigurator().apply(config);
    }

    // Manual control method for joystick input
    public void manualControl(double speed) {
        // Apply a speed limit for safety
        speed = Math.max(-0.7, Math.min(0.7, speed));
        elevatorMotor.setControl(voltageRequest.withOutput(speed * 12.0));
    }

    // Position control methods
    public void setPosition(double position) {
        elevatorMotor.setControl(positionRequest.withPosition(position));
    }

    /*
    public double getPosition() {
        return elevatorMotor.getPosition().getValue();
    }
    */

    public void stop() {
        elevatorMotor.setControl(voltageRequest.withOutput(0));
    }

    @Override
    public void periodic() {
        // Add any periodic logging here if needed
    }
}