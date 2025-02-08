package frc.robot.subsystems.wrist;

// Wrist position when elevator down, max is 0.2 and about level with the ground

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration; 
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
    private final TalonFX wristMotor;
    private final PositionVoltage positionRequest;
    
    // Constants
    private static final int WRIST_MOTOR_ID = 20; // Update with your CAN ID
    private static final double WRIST_GEAR_RATIO = 75.0; // Update with your gear ratio
    private static final double ENCODER_TO_MECHANISM_RATIO = 1.0; // Update if encoder gearing differs
    
    // Soft limits in rotations
    private static final double MIN_POSITION = 0.0; // Adjust based on mechanism 
    private static final double MAX_POSITION = 0.2;  // Adjust based on mechanism

    public WristSubsystem() {
        wristMotor = new TalonFX(WRIST_MOTOR_ID, "rio");
        positionRequest = new PositionVoltage(0).withSlot(0);
        
        configureMotor();
        
        /*
        // Verify encoder is present
        var absoluteEncoder = wristMotor.getPosition().getValue();
        if (Double.isNaN(absoluteEncoder)) {
            System.out.println("WARNING: Wrist encoder not detected!");
            SmartDashboard.putBoolean("Wrist/EncoderConnected", false);
        } else {
            SmartDashboard.putBoolean("Wrist/EncoderConnected", true);
        }
             */
    
    }

        // Add current limiting configs

    private void configureMotor() {
        
        wristMotor.getConfigurator().apply(new TalonFXConfiguration());

        var motorConfig = new TalonFXConfiguration();
        
        // Add current limits to protect mechanism
        var currentLimits = motorConfig.CurrentLimits;
        currentLimits.StatorCurrentLimit = 10; // Adjust based on your mechanism
        currentLimits.StatorCurrentLimitEnable = true;
        
        // Add supply current limits
        currentLimits.SupplyCurrentLimit = 10;
        currentLimits.SupplyCurrentLimitEnable = true;

        // Configure motor output
        var motorOutputConfigs = motorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        // Gravity Type as Elevator_Static or Arm_Cosine

        // Configure feedback/scaling
        var feedback = motorConfig.Feedback;
        feedback.SensorToMechanismRatio = WRIST_GEAR_RATIO * ENCODER_TO_MECHANISM_RATIO;
        
        // Configure soft limits 
        var softLimits = motorConfig.SoftwareLimitSwitch;
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = MAX_POSITION;
        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = MIN_POSITION;


        // Adjust PID gains - start conservative
        var slot0 = motorConfig.Slot0;
        slot0.GravityType = GravityTypeValue.Arm_Cosine; /* .Elevator_Static | .Arm_Cosine */
        slot0.kP = 10.0; // Start lower than 80 for testing
        slot0.kI = 0.00;  
        slot0.kD = 0.00; // 0.5 Add some derivative to dampen oscillation
        slot0.kG = 0.00; // .15 Enable gravity compensation
        slot0.kV = 0.00;// Velocity Feedforward .12
        slot0.kS = 0.00; //Static Feedforward .25

        /* Arm Examples */
        // armGains0.kP = 0.50; /* Proportional Gain */
        // armGains0.kI = 0.00; /* Integral Gain */
        // armGains0.kD = 0.00; /* Derivative Gain */
        // armGains0.kV = 0.00; /* Velocity Feed Forward Gain */
        // armGains0.kS = 0.00; /* Static Feed Forward Gain */
        // armGains0.kA = 0.00; /* Acceleration Feedforward */
        // armGains0.kG = 0.00; /* Gravity Feedfoward */


        wristMotor.getConfigurator().apply(motorConfig);
    }

    // Add methods for direct state control
    public void setState(WristStates state) {
        setPosition(state.getWristPosition());
    }

    // Add safety methods
    public boolean isAtSoftLimit() {
        double pos = getWristPosition();
        return pos <= MIN_POSITION || pos >= MAX_POSITION;
    }

    // Add these methods that were referenced but missing
    public void setPosition(double targetPositionRotations) {
        wristMotor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    public double getWristPosition() {
        return wristMotor.getPosition().getValueAsDouble();
    }
    /*
     * public double getWristPosition() {
    return wristMotor.getPosition().getValue().in(com.ctre.phoenix6.configs.PositionUnit.ROTATIONS);
}
     */

    public boolean atPosition() {
        return Math.abs(wristMotor.getClosedLoopError().getValue()) < 0.02; // Tune threshold
    }

    public void stop() {
        wristMotor.stopMotor();
    }

    // Enhanced telemetry
    @Override
    public void periodic() {
 // Basic position telemetry
//  SmartDashboard.putNumber("Wrist/Position", getWristPosition());
//  SmartDashboard.putNumber("Wrist/Target", positionRequest.Position);
//  SmartDashboard.putBoolean("Wrist/AtPosition", atPosition());
 
 // Diagnostic data
//  SmartDashboard.putNumber("Wrist/StatorCurrent", wristMotor.getStatorCurrent().getValue());
//  SmartDashboard.putNumber("Wrist/SupplyCurrent", wristMotor.getSupplyCurrent().getValue());
//  SmartDashboard.putNumber("Wrist/Voltage", wristMotor.getMotorVoltage().getValue());
//  SmartDashboard.putBoolean("Wrist/AtLimit", isAtSoftLimit());
 
 // Error tracking
//  SmartDashboard.putNumber("Wrist/ClosedLoopError", wristMotor.getClosedLoopError().getValue());
//  SmartDashboard.putNumber("Wrist/ClosedLoopReference", wristMotor.getClosedLoopReference().getValue());
    }

    public void setWristZero() {
        // Set the current position to be considered "zero"
        wristMotor.setPosition(0);
    
        // Log that we zeroed the sensor
        System.out.println("Wrist encoder zeroed");
        SmartDashboard.putBoolean("Wrist/WasZeroed", true);
    }
}