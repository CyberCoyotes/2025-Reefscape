package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class WristConfigs {

    public static void applyWristConfigs(TalonFX wrist) {

        // Set the default configuration for the arm motor
        wrist.getConfigurator().apply(new TalonFXConfiguration());

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
        softLimits.ForwardSoftLimitThreshold = 0.20; // FORWARD_POSE_LIMIT;
        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = 0.00; // MIN_POSITION;


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
    
        /* EXAMPLE CONFIGS ONLY Gains or configuration of arm motor for config slot 1 */
        /* 
        // set Motion Magic settings
        var armMotionMagic0 = new MotionMagicConfigs();
        armMotionMagic0.MotionMagicCruiseVelocity = 200; 
        armMotionMagic0.MotionMagicAcceleration = 100; 
        armMotionMagic0.MotionMagicJerk = 0;

        var armSoftLimit0 = new SoftwareLimitSwitchConfigs();
        armSoftLimit0.ForwardSoftLimitEnable = true;
        armSoftLimit0.ForwardSoftLimitThreshold = 100;
        armSoftLimit0.ReverseSoftLimitEnable = true;
        armSoftLimit0.ReverseSoftLimitThreshold = 0;

        var armCurrent0 = new CurrentLimitsConfigs();
        armCurrent0.StatorCurrentLimitEnable = true;
        armCurrent0.StatorCurrentLimit = 15;
        armCurrent0.SupplyCurrentLimitEnable = true;
        armCurrent0.SupplyCurrentLimit = 15;
        */

        /* Apply Configs */
        /*
        m_arm.getConfigurator().apply(armGains0, 0.050);
        m_arm.getConfigurator().apply(armMotionMagic0, 0.050);
        m_arm.getConfigurator().apply(armSoftLimit0, 0.050);
        m_arm.getConfigurator().apply(armCurrent0, 0.050);
        */
    }        
} // end of class ArmIO