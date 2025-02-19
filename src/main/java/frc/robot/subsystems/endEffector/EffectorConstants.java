package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EffectorConstants {

    public static final double INTAKE_CORAL = 0.4;   // (-) Power to intake coral
    public static final double SCORE_CORAL = 0.2;
    public static final double SCORE_CORAL_INVERTED = -0.2;    // (-) Power to score coral, may consider changing power magnitude
    public static final double INTAKE_ALGAE = -0.4; // Opposite power of INTAKE
    public static final double SCORE_ALGAE = 0.2;   // 40% power for scoring
    public static final double HOLD_ALGAE = 0.05;    // Low power hold
    public static final double STOP = 0.0;    // Fully stopped


    public static final TalonFXConfiguration EFFECTOR_CONFIG = new TalonFXConfiguration();

    static {
        // Motor output settings
        EFFECTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // EFFECTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        /* TODO: Check if this directiion is correct
        * I also changed the direction of the power in the States
         */ 

        EFFECTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Current limiting settings
        EFFECTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 40.0; // Maximum allowed current
        EFFECTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    

        
        
    }
}
