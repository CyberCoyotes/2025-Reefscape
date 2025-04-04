package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EffectorConstants {

    public static final double INTAKE_CORAL = 0.55;
    public static final double SCORE_CORAL = 0.75;
    public static final double SCORE_SLOW_CORAL = -0.15;
    public static final double SCORE_CORAL_INVERTED = 0.3;

    public static final double INTAKE_ALGAE = -0.40; // Opposite power of INTAKE
    public static final double SCORE_ALGAE = 0.40;
    public static final double HOLD_ALGAE = 0.05; // Low power hold

    public static final double SCORE_WAIT = 0.5; // Time to wait after scoring before stopping
    
    public static final double STOP = 0.0; // Fully stopped

    // public static final double INTAKE_CURRENT_AMPS = 5.0; // Start low and tune up
    // public static final double SCORE_CURRENT_AMPS = -5.0; // Negative for opposite direction

    public static final TalonFXConfiguration EFFECTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration TOP_EFFECTOR_CONFIG = new TalonFXConfiguration();

    static {
        // Motor output settings
        EFFECTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        EFFECTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


        // Current limiting settings
        EFFECTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 40.0;
        EFFECTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

    }

    static {
        // Motor output settings
        TOP_EFFECTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        TOP_EFFECTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limiting settings
        TOP_EFFECTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 40.0; // Maximum allowed current
        TOP_EFFECTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

    }
}
