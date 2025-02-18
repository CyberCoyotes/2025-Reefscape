package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EffectorConstants {

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
