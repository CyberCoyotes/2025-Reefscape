package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EffectorConstants {

    // CAN settings
    public static final int EFFECTOR_MOTOR_ID = 21; // Adjust ID as needed
    public static final CANBus kCANBus = new CANBus("rio");


    public static final TalonFXConfiguration EFFECTOR_CONFIG = new TalonFXConfiguration();

    static {
        // Motor output settings
        EFFECTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        EFFECTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limiting settings
        EFFECTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 40.0; // Maximum allowed current
        EFFECTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    }  

}
