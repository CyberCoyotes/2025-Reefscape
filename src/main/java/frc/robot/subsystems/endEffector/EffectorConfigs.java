package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;

public class EffectorConfigs {

    // CAN settings
    public static final int EFFECTOR_MOTOR_ID = 21; // Adjust ID as needed
    public static final String CANBUS_NAME = "rio"; // or "canivore" if using CANivore


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
