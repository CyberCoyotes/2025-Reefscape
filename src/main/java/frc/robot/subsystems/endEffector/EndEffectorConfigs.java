package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public class EndEffectorConfigs {
    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();

    static {
        MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 40.0;  // Maximum current before limiting
        MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    }
}
