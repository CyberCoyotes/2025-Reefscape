package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;

public class EndEffectorConfigs {
    public static final TalonFXConfiguration EFFECTOR_CONFIG = new TalonFXConfiguration();

    // Current Limits
    public static final Current INTAKE_CURRENT = Units.Amps.of(20);  // Fast intake
    public static final Current SCORE_CURRENT = Units.Amps.of(15);   // Controlled scoring
    public static final Current HOLD_CURRENT = Units.Amps.of(5);     // Light holding force
    public static final Current STOP_CURRENT = Units.Amps.of(0);     // Fully stopped

    static {
        // Motor output settings
        EFFECTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        EFFECTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limiting settings
        EFFECTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 40.0;  // Maximum allowed current
        EFFECTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    }
}

