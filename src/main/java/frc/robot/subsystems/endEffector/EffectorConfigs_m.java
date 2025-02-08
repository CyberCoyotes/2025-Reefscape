package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// }
// @Override
// public void setState(EffectorState state) {
//     setCurrent(state.current);
//     inputs.currentState = state;
// }

// EffectorConfigs.java
public class EffectorConfigs {
    // Phoenix 6 Slot0 gains for torque control
    private static final Slot0Configs slot0Configs = new Slot0Configs()
        .withKP(0.00) // .11 Some example values - tune for your mechanism
        .withKI(0.00) // .48
        .withKD(0.00) // .01
        .withKS(0.00)  // Add 0.24V to overcome friction
        .withKV(0.00); // Apply 12V for 100 rps velocity target
        
    public static void configureMotor(TalonFX motor) {
        var configs = new TalonFXConfiguration();
        
        // Current limits
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = EffectorConstants.MAX_CURRENT_AMPS;
        
        // Apply Slot0 gains
        configs.Slot0 = slot0Configs;
 
        // Motor config 
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
 
        motor.getConfigurator().apply(configs, 0.050); // 50ms timeout
    }
 }