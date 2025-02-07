package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetEffectorCommand;

public class EffectorSubsystem extends SubsystemBase {
    private final EffectorIO io;

    public EffectorSubsystem() {
        io = new EffectorIO() {
            private final TalonFX effector = new TalonFX(0);
            private final TalonFXConfiguration config = new TalonFXConfiguration();

            {
                effector.configAllSettings(config);
                effector.setControlMode(TorqueCurrentFOC.kCurrent);
            }

            @Override
            public void setState(EffectorState state) {
                effector.setTargetCurrent(state.current);
            }
        };
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.getInstance().recordOutput("Effector/State", inputs.currentState.toString());
        // Logger.getInstance().recordOutput("Effector/CurrentAmps", inputs.appliedCurrent);
        // Logger.getInstance().recordOutput("Effector/TemperatureC", inputs.motorTemperature);
    }

    public void setState(EffectorState state) {
        io.setState(state);
    }

    public EffectorState getState() {
        return inputs.currentState;
    }
}

//https://claude.ai/chat/a29e08f4-dbaa-4e2c-a7fd-948d4f2f18a5
