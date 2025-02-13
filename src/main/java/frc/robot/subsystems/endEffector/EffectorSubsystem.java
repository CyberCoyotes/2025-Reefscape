package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.DutyCycleOut;

public class EffectorSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private EffectorState currentState = EffectorState.STOP;

    public EffectorSubsystem() {
        motor = new TalonFX(EffectorConfigs.EFFECTOR_MOTOR_ID, EffectorConfigs.CANBUS_NAME);
        configureMotor();
    }

    private void configureMotor() {
        motor.getConfigurator().apply(EffectorConfigs.EFFECTOR_CONFIG);
    }

    public void setState(EffectorState state) {
        this.currentState = state;
        motor.setControl(new DutyCycleOut(state.dutyCycle));
    }

    public EffectorState getState() {
        return currentState;
    }

    @Override
    public void periodic() {
        // Debugging purposes
        // System.out.println("Effector State: " + currentState + " | Power: " + currentState.dutyCycle);
    }
}
