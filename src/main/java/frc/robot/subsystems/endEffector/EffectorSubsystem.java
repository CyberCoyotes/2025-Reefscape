package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.DutyCycleOut;

public class EffectorSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private EffectorState currentState = EffectorState.STOP;

    public EffectorSubsystem() {
        motor = new TalonFX(Constants.EFFECTOR_MOTOR_ID, Constants.kCANBus);
        configureMotor();
    }

    private void configureMotor() {
        

        motor.getConfigurator().apply(EffectorConstants.EFFECTOR_CONFIG);
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