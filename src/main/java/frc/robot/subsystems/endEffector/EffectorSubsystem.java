package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EffectorSubsystem extends SubsystemBase {
    // Hardware
    private final TalonFX motor;

    // Control requests (reuse to avoid object allocation)
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    
    // State tracking  
    private EffectorState currentState = EffectorState.STOP;

    public EffectorSubsystem() {
        motor = new TalonFX(EffectorConstants.EFFECTOR_MOTOR_ID, EffectorConstants.CANBUS_NAME);
        configureMotor();
    }

    private void configureMotor() {
        motor.getConfigurator().apply(EffectorConstants.EFFECTOR_CONFIG);
    }

    /**
     * Sets the effector state and applies corresponding duty cycle
     */
    public void setState(EffectorState state) {
        this.currentState = state;
        motor.setControl(dutyCycleRequest.withOutput(state.dutyCycle));
    }

    /**
     * Gets the current effector state
     */
    public EffectorState getState() {
        return currentState;
    }

    @Override
    public void periodic() {
        // Add dashboard data or logging if needed
    }
}