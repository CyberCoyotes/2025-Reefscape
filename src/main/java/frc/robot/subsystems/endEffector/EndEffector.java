package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EndEffector.EndEffectorConfigs;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

public class EndEffector extends SubsystemBase {

    public enum EndEffectorState {
        INTAKE, SCORE, HOLD, STOP
    }

    private final TalonFX motor;
    private EndEffectorState currentState = EndEffectorState.STOP;

    public EndEffector(int motorID, String CANBus) {
        motor = new TalonFX(motorID, CANBus);
        motor.getConfigurator().apply(EndEffectorConfigs.MOTOR_CONFIG);
    }

    public void setState(EndEffectorState state) {
        this.currentState = state;
        applyState(state);
    }

    private void applyState(EndEffectorState state) {
        switch (state) {
            case INTAKE:
                motor.setControl(new TorqueCurrentFOC(EndEffectorConstants.INTAKE_CURRENT.in(Units.Amps)));
                break;
            case SCORE:
                motor.setControl(new TorqueCurrentFOC(EndEffectorConstants.SCORE_CURRENT.in(Units.Amps)));
                break;
            case HOLD:
                motor.setControl(new TorqueCurrentFOC(EndEffectorConstants.HOLD_CURRENT.in(Units.Amps)));
                break;
            case STOP:
                motor.setControl(new TorqueCurrentFOC(EndEffectorConstants.STOP_CURRENT.in(Units.Amps)));
                break;
        }
    }

    public EndEffectorState getState() {
        return currentState;
    }

    public Current getCurrentDraw() {
        return motor.getStatorCurrent().getValue();
    }

    @Override
    public void periodic() {
        // Monitor motor current for debugging
        System.out.println("EndEffector State: " + currentState + " | Current: " + getCurrentDraw() + " A");
    }
}
