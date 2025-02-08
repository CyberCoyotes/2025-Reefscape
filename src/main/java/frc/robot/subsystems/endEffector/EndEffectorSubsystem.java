package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endEffector.EndEffectorConfigs;

public class EndEffectorSubsystem extends SubsystemBase {

    public enum EndEffectorState {
        INTAKE, SCORE, HOLD, STOP
    }

    private final TalonFX motor;
    private EndEffectorState currentState = EndEffectorState.STOP;

    public EndEffectorSubsystem(int motorID, String CANBus) {
        motor = new TalonFX(motorID, CANBus);
        motor.getConfigurator().apply(EndEffectorConfigs.EFFECTOR_CONFIG);
    }

    public void setState(EndEffectorState targetState) {
        this.currentState = targetState;
        applyState(targetState);
    }

    private void applyState(EndEffectorState state) {
        switch (state) {
            case INTAKE:
                motor.setControl(new TorqueCurrentFOC(EndEffectorConfigs.INTAKE_CURRENT.in(Units.Amps)));
                break;
            case SCORE:
                motor.setControl(new TorqueCurrentFOC(EndEffectorConfigs.SCORE_CURRENT.in(Units.Amps)));
                break;
            case HOLD:
                motor.setControl(new TorqueCurrentFOC(EndEffectorConfigs.HOLD_CURRENT.in(Units.Amps)));
                break;
            case STOP:
                motor.setControl(new TorqueCurrentFOC(EndEffectorConfigs.STOP_CURRENT.in(Units.Amps)));
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
        System.out.println("EndEffector State: " + currentState + " | Current: " + getCurrentDraw() + " A");
    }
}
