package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EffectorState;
import frc.robot.subsystems.endEffector.EffectorSubsystem;

public class EndEffectorCommands {
    private final EffectorSubsystem effector;

    public EndEffectorCommands(EffectorSubsystem effector) {
        this.effector = effector;
    }

    /**
     * Creates a command that sets the effector to a target state while running
     * and stops it when the command ends
     */
    private Command setStateCommand(EffectorState targetState) {
        return effector.runEnd(
            // While running, set state to target
            () -> effector.setState(targetState),
            // When ended, set state to STOP  
            () -> effector.setState(EffectorState.STOP)
        ).withName("Effector(" + targetState.toString() + ")");
    }

    // Common state commands
    public Command intakeCoral() {
        return setStateCommand(EffectorState.INTAKE_CORAL);
    }

    public Command scoreCoral() {
        return setStateCommand(EffectorState.SCORE_CORAL); 
    }

    public Command intakeAlgae() {
        return setStateCommand(EffectorState.INTAKE_ALGAE);
    }

    public Command scoreAlgae() {
        return setStateCommand(EffectorState.SCORE_ALGAE);
    }

    public Command hold() {
        return setStateCommand(EffectorState.HOLD);
    }

    public Command stop() {
        return setStateCommand(EffectorState.STOP);
    }
}