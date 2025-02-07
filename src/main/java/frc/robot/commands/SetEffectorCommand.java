package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EffectorState;
import frc.robot.subsystems.endEffector.EffectorSubsystem;

public class SetEffectorCommand extends Command {
    private final EffectorSubsystem effector;
    private final EffectorState targetState;

    public SetEffectorCommand(EffectorSubsystem effector, EffectorState state) {
        this.effector = effector;
        this.targetState = state;
        addRequirements(effector);
    }

    @Override
    public void execute() {
        effector.setState(targetState);
    }

    @Override
    public void end(boolean interrupted) {
        effector.setState(EffectorState.STOP);
    }
}
