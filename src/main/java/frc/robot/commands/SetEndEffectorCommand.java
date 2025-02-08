package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;

public class SetEndEffectorCommand extends Command {
    private final EndEffector endEffector;
    private final EndEffector.EndEffectorState targetState;

    public SetEndEffectorCommand(EndEffector endEffector, EndEffector.EndEffectorState state) {
        this.endEffector = endEffector;
        this.targetState = state;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.setState(targetState);
    }

    @Override
    public boolean isFinished() {
        return true; // One-time state change
    }
}
