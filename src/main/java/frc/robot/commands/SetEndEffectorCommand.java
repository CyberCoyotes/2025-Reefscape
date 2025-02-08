package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorState;

public class SetEndEffectorCommand extends Command {
    private final EndEffectorSubsystem endEffector;
    private final EndEffectorState targetState;

    public SetEndEffectorCommand(EndEffectorS endEffector, EndEffector.EndEffectorState state) {
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
