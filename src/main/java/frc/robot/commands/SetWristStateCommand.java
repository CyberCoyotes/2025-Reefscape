package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.WristStates;
import frc.robot.subsystems.wrist.WristSubsystem;

public class SetWristStateCommand extends Command {
    private final WristSubsystem wrist;
    private final WristStates targetState;

    public SetWristStateCommand(WristSubsystem wrist, WristStates state) {
        this.wrist = wrist;
        this.targetState = state;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setState(targetState);
    }

    @Override
    public boolean isFinished() {
        return wrist.atPosition();
    }
}