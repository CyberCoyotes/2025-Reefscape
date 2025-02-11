package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EffectorSubsystem;
import frc.robot.subsystems.endEffector.EffectorState;

public class SetEndEffectorCommand extends Command {
    private final EffectorSubsystem effector;
    private final EffectorState targetState;

    public SetEndEffectorCommand(EffectorSubsystem effector, EffectorState targetState) {
        this.effector = effector;
        this.targetState = targetState;
        addRequirements(effector);
    }

    @Override
    public void initialize() {
        System.out.println("Setting Effector to: " + targetState);
        effector.setState(targetState);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Effector command stopped. Setting to STOP.");
        effector.setState(EffectorState.STOP);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs while button is held
    }
}
