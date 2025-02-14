package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.WristSubsystemMotor;

public class SetWristPositionCommand extends Command {
    private final WristSubsystemMotor wrist;
    private final double targetPosition;

    public SetWristPositionCommand(WristSubsystemMotor wrist, double position) {
        this.wrist = wrist;
        this.targetPosition = position;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return wrist.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            wrist.stop();
        }
        // Add print that the wrist has reached the position
    }
}

