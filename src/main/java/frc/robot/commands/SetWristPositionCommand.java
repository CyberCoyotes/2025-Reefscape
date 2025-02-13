package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.WristSubsystemMotor;

public class SetWristPositionCommand extends Command {
    private final WristSubsystemMotor wristMotor;
    private final double targetPosition;

    public SetWristPositionCommand(WristSubsystemMotor wristMotor, double position) {
        this.wristMotor = wristMotor;
        this.targetPosition = position;
        addRequirements(wristMotor);
    }

    @Override
    public void initialize() {
        wristMotor.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return wristMotor.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            wristMotor.stop();
        }
        // Add print that the wrist has reached the position
    }
}

