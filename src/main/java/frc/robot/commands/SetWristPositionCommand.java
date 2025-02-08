package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.WristSubsystem;

public class SetWristPositionCommand extends Command {
    private final WristSubsystem wrist;
    private final double targetPosition;
    private static final double POSITION_TIMEOUT = 2.0; // Seconds
    private double startTime;

    public SetWristPositionCommand(WristSubsystem wrist, double position) {
        this.wrist = wrist;
        this.targetPosition = position;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        wrist.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // End if at position or timed out
        if (Timer.getFPGATimestamp() - startTime > POSITION_TIMEOUT) {
            wrist.stop();
            return true;
        }
        return wrist.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            wrist.stop();
        }
    }
}

