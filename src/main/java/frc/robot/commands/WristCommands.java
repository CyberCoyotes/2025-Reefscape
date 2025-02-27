package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.WristSubsystem;

// Command Factory methods
public class WristCommands {
    private final WristSubsystem subsystem;
    private static final double DEFAULT_TOLERANCE = 0.05; // 0.05 rotations tolerance

    public WristCommands(WristSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    public Command moveToPosition(double targetRotations) {
        return moveToPosition(targetRotations, DEFAULT_TOLERANCE);
    }

    public Command moveToPosition(double targetRotations, double toleranceRotations) {
        return this.subsystem.runEnd(
                () -> this.subsystem.setPosition(targetRotations),
                () -> this.subsystem.setPosition(this.subsystem.getPosition()))
                .until(() -> this.subsystem.atPosition(targetRotations, toleranceRotations));
    }

    public Command incrementOut() {
        return this.subsystem.runOnce(() -> this.subsystem.incrementOut());
    }

    public Command incrementIn() {
        return this.subsystem.runOnce(() -> this.subsystem.incrementIn());
    }

    public Command moveTo(WristSubsystem.WristPositions wristPose) {
        return moveToPosition(wristPose.getRotations());
    }

    public Command setStowed() {
        return moveToPosition(WristSubsystem.WristPositions.STOWED.getRotations());
    }

    public Command setL2() {
        return moveToPosition(WristSubsystem.WristPositions.L2.getRotations());
    }

    public Command setL3() {
        return moveToPosition(WristSubsystem.WristPositions.L3.getRotations());
    }

    public Command setL4() {
        return moveToPosition(WristSubsystem.WristPositions.L4.getRotations());
    }

    public Command pickAlgae() {
        return moveToPosition(WristSubsystem.WristPositions.PICK_ALGAE.getRotations());
    }

    public Command setAlgae() {
        return moveToPosition(WristSubsystem.WristPositions.SCORE_ALGAE.getRotations());
    }
    public Command resetWrist() {
        return this.subsystem.runOnce(() -> this.subsystem.resetWrist());
    }
}
