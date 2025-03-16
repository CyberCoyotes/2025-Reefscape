package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristConstants;

// Command Factory methods
public class WristCommands {
    private final WristSubsystem subsystem;

    public WristCommands(WristSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    public Command moveToPosition(double targetRotations) {
        return moveToPosition(targetRotations, WristConstants.TOLERANCE);
    }

    public Command moveToPosition(double targetRotations, double toleranceRotations) {
        return this.subsystem.runEnd(
                () -> this.subsystem.setPosition(targetRotations),
                () -> this.subsystem.setPosition(this.subsystem.getPosition()))
                .until(() -> this.subsystem.atPosition(targetRotations, toleranceRotations));
    }

    public Command incrementOut() {
        return Commands.run(
            () -> subsystem.incrementOut())
        .finallyDo((boolean interrupted) -> subsystem.setPosition(subsystem.getPosition()))
        .withName("IncrementWristOut");
    }

    public Command incrementIn() {
        return Commands.run(
            () -> subsystem.incrementIn())
        .finallyDo((boolean interrupted) -> subsystem.setPosition(subsystem.getPosition()))
        .withName("IncrementWristIn");
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

    public Command setIntakeCoral() {
        return moveToPosition(WristSubsystem.WristPositions.INTAKE_CORAL.getRotations());
    }

    public Command pickAlgae() {
        return moveToPosition(WristSubsystem.WristPositions.PICK_ALGAE.getRotations());
    }

    public Command scoreAlgae() {
        return moveToPosition(WristSubsystem.WristPositions.SCORE_ALGAE.getRotations());
    }
    
    public Command setTravelPose() {
        return moveToPosition(WristSubsystem.WristPositions.TRAVEL.getRotations());
    }

    public Command resetWrist() {
        return this.subsystem.runOnce(() -> this.subsystem.resetWrist());
    }
}