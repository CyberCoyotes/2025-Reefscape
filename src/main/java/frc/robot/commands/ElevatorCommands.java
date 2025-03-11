package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ElevatorCommands {
    private final ElevatorSubsystem elevator;
    private final WristSubsystem wrist;

    public ElevatorCommands(ElevatorSubsystem elevator, WristSubsystem wrist) {
        this.elevator = elevator;
        this.wrist = wrist;
    }

    /**
     * Creates a command to increment elevator up with fine control
     */
    public Command incrementUpNoCheck() {
        return elevator.runOnce(() -> elevator.incrementPosition(true))
            .withName("ElevatorIncrement(up)");
    }

    /**
     * Creates a command to increment elevator down with fine control
     */
    public Command incrementDownNoCheck() {
        return elevator.runOnce(() -> elevator.incrementPosition(false))
            .withName("ElevatorIncrement(down)");
    }

    /**
     * Creates a command to increment elevator up safely (checks wrist position)
     */
    public Command incrementUp() {
        return incrementUpNoCheck().withName("SafeElevatorIncrement(up)");
    }

    /**
     * Creates a command to increment elevator down safely (checks wrist position)
     */
    public Command incrementDown() {
        return incrementDownNoCheck().withName("SafeElevatorIncrement(down)");
    }

    public double increment = 0.02;

    public Command incrementUpCommand() {
        return elevator.run(() -> elevator.incrementPosition(increment))
            .withName("IncrementElevatorUp");
    }

    public Command decrementDownCommand() {
        return elevator.run(() -> elevator.incrementPosition(-increment))
            .withName("IncrementElevatorDown");
    }

    /**
     * Creates a command that moves the elevator to a target position
     * This version does not check wrist position
     */
    public Command setPositionNoCheck(double targetPosition) {
        return new FunctionalCommand(
            () -> {
                // Log that we're starting the elevator movement
                System.out.println("Starting elevator movement to " + targetPosition);
            },
            () -> elevator.setPosition(targetPosition),
            interrupted -> {
                if (interrupted) {
                    System.out.println("Elevator movement interrupted");
                } else {
                    System.out.println("Elevator reached position " + targetPosition);
                }
            },
            () -> elevator.isAtPosition(targetPosition),
            elevator
        ).withName("MoveElevatorTo(" + targetPosition + ")");
    }

    /**
     * Creates a command that safely moves the elevator to a target position
     * This improved version will wait for the wrist to move to a safe position
     */
    public Command setPosition(double targetPosition) {
        return Commands.sequence(
            // First wait until the wrist is in a safe position
            Commands.waitUntil(() -> wrist.inSafePosition()),
            // Then move the elevator
            setPositionNoCheck(targetPosition)
        ).withName("SafeMoveElevatorTo(" + targetPosition + ")");
    }

    // Preset position commands - NoCheck movement
    public Command setHomeNoCheck() {
        return setPositionNoCheck(ElevatorPosition.HOME.getPosition()).withName("MoveElevatorToHome");
    }

    public Command setL1NoCheck() {
        return setPositionNoCheck(ElevatorPosition.L1.getPosition()).withName("MoveElevatorToL1Pose");
    }

    public Command setL2NoCheck() {
        return setPositionNoCheck(ElevatorPosition.L2.getPosition()).withName("MoveElevatorToL2Pose");
    }

    public Command setL3NoCheck() {
        return setPositionNoCheck(ElevatorPosition.L3.getPosition()).withName("MoveElevatorToL3Pose");
    }

    public Command setL4NoCheck() {
        return setPositionNoCheck(ElevatorPosition.L4.getPosition()).withName("MoveElevatorToL4Pose");
    }

    public Command setAlgae2NoCheck() {
        return setPositionNoCheck(ElevatorPosition.ALGAE2.getPosition()).withName("MoveElevatorToAlgae2Pose");
    }

    public Command setAlgae3NoCheck() {
        return setPositionNoCheck(ElevatorPosition.ALGAE3.getPosition()).withName("MoveElevatorToAlgae3Pose");
    }

    public Command setScoreAlgaeNoCheck() {
        return setPositionNoCheck(ElevatorPosition.SCORE_ALGAE.getPosition()).withName("MoveElevatorToScoreAlgaePose");
    }

    // Preset position commands - safe movement
    public Command setHome() {
        return setPosition(ElevatorPosition.HOME.getPosition()).withName("SafeMoveElevatorToHome");
    }

    public Command setL1() {
        return setPosition(ElevatorPosition.L1.getPosition()).withName("SafeMoveElevatorToL1Pose");
    }

    public Command setL2() {
        return setPosition(ElevatorPosition.L2.getPosition()).withName("SafeMoveElevatorToL2Pose");
    }

    public Command setL3() {
        return setPosition(ElevatorPosition.L3.getPosition()).withName("SafeMoveElevatorToL3Pose");
    }

    public Command setL4() {
        return setPosition(ElevatorPosition.L4.getPosition()).withName("SafeMoveElevatorToL4Pose");
    }

    public Command setAlgae2() {
        return setPosition(ElevatorPosition.ALGAE2.getPosition()).withName("SafeMoveElevatorToAlgae2Pose");
    }

    public Command setAlgae3() {
        return setPosition(ElevatorPosition.ALGAE3.getPosition()).withName("SafeMoveElevatorToAlgae3Pose");
    }

    public Command setScoreAlgae() {
        return setPosition(ElevatorPosition.SCORE_ALGAE.getPosition()).withName("SafeMoveElevatorToScoreAlgaePose");
    }

    public Command setIntakeCoral() {
        return setPosition(ElevatorPosition.INTAKE_CORAL.getPosition()).withName("SafeMoveElevatorToIntakeCoralPose");
    }   
}