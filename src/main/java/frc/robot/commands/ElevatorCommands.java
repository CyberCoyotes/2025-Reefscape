package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.wrist.WristSubsystem;

@SuppressWarnings("unused") // Suppress warnings for unused imports and methods

/**
 * Command factory class for the Elevator subsystem.
 * This class contains methods that create and return commands for the elevator.
 */

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
    public Command incrementUp() {
        return elevator.runOnce(() -> elevator.incrementPosition(true))
            .withName("ElevatorIncrement(up)");
    }

    /**
     * Creates a command to increment elevator down with fine control
     */
    public Command incrementDown() {
        return elevator.runOnce(() -> elevator.incrementPosition(false))
            .withName("ElevatorIncrement(down)");
    }


    public double increment = 0.02;

    public Command incrementUpCommand() {
        return elevator.run(() -> elevator.incrementPosition(increment))
            .withName("IncrementElevatorUp");
    }

    public Command incrementDownCommand() {
        return elevator.run(() -> elevator.incrementPosition(-increment))
            .withName("IncrementElevatorDown");
    }

    /**
     * Creates a command that moves the elevator to a target position
     * This version does not check wrist position
     */
    public Command setPosition(double targetPosition) {
        return new FunctionalCommand(
            () -> {
                // Log that we're starting the elevator movement
                System.out.println("Starting elevator movement to " + targetPosition);
            },
            // Move the elevator to the target position using the elevator subsystem
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



    /* DEPRECATE 
    public Command setPosition(double targetPosition) {
        return Commands.sequence(
            // First wait until the wrist is in a safe position
            Commands.waitUntil(() -> wrist.inSafePosition()),
            // Then move the elevator
            setPositionNoCheck(targetPosition)
        ).withName("SafeMoveElevatorTo(" + targetPosition + ")");
    }

    */

    // Preset position commands - NoCheck movement
    public Command setHome() {
        return setPosition(ElevatorPosition.HOME.getPosition()).withName("MoveElevatorToHome");
    }

    public Command setL1() {
        return setPosition(ElevatorPosition.L1.getPosition()).withName("MoveElevatorToL1Pose");
    }

    public Command setL2() {
        return setPosition(ElevatorPosition.L2.getPosition()).withName("MoveElevatorToL2Pose");
    }

    public Command setL3() {
        return setPosition(ElevatorPosition.L3.getPosition()).withName("MoveElevatorToL3Pose");
    }

    public Command setL4() {
        return setPosition(ElevatorPosition.L4.getPosition()).withName("MoveElevatorToL4Pose");
    }

    public Command setAlgae2() {
        return setPosition(ElevatorPosition.ALGAE2.getPosition()).withName("MoveElevatorToAlgae2Pose");
    }

    public Command setAlgae3() {
        return setPosition(ElevatorPosition.ALGAE3.getPosition()).withName("MoveElevatorToAlgae3Pose");
    }

    public Command setScoreAlgae() {
        return setPosition(ElevatorPosition.SCORE_ALGAE.getPosition()).withName("MoveElevatorToScoreAlgaePose");
    }

    public Command setIntakeCoral() {
        // Check as it used the original setPosition method
        return setPosition(ElevatorPosition.INTAKE_CORAL.getPosition()).withName("SafeMoveElevatorToIntakeCoralPose");
    }

    /* DEPRECATE
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
    */
       
}