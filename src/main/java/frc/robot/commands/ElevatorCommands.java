package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorCommands {
    private final ElevatorSubsystem elevator;

    public ElevatorCommands(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    /**
     * Creates a command that moves the elevator to a target position
     * @param targetPosition The position to move to in rotations
     * @return Command to run
     */
    private Command createMoveToPositionCommand(double targetPosition) {
        return new FunctionalCommand(
            // Initialize - Nothing to initialize
            () -> {},
            // Execute - Set the target position
            () -> elevator.setPosition(targetPosition),
            // End - Stop the elevator
            interrupted -> elevator.setManualOutput(0),
            // IsFinished - Check if we've reached the target
            () -> elevator.isAtPosition(targetPosition),
            // Requires - This subsystem
            elevator
        )
        .withName("MoveElevatorTo" + targetPosition);
    }

    /**
     * Creates a command to move the elevator to ground position
     */
    public Command moveToGround() {
        return createMoveToPositionCommand(ElevatorConstants.GROUND_POSITION)
            .withName("MoveElevatorToGround");
    }

    /**
     * Creates a command to move the elevator to middle position
     */
    public Command moveToMiddle() {
        return createMoveToPositionCommand(ElevatorConstants.MIDDLE_POSITION)
            .withName("MoveElevatorToMiddle");
    }

    /**
     * Creates a command to move the elevator to high position
     */
    public Command moveToHigh() {
        return createMoveToPositionCommand(ElevatorConstants.HIGH_POSITION)
            .withName("MoveElevatorToHigh");
    }

    /**
     * Creates a command for manual elevator control
     * @param percentSupplier A supplier that returns the percent output (-1 to 1)
     */
    public Command manualControl(java.util.function.DoubleSupplier percentSupplier) {
        return new FunctionalCommand(
            // Initialize - Nothing to initialize
            () -> {},
            // Execute - Set the manual output
            () -> elevator.setManualOutput(percentSupplier.getAsDouble()),
            // End - Stop the elevator
            interrupted -> elevator.setManualOutput(0),
            // IsFinished - Never finish (run until interrupted)
            () -> false,
            // Requires - This subsystem
            elevator
        )
        .withName("ManualElevatorControl");
    }
}