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
     * 
     * @param targetPosition The position to move to in rotations
     * @return Command to run
     */
    private Command createMoveToPositionCommand(double targetPosition) {
        return new FunctionalCommand(
                // Initialize - Nothing to initialize
                () -> {
                },
                // Execute - Set the target position
                () -> elevator.setPosition(targetPosition),
                // End - Stop the elevator
                interrupted -> {}, //elevator.setManualOutput(0),
                // IsFinished - Check if we've reached the target
                () -> elevator.isAtPosition(targetPosition),
                // Requires - This subsystem
                elevator)
                .withName("MoveElevatorTo" + targetPosition);
    }

    /**
     * Creates a command to move the elevator to ground position
     */
    public Command moveToBase() {
        return createMoveToPositionCommand(ElevatorConstants.BASE_POSE)
                .withName("MoveElevatorToBase");
    }
/*
 * 
 */
    /**
     * Creates a command to move the elevator to middle position
     */
    public Command moveToL1() {
        /* TODO Check Wrist position-state
        * If wrist is in the way, move wrist to safe position
        */
        return createMoveToPositionCommand(ElevatorConstants.L1_POSE)
                .withName("MoveElevatorToL1");
    }

    /**
     * Creates a command to move the elevator to high position
     */
    public Command moveToL2() {
        /* TODO Check Wrist position-state
        * If wrist is in the way, move wrist to safe position
        */
        return createMoveToPositionCommand(ElevatorConstants.L2_POSE)
                .withName("MoveElevatorToL2");
    }

    /**
     * Creates a command to move the elevator to high position
     */
    public Command moveToL3() {
        /* TODO Check Wrist position-state
        * If wrist is in the way, move wrist to safe position
        */
        return createMoveToPositionCommand(ElevatorConstants.L3_POSE)
                .withName("MoveElevatorToL3");
    }

    /**
     * Creates a command to move the elevator to high position
     */
    public Command moveToL4() {
        /* TODO Check Wrist position-state
        * If wrist is in the way, move wrist to safe position
        */
        return createMoveToPositionCommand(ElevatorConstants.L4_POSE)
                .withName("MoveElevatorToL4");
    }

    /**
     * Creates a command for manual elevator control
     * 
     * @param percentSupplier A supplier that returns the percent output (-1 to 1)
     */
    public Command manualControl(java.util.function.DoubleSupplier percentSupplier) {
        return new FunctionalCommand(
                // Initialize - Nothing to initialize
                () -> {
                },
                // Execute - Set the manual output
                () -> elevator.setManualOutput(percentSupplier.getAsDouble()),
                // End - Stop the elevator
                interrupted -> elevator.setManualOutput(0),
                // IsFinished - Never finish (run until interrupted)
                () -> false,
                // Requires - This subsystem
                elevator)
                .withName("ManualElevatorControl");
    }
}