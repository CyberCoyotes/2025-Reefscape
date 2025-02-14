package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ElevatorAvdancedCommands {
    private final ElevatorSubsystem elevator;
    private final WristSubsystem wrist;


    public ElevatorAvdancedCommands(ElevatorSubsystem elevator, WristSubsystem wrist) {
        this.elevator = elevator;
        this.wrist = wrist;

    }

     /**
     * Checks if the wrist is in a safe position for elevator movement
     * @return true if safe, false if unsafe
     */
    private boolean isWristSafe() {
        double wristPos = wrist.getPosition();
        // Check if wrist is within safe zone around ELEVATOR_SAFE position
        boolean isSafe = Math.abs(wristPos - WristConstants.Positions.ELEVATOR_SAFE) < WristConstants.WRIST_POSE_TOLERANCE;
        
        if (!isSafe) {
            // Send warning to driver station
            DriverStation.reportWarning("UNSAFE ELEVATOR MOVE: Wrist must be in safe position", false);
        }
        return isSafe;
    }

    /**
     * Creates a command that moves the elevator to a target position
     * 
     * @param targetPosition The position to move to in rotations
     * @return Command to run
     */
    private Command createMoveToPositionRaw(double targetPosition) {
        // Does not check wrist safety
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
    public Command moveToBaseRaw() {
        return createMoveToPositionRaw(ElevatorConstants.BASE_POSE)
                .withName("MoveElevatorToBase");
    }
/*
 * 
 */
    /**
     * Creates a command to move the elevator to middle position
     */
    public Command moveToL1Raw() {
        /*
        * If wrist is in the way, move wrist to safe position
        */
        return createMoveToPositionRaw(ElevatorConstants.L1_POSE)
                .withName("MoveElevatorToL1");
    }

    /**
     * Creates a command to move the elevator to high position
     */
    public Command moveToL2Raw() {
        /*
        * If wrist is in the way, move wrist to safe position
        */
        return createMoveToPositionRaw(ElevatorConstants.L2_POSE)
                .withName("MoveElevatorToL2");
    }

    /**
     * Creates a command to move the elevator to high position
     */
    public Command moveToL3Raw() {
        /* 
        * If wrist is in the way, move wrist to safe position
        */
        return createMoveToPositionRaw(ElevatorConstants.L3_POSE)
                .withName("MoveElevatorToL3");
    }

    /**
     * Creates a command to move the elevator to high position
     */
    public Command moveToL4Raw() {
        /*
        * If wrist is in the way, move wrist to safe position
        */
        return createMoveToPositionRaw(ElevatorConstants.L4_POSE)
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

    /***********************************************************************
     * Creates a command that safely moves the elevator to a target position
     *************************************************************************/
    private Command createMoveToPosition(double targetPosition) {
        return Commands.sequence(
            // First check if wrist is safe
            Commands.either(
                // If wrist is safe, proceed with elevator movement
                createMoveToPositionRaw(targetPosition),
                
                // If wrist is unsafe, do nothing but log warning
                Commands.none(),
                
                // The condition to check
                this::isWristSafe
            )
        ).withName("SafeMoveElevatorTo" + targetPosition);
    }

    /**
     * Creates a command to safely move the elevator to ground position
     */
    public Command moveToBase() {
        return createMoveToPosition(ElevatorConstants.BASE_POSE)
                .withName("SafeMoveElevatorToBase");
    }

    /**
     * Creates a command to safely move the elevator to L1 position
     */
    public Command moveToL1() {
        return createMoveToPosition(ElevatorConstants.L1_POSE)
                .withName("SafeMoveElevatorToL1");
    }

    /**
     * Creates a command to safely move the elevator to L2 position
     */
    public Command moveToL2() {
        return createMoveToPosition(ElevatorConstants.L2_POSE)
                .withName("SafeMoveElevatorToL2");
    }

    /**
     * Creates a command to safely move the elevator to L3 position
     */
    public Command moveToL3() {
        return createMoveToPosition(ElevatorConstants.L3_POSE)
                .withName("SafeMoveElevatorToL3");
    }

    /**
     * Creates a command to safely move the elevator to L4 position
     */
    public Command moveToL4() {
        return createMoveToPosition(ElevatorConstants.L4_POSE)
                .withName("SafeMoveElevatorToL4");
    }

        /**
     * Creates a command that moves wrist to safe position then moves elevator
     */
    public Command moveWristMoveElevator(double targetPosition) {
        return Commands.sequence(
            // First move wrist to safe position
            WristCommands.setElevatorSafe(wrist),
            
            // Wait until wrist is in position
            Commands.waitUntil(() -> wrist.atTargetPosition(WristConstants.WRIST_POSE_TOLERANCE)),
            
            // Then move elevator
            createMoveToPositionRaw(targetPosition)
        ).withName("MoveElevatorWithWristSafety" + targetPosition);
    }
}