package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;


public class ElevatorCommands {
    private final ElevatorSubsystem elevator;
    private final WristSubsystem wrist;

    public ElevatorCommands(ElevatorSubsystem elevator, WristSubsystem wrist) {
        this.elevator = elevator;
        this.wrist = wrist;
    }

    /**
     * Sets elevator operation mode
     */
    // public Command setMode(ElevatorMode mode) {
        // return elevator.runOnce(() -> elevator.setMode(mode))
            // .withName("SetElevatorMode(" + mode + ")");
    // }

    /**
     * Creates a command to increment elevator up with fine control
     */
    public Command incrementUpRaw() {
        return elevator.runOnce(() -> elevator.incrementPosition(true))
            .withName("ElevatorIncrement(up)");
    }

    /**
     * Creates a command to increment elevator down with fine control
     */
    public Command incrementDownRaw() {
        return elevator.runOnce(() -> elevator.incrementPosition(false))
            .withName("ElevatorIncrement(down)");
    }

    /**
     * Creates a command to increment elevator up safely (checks wrist position)
     */
    public Command incrementUp() {
        return Commands.either(
            incrementUpRaw(),
            Commands.none(),
            () -> wrist.isSafeForElevator()  // Use wrist subsystem's method
        ).withName("SafeElevatorIncrement(up)");
    }

    /**
     * Creates a command to increment elevator down safely (checks wrist position)
     */
    public Command incrementDown() {
        return Commands.either(
            incrementDownRaw(),
            Commands.none(),
            () -> wrist.isSafeForElevator()  // Use wrist subsystem's method
        ).withName("SafeElevatorIncrement(down)");
    }

    /**
     * Creates a command that moves the elevator to a target position
     */
    private Command createMoveToPositionRaw(double targetPosition) {
        return new FunctionalCommand(
            () -> {},  // No initialization
            () -> elevator.setPosition(targetPosition),
            interrupted -> {},  // No end behavior needed
            () -> elevator.isAtPosition(targetPosition),
            elevator
        ).withName("MoveElevatorTo(" + targetPosition + ")");
    }

    /**
     * Creates a command that safely moves the elevator to a target position
     */
    private Command createMoveToPosition(double targetPosition) {
        return Commands.either(
            createMoveToPositionRaw(targetPosition),
            Commands.none(),
            () -> wrist.isSafeForElevator()  // Use wrist subsystem's method
        ).withName("SafeMoveElevatorTo(" + targetPosition + ")");
    }

    // Preset position commands - raw movement
    public Command moveToHomeRaw() {
        return createMoveToPositionRaw(ElevatorConstants.HOME_POSE).withName("MoveElevatorToHome");
    }

    public Command moveToL1Raw() {
        return createMoveToPositionRaw(ElevatorConstants.L1_POSE).withName("MoveElevatorToL1Pose");
    }

    public Command moveToL2Raw() {
        return createMoveToPositionRaw(ElevatorConstants.L2_POSE).withName("MoveElevatorToL2Pose");
    }

    public Command moveToL3Raw() {
        return createMoveToPositionRaw(ElevatorConstants.L3_POSE).withName("MoveElevatorToL3Pose");
    }

    public Command moveToL4Raw() {
        return createMoveToPositionRaw(ElevatorConstants.L4_POSE).withName("MoveElevatorToL3Pose");
    }

    // Preset position commands - safe movement
    public Command moveToHome() {
        return createMoveToPosition(ElevatorConstants.HOME_POSE).withName("SafeMoveElevatorToHome");
    }

    public Command moveToL1() {
        return createMoveToPosition(ElevatorConstants.L1_POSE).withName("SafeMoveElevatorToL1Pose");
    }

    public Command moveToL2() {
        return createMoveToPosition(ElevatorConstants.L2_POSE).withName("SafeMoveElevatorToL2Pose");
    }

    public Command moveToL3() {
        return createMoveToPosition(ElevatorConstants.L3_POSE).withName("SafeMoveElevatorToL3Pose");
    }

    public Command moveToL4() {
        return createMoveToPosition(ElevatorConstants.L4_POSE).withName("SafeMoveElevatorToL4Pose");
    }
}