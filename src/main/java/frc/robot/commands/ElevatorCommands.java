package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.elevator.ElevatorConstants;
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
        return incrementUpRaw().withName("SafeElevatorIncrement(up)");
    }

    /**
     * Creates a command to increment elevator down safely (checks wrist position)
     */
    public Command incrementDown() {
        return incrementDownRaw().withName("SafeElevatorIncrement(down)");
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
     */
    private Command moveToPositionRaw(double targetPosition) {
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
    // moveToPosition
    // 
    private Command moveToPosition(double targetPosition) {
        return Commands.either(
            moveToPositionRaw(targetPosition),
            Commands.none(),
            () -> wrist.inSafePosition()  // Use wrist subsystem's method
        ).withName("SafeMoveElevatorTo(" + targetPosition + ")");
    }

    // Preset position commands - raw movement
    public Command moveToHomeRaw() {
        return moveToPositionRaw(ElevatorPosition.HOME.getPosition()).withName("MoveElevatorToHome");
    }

    public Command moveToL1Raw() {
        return moveToPositionRaw(ElevatorPosition.L1.getPosition()).withName("MoveElevatorToL1Pose");
    }

    public Command moveToL2Raw() {
        return moveToPositionRaw(ElevatorPosition.L2.getPosition()).withName("MoveElevatorToL2Pose");
    }

    public Command moveToL3Raw() {
        return moveToPositionRaw(ElevatorPosition.L3.getPosition()).withName("MoveElevatorToL3Pose");
    }

    public Command moveToL4Raw() {
        return moveToPositionRaw(ElevatorPosition.L4.getPosition()).withName("MoveElevatorToL3Pose");
    }

    // Preset position commands - safe movement
    public Command moveToHome() {
        return moveToPosition(ElevatorPosition.HOME.getPosition()).withName("SafeMoveElevatorToHome");
    }

    public Command moveToL1() {
        return moveToPosition(ElevatorPosition.L1.getPosition()).withName("SafeMoveElevatorToL1Pose");
    }

    public Command moveToL2() {
        return moveToPosition(ElevatorPosition.L2.getPosition()).withName("SafeMoveElevatorToL2Pose");
    }

    public Command moveToL3() {
        return moveToPosition(ElevatorPosition.L3.getPosition()).withName("SafeMoveElevatorToL3Pose");
    }

    public Command moveToL4() {
        return moveToPosition(ElevatorPosition.L4.getPosition()).withName("SafeMoveElevatorToL4Pose");
    }

    public Command moveToPickAlgae2Raw() {
        return moveToPositionRaw(ElevatorPosition.Algae2.getPosition()).withName("SafeMoveElevatorToL5Pose");
    }

    public Command moveToPickAlgae3Raw() {
        return moveToPositionRaw(ElevatorPosition.Algae3.getPosition()).withName("SafeMoveElevatorToL5Pose");
    }
}