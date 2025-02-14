package frc.robot.commands;

import java.time.Period;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

/**
 * Factory class for creating elevator commands.
 * This class centralizes all elevator command creation to make it easier to modify and maintain.
 */
public class ElevatorCommands {
    private final ElevatorSubsystem elevator;
    private final WristSubsystem wrist;
    private boolean isTestMode = false;

    public ElevatorCommands(ElevatorSubsystem elevator, WristSubsystem wrist) {
        this.elevator = elevator;
        this.wrist = wrist;
    }

    /**
     * Sets whether the elevator should operate in test mode (reduced speeds)
     * @param enabled True for test mode, false for performance mode
     */
    public void setTestMode(boolean enabled) {
        isTestMode = enabled;
        elevator.setSafetyMode(enabled); // Update subsystem safety mode to match
    }

    /**
     * Creates a manual control command with test/performance speed limits
     * @param speedSupplier Supplier for the speed control input (-1 to 1)
     * @return The manual control command
     */
    public Command createManualCommand(DoubleSupplier speedSupplier) {
        double maxSpeed = isTestMode ? 
            ElevatorConstants.MANUAL_MAX_SPEED_TESTING : 
            ElevatorConstants.MANUAL_MAX_SPEED_PERFORMANCE;
            
        return new ManualElevatorCommand(elevator, speedSupplier, maxSpeed);
    }

    /**
     * Creates a manual control command with a custom max speed
     * @param speedSupplier Supplier for the speed control input (-1 to 1)
     * @param maxSpeed Maximum speed limit (0 to 1)
     * @return The manual control command
     */
    public Command createManualCommand(DoubleSupplier speedSupplier, double maxSpeed) {
        return new ManualElevatorCommand(elevator, speedSupplier, maxSpeed);
    }

     /**
     * Checks if the wrist is in a safe position for elevator movement
     * @return true if safe, false if unsafe
     */
    private boolean isWristSafe() {
        double wristPos = wrist.getPosition();

        // Wrist is safe if current wrist position is equal or great thean ELEVATOR_SAFE
        boolean isSafe = Math.abs(wristPos) > WristConstants.Positions.ELEVATOR_SAFE;
        
        
        if (!isSafe) {
            DriverStation.reportWarning("WRIST IS UNSAFE", false);
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
        return new FunctionalCommand(
                () -> {},
                () -> elevator.setPosition(targetPosition),
                interrupted -> {},
                () -> elevator.isAtPosition(targetPosition),
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

    public Command moveToL1Raw() {
        return createMoveToPositionRaw(ElevatorConstants.L1_POSE)
                .withName("MoveElevatorToL1");
    }

    public Command moveToL2Raw() {
        return createMoveToPositionRaw(ElevatorConstants.L2_POSE)
                .withName("MoveElevatorToL2");
    }

    public Command moveToL3Raw() {
        return createMoveToPositionRaw(ElevatorConstants.L3_POSE)
                .withName("MoveElevatorToL3");
    }

    public Command moveToL4Raw() {
        return createMoveToPositionRaw(ElevatorConstants.L4_POSE)
                .withName("MoveElevatorToL4");
    }

    /**
     * Creates a command for manual elevator control
     * 
     * @param percentSupplier A supplier that returns the percent output (-1 to 1)
     */
    public Command manualControl(DoubleSupplier percentSupplier) {
        return new FunctionalCommand(
                () -> {},
                () -> elevator.setManualOutput(percentSupplier.getAsDouble()),
                interrupted -> elevator.setManualOutput(0),
                () -> false,
                elevator)
                .withName("ManualElevatorControl");
    }

    /***********************************************************************
     * Creates a command that safely moves the elevator to a target position
     *************************************************************************/
    private Command createMoveToPosition(double targetPosition) {
        return Commands.sequence(
            Commands.either(
                createMoveToPositionRaw(targetPosition),
                Commands.none(),
                this::isWristSafe
            )
        ).withName("SafeMoveElevatorTo" + targetPosition);
    }

    public Command moveToBase() {
        return createMoveToPosition(ElevatorConstants.BASE_POSE)
                .withName("SafeMoveElevatorToBase");
    }

    public Command moveToL1() {
        return createMoveToPosition(ElevatorConstants.L1_POSE)
                .withName("SafeMoveElevatorToL1");
    }

    public Command moveToL2() {
        return createMoveToPosition(ElevatorConstants.L2_POSE)
                .withName("SafeMoveElevatorToL2");
    }

    public Command moveToL3() {
        return createMoveToPosition(ElevatorConstants.L3_POSE)
                .withName("SafeMoveElevatorToL3");
    }

    public Command moveToL4() {
        return createMoveToPosition(ElevatorConstants.L4_POSE)
                .withName("SafeMoveElevatorToL4");
    }

    /**
     * Creates a command that moves wrist to safe position then moves elevator
     */
    public Command moveWristMoveElevator(double targetPosition) {
        return Commands.sequence(
            WristCommands.setElevatorSafe(wrist),
            Commands.waitUntil(() -> wrist.atTargetPosition(WristConstants.WRIST_POSE_TOLERANCE)),
            createMoveToPositionRaw(targetPosition)
        ).withName("MoveElevatorWithWristSafety" + targetPosition);
    }

    public void periodic() {
        // Log the wrist safety check
        Logger.recordOutput("Elevator/Commands/IsWristSafe", isWristSafe());
    }

}