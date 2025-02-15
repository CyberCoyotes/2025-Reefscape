package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMode;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

import java.util.function.DoubleSupplier;

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
    public Command setMode(ElevatorMode mode) {
        return elevator.runOnce(() -> elevator.setMode(mode))
            .withName("SetElevatorMode(" + mode + ")");
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

    /**
     * Creates a command to increment elevator up safely (checks wrist position)
     */
    public Command incrementUpSafely() {
        return Commands.either(
            incrementUp(),
            Commands.none(),
            this::isWristSafe
        ).withName("SafeElevatorIncrement(up)");
    }

    /**
     * Creates a command to increment elevator down safely (checks wrist position)
     */
    public Command incrementDownSafely() {
        return Commands.either(
            incrementDown(),
            Commands.none(),
            this::isWristSafe
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
            this::isWristSafe
        ).withName("SafeMoveElevatorTo(" + targetPosition + ")");
    }

    // Preset position commands - raw movement
    public Command moveToHomeRaw() {
        return createMoveToPositionRaw(0.0).withName("MoveElevatorToHome");
    }

    public Command moveToL1Raw() {
        return createMoveToPositionRaw(0.2).withName("MoveElevatorToL1Pose");
    }

    public Command moveToL2Raw() {
        return createMoveToPositionRaw(0.5).withName("MoveElevatorToL2Pose");
    }

    public Command moveToL3Raw() {
        return createMoveToPositionRaw(0.9).withName("MoveElevatorToL3Pose");
    }

    // Preset position commands - safe movement
    public Command moveToHome() {
        return createMoveToPosition(0.0).withName("SafeMoveElevatorToHome");
    }

    public Command moveToL1() {
        return createMoveToPosition(0.2).withName("SafeMoveElevatorToL1Pose");
    }

    public Command moveToL2() {
        return createMoveToPosition(0.5).withName("SafeMoveElevatorToL2Pose");
    }

    public Command moveToL3() {
        return createMoveToPosition(0.9).withName("SafeMoveElevatorToL3Pose");
    }

    /**
     * Creates a command for manual elevator control
     */
    public Command manualControl(DoubleSupplier percentSupplier) {
        return new FunctionalCommand(
            () -> {},
            () -> elevator.setManualOutput(percentSupplier.getAsDouble()),
            interrupted -> elevator.setManualOutput(0),
            () -> false,
            elevator
        ).withName("ManualElevatorControl");
    }

    /**
     * Creates a command sequence that ensures wrist safety before moving elevator
     */
    public Command moveWithWristSafety(double targetPosition) {
        return Commands.sequence(
            // First move wrist to safe position
            WristCommands.setSafePose(wrist),
            // Wait for wrist to reach position
            Commands.waitUntil(() -> wrist.atTargetPosition(0.02)),
            // Then move elevator
            createMoveToPositionRaw(targetPosition)
        ).withName("MoveElevatorWithWristSafety(" + targetPosition + ")");
    }

    /**
     * Checks if the wrist is in a safe position for elevator movement
     */
    private boolean isWristSafe() {
        double wristPos = wrist.getPosition();
        boolean isSafe = wristPos >= WristConstants.Positions.SAFE; // Example safe threshold
        
        if (!isSafe) {
            DriverStation.reportWarning("Wrist position unsafe for elevator movement", false);
        }
        
        return isSafe;
    }

    public void periodic() {
        Logger.recordOutput("Elevator/Commands/WristSafetyCheck", isWristSafe());
    }
}