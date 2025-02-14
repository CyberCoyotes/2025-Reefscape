package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * Factory class for creating elevator commands.
 * This class centralizes all elevator command creation to make it easier to modify and maintain.
 */
public class ElevatorAdvancedCommands {
    private final ElevatorSubsystem elevator;
    private boolean isTestMode = false;

    public ElevatorAdvancedCommands(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    /**
     * Sets whether the elevator should operate in test mode (reduced speeds)
     * @param enabled True for test mode, false for performance mode
     */
    public void setTestMode(boolean enabled) {
        isTestMode = enabled;
    }

    /**
     * Creates a manual control command for the elevator
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
}