package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.FrontTOFSubsystem;

/**
 * Utility class to check if the robot is in the correct range for loading coral
 * and conditionally execute commands based on that information.
 */
public class LoadingRangeChecker {
    
    // Constants for loading range thresholds in millimeters
    public static final int LOADING_RANGE_MIN = 700; // Needs to be tested empirically more
    public static final int LOADING_RANGE_MAX = 780; // Needs to be tested empirically more
    
    private final FrontTOFSubsystem tofSensor;
    
    public LoadingRangeChecker(FrontTOFSubsystem tofSensor) {
        this.tofSensor = tofSensor;
    }
    
    /**
     * Checks if the robot is within the loading range
     * @return true if within range, false otherwise
     */
    public boolean isInLoadingRange() {
        double currentDistance = tofSensor.getFrontDistance();
        // Ability to check a range, but using a "too close" check for now
        return currentDistance >= LOADING_RANGE_MIN /*&& currentDistance <= LOADING_RANGE_MAX*/;
    }
    
    /**
     * Creates a command that executes the loadingCommand only if the robot is within loading range.
     * If not in range, it can execute an optional fallback command or do nothing.
     * 
     * @param loadingCommand The command to execute when in loading range
     * @param fallbackCommand The command to execute when not in loading range (optional)
     * @return A conditional command that checks loading range first
     */
    public Command whenInLoadingRange(Command loadingCommand, Command fallbackCommand) {
        return new ConditionalCommand(
            loadingCommand,
            fallbackCommand != null ? fallbackCommand : Commands.none(),
            this::isInLoadingRange
        );
    }
    
    /**
     * Simplified version that only executes when in range, otherwise does nothing
     */
    public Command whenInLoadingRange(Command loadingCommand) {
        return whenInLoadingRange(loadingCommand, null);
    }
    
    /**
     * Creates a command that provides feedback about the current loading position
     * and waits until the robot is in the loading range before proceeding.
     * 
     * @param loadingCommand The command to execute once in loading range
     * @return A command sequence that waits for proper positioning then executes the loading command
     */
    public Command waitUntilInLoadingRangeThen(Command loadingCommand) {
        return Commands.sequence(
            // First, print the current distance
            Commands.runOnce(() -> System.out.println("Current distance: " + tofSensor.getFrontDistance() + "mm")),
            
            // Wait until within loading range
            Commands.waitUntil(this::isInLoadingRange),
            
            // Confirm we're in range
            Commands.runOnce(() -> System.out.println("In loading range: " + tofSensor.getFrontDistance() + "mm")),
            
            // Execute the provided command
            loadingCommand
        );
    }
}