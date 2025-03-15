// AutoScoreCoralWithSensorCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EffectorConstants;
import frc.robot.subsystems.endEffector.EffectorSubsystem;

/**
 * A command that scores a coral and automatically stops when the coral has left the mechanism.
 * This command detects the transition from isCoralLoaded() true → false.
 */
public class AutoScoreCoral extends Command {
    private final EffectorSubsystem effector;
    private boolean wasLoaded = false;
    private boolean hasStarted = false;
    
    public AutoScoreCoral(EffectorSubsystem effector) {
        this.effector = effector;
        addRequirements(effector);
        // Add a safety timeout
        withTimeout(1.25);
    }
    
    @Override
    public void initialize() {
        // Capture initial state
        wasLoaded = effector.isCoralLoaded();
        hasStarted = false;
    }
    
    @Override
    public void execute() {
        // Run the motor to score
        effector.setEffectorOutput(EffectorConstants.SCORE_CORAL);
        
        // Mark that we've started execution
        hasStarted = true;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Always stop the motor when done
        effector.stopMotor();
    }
    
    @Override
    public boolean isFinished() {
        // Skip the first check to avoid immediate termination
        if (!hasStarted) {
            return false;
        }
        
        // Get current state
        boolean isCurrentlyLoaded = effector.isCoralLoaded();
        
        // Detect transition from loaded to not loaded
        boolean completed = wasLoaded && !isCurrentlyLoaded;
        
        // Update state for next check
        wasLoaded = isCurrentlyLoaded;
        
        return completed;
    }
}