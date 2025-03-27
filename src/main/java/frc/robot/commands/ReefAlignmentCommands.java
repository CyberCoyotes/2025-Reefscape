package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.MaserCannon;
import frc.robot.Constants;

/**
 * Factory class for creating reef alignment commands.
 * Provides methods to create commands that align the robot to reef branches
 * by strafing left or right until an opening is detected.
 */
public class ReefAlignmentCommands {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final MaserCannon maserSensor;
    
    /**
     * Creates a new ReefAlignmentCommands factory.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param maserSensor The LaserCan sensor subsystem for reef detection
     */
    public ReefAlignmentCommands(CommandSwerveDrivetrain drivetrain, MaserCannon maserSensor) {
        this.drivetrain = drivetrain;
        this.maserSensor = maserSensor;
    }
    
    /**
     * Creates a command that strafes the robot to the right until 
     * an opening is detected by the LaserCan sensor.
     * 
     * @return A command that aligns the robot to the reef by strafing right
     */
    public Command alignToReefRight() {
        return new AlignToReefRight(drivetrain, maserSensor);
    }
    
    /**
     * Creates a command that strafes the robot to the left until 
     * an opening is detected by the LaserCan sensor.
     * 
     * @return A command that aligns the robot to the reef by strafing left
     */
    public Command alignToReefLeft() {
        return new AlignToReefLeft(drivetrain, maserSensor);
    }
    
    /**
     * Creates a command that aligns to either the left or right reef branch
     * depending on the direction parameter.
     * 
     * @param strafeRight If true, aligns to the right; if false, aligns to the left
     * @return A command that aligns the robot to the reef in the specified direction
     */
    public Command alignToReef(boolean strafeRight) {
        return strafeRight ? alignToReefRight() : alignToReefLeft();
    }
    
    /**
     * Creates a command that attempts to find an opening by first strafing in the 
     * specified direction, and if that fails (times out), trying the opposite direction.
     * 
     * @param tryRightFirst If true, tries strafing right first; if false, tries left first
     * @param timeoutSeconds The timeout in seconds before trying the opposite direction
     * @return A command sequence that tries to find an opening in either direction
     */
    public Command findReefOpening(boolean tryRightFirst, double timeoutSeconds) {
        Command firstAttempt = tryRightFirst ? alignToReefRight() : alignToReefLeft();
        Command secondAttempt = tryRightFirst ? alignToReefLeft() : alignToReefRight();
        
        return Commands.sequence(
            // Try the first direction with a timeout
            Commands.race(
                firstAttempt,
                Commands.waitSeconds(timeoutSeconds)
            ),
            // If we're still not at an opening (timed out), try the other direction
            Commands.either(
                Commands.none(),
                secondAttempt,
                () -> maserSensor.getReefDistance() > Constants.YOU_SHALL_NOT_PASS
            )
        );
    }
}