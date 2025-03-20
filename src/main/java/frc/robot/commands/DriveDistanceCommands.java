package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Factory class for creating commands that move the robot specific distances.
 */
public class DriveDistanceCommands {
    private final CommandSwerveDrivetrain drivetrain;
    
    /**
     * Creates a new DriveDistanceCommands factory.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     */
    public DriveDistanceCommands(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    
    /**
     * Creates a command that drives the robot forward 0.15 meters from its current position,
     * regardless of its rotational orientation.
     * 
     * @return A command that drives forward 0.15 meters at the default speed (0.5 m/s)
     */
    public Command driveForward15cm() {
        return new DriveForwardCommand(drivetrain, 0.15);
    }
    
    /**
     * Creates a command that drives the robot forward 0.15 meters from its current position
     * at the specified speed.
     * 
     * @param speedMetersPerSecond The speed at which to drive (in meters per second)
     * @return A command that drives forward 0.15 meters at the specified speed
     */
    public Command driveForward15cm(double speedMetersPerSecond) {
        return new DriveForwardCommand(drivetrain, 0.15, speedMetersPerSecond);
    }
    
    /**
     * Creates a command that drives the robot forward by the specified distance.
     * 
     * @param distanceMeters The distance to drive in meters (positive for forward, negative for backward)
     * @return A command that drives the specified distance
     */
    public Command driveDistance(double distanceMeters) {
        return new DriveForwardCommand(drivetrain, distanceMeters);
    }
    
    /**
     * Creates a command that drives the robot forward by the specified distance at the specified speed.
     * 
     * @param distanceMeters The distance to drive in meters (positive for forward, negative for backward)
     * @param speedMetersPerSecond The speed at which to drive (always positive)
     * @return A command that drives the specified distance at the specified speed
     */
    public Command driveDistance(double distanceMeters, double speedMetersPerSecond) {
        return new DriveForwardCommand(drivetrain, distanceMeters, speedMetersPerSecond);
    }
    
    /**
     * Creates a command that drives the robot backward 0.15 meters from its current position,
     * regardless of its rotational orientation.
     * 
     * @return A command that drives backward 0.15 meters
     */
    public Command driveBackward15cm() {
        return new DriveForwardCommand(drivetrain, -0.15);
    }
}