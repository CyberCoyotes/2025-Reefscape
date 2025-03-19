package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command that drives the robot forward by a specific distance using time-based control.
 * This command uses robot-centric control to drive forward regardless of robot heading.
 */

@SuppressWarnings("unused")

public class DriveForwardCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final double distanceMeters;
    private final double speedMetersPerSecond;
    private final Timer timer = new Timer();
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private double timeToTravel;
    
    /**
     * Creates a command that drives the robot forward by the specified distance.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param distanceMeters The distance to drive in meters
     * @param speedMetersPerSecond The speed at which to drive
     */
    public DriveForwardCommand(CommandSwerveDrivetrain drivetrain, double distanceMeters, double speedMetersPerSecond) {
        this.drivetrain = drivetrain;
        this.distanceMeters = distanceMeters;
        this.speedMetersPerSecond = speedMetersPerSecond;
        
        // Calculate the time needed to travel the specified distance at the specified speed
        this.timeToTravel = Math.abs(distanceMeters / speedMetersPerSecond);
        
        addRequirements(drivetrain);
    }
    
    /**
     * Creates a command that drives the robot forward by the specified distance at 0.5 m/s.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param distanceMeters The distance to drive in meters
     */
    public DriveForwardCommand(CommandSwerveDrivetrain drivetrain, double distanceMeters) {
        this(drivetrain, distanceMeters, 0.5); // Default speed of 0.5 m/s
    }
    
    @Override
    public void initialize() {
        // Reset the timer when the command starts
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        // Drive forward using robot-centric control
        // This means "forward" is relative to the robot's current orientation
        double speed = distanceMeters >= 0 ? speedMetersPerSecond : -speedMetersPerSecond;
        drivetrain.setControl(robotCentricRequest
                .withVelocityX(speed) // Forward/backward motion
                .withVelocityY(0)     // Left/right motion
                .withRotationalRate(0)); // Maintain current heading
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        drivetrain.setControl(robotCentricRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        timer.stop();
    }
    
    @Override
    public boolean isFinished() {
        // Finish when we've driven for the calculated time
        return timer.hasElapsed(timeToTravel);
    }
}