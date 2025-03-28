package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.Subsystem;


import frc.robot.subsystems.ReefTOFSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/*
 * Similar to Align with Maser (LaserCAN) commands, but using Time of Flight sensor
 */
public class AlignToReefCommands {
    // Strafe speed constant
    private static final double STRAFE_RIGHT = 1.0; // meters per second, adjust as needed
    private static final double STRAFE_LEFT = -STRAFE_RIGHT; // meters per second, adjust as needed

    /**
     * Creates a command that strafes left until the reef is detected
     */
    public static Command strafeLeftToReef(ReefTOFSubsystem reefSensor, CommandSwerveDrivetrain drivetrain) {
        // Create the robot-centric swerve request for strafing
        SwerveRequest.RobotCentric strafeRequest = new SwerveRequest.RobotCentric();
        
        return new Command() {
            @Override
            public void initialize() {
                System.out.println("Starting strafe LEFT until reef detected");
            }
            
            @Override
            public void execute() {
                // Apply negative velocity for left strafing
                drivetrain.setControl(strafeRequest
                    .withVelocityX(0) // forward/back
                    .withVelocityY(STRAFE_LEFT)
                    .withRotationalRate(0)); // no rotation
            }
            
            @Override
            public boolean isFinished() {
                return reefSensor.atReefTarget();
            }
            
            @Override
            public void end(boolean interrupted) {
                // Stop the drivetrain when we're done
                drivetrain.setControl(strafeRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
                System.out.println("Reef detected, stopping LEFT strafe" + 
                    (interrupted ? " (interrupted)" : ""));
            }
            
            // TODO Remove if I want manual control to interrupt?
            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(drivetrain, reefSensor);
            }
        };
    }
    
    /**
     * Creates a command that strafes right until the reef is detected
     */
    public static Command strafeRightToReef(ReefTOFSubsystem reefSensor, CommandSwerveDrivetrain drivetrain) {
        // Create the robot-centric swerve request for strafing
        SwerveRequest.RobotCentric strafeRequest = new SwerveRequest.RobotCentric();
        
        return new Command() {
            @Override
            public void initialize() {
                System.out.println("Starting strafe RIGHT until reef detected");
            }
            
            @Override
            public void execute() {
                // Apply positive velocity for right strafing
                drivetrain.setControl(strafeRequest
                    .withVelocityX(0) // forward/back
                    .withVelocityY(STRAFE_RIGHT) //
                    .withRotationalRate(0)); // no rotation
            }
            
            @Override
            public boolean isFinished() {
                return reefSensor.atReefTarget();
            }
            
            @Override
            public void end(boolean interrupted) {
                // Stop the drivetrain when we're done
                drivetrain.setControl(strafeRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
                System.out.println("Reef detected, stopping RIGHT strafe" + 
                    (interrupted ? " (interrupted)" : ""));
            }
            
            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(drivetrain, reefSensor);
            }
        };
    }
}