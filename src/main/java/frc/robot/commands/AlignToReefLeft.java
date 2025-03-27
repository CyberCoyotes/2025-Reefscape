package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.MaserCannon;
import frc.robot.Constants;

/**
 * Command for aligning to reef branches using LaserCan sensor and swerve drivetrain.
 * Strafe left until we detect an opening (LaserCan distance exceeds threshold)
 */
public class AlignToReefLeft extends SequentialCommandGroup {
    
    // Request for robot-centric strafing
    private final SwerveRequest.RobotCentric strafeRequest = new SwerveRequest.RobotCentric();
    
    /**
     * Creates a new ReefBranchAlignCommand.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param maserSensor The LaserCan sensor subsystem
     */
    public AlignToReefLeft(CommandSwerveDrivetrain drivetrain, MaserCannon maserSensor) {
        // Step 1: Strafe left until we detect an opening
        addCommands(
            // Log the start of alignment process
            Commands.runOnce(() -> System.out.println("Starting reef alignment for left")),
            
            // Strafe left until LaserCan sensor detects an opening
            drivetrain.run(() -> {
                drivetrain.setControl(strafeRequest
                    .withVelocityX(0)
                    .withVelocityY(Constants.STRAFE_SPEED_LT) // 
                    .withRotationalRate(0));
            }).until(() -> maserSensor.getReefDistance() > Constants.YOU_SHALL_NOT_PASS),
            
            // Stop briefly
            drivetrain.runOnce(() -> drivetrain.setControl(strafeRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0))),
                
            // Log detection of opening
            Commands.runOnce(() -> {
                System.out.println("Opening detected at distance: " + maserSensor.getReefDistance());
            }),
    
            
            // Log completion
            Commands.runOnce(() -> System.out.println("Reef alignment complete"))
        );
        
        addRequirements(drivetrain);
    }
  
}