package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FrontTOFSubsystem;
import frc.robot.subsystems.MaserCannon;
import frc.robot.Constants;

/**
 * Drive forward until TOF sensor detects target at specified distance,
 * then execute reef alignment.
 */
public class DriveUntilDistance extends SequentialCommandGroup {
    
    // Request for robot-centric forward driving
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    
    /**
     * Creates a command that drives forward until the TOF sensor detects a target
     * at the specified distance, then executes reef alignment.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param tofSensor The Time of Flight sensor subsystem
     * @param maserSensor The LaserCan sensor subsystem for reef detection
     */
    public DriveUntilDistance(
            CommandSwerveDrivetrain drivetrain, 
            FrontTOFSubsystem tofSensor,
            MaserCannon maserSensor) {
        
        addCommands(
            // Log the start of the approach sequence
            Commands.runOnce(() -> System.out.println("Starting approach to reef")),
            
            // Drive forward until TOF sensor detects target at or closer than TARGET_DISTANCE_MM
            drivetrain.run(() -> {
                drivetrain.setControl(driveRequest
                    .withVelocityX(Constants.AUTO_DRIVE_SPEED)  // Forward at specified speed
                    .withVelocityY(0)            // No lateral movement
                    .withRotationalRate(0));     // Maintain heading
            }).until(() -> tofSensor.getFrontDistance() <= Constants.AUTO_TARGET_DISTANCE),
            
            // Stop the drivetrain
            drivetrain.runOnce(() -> drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0))),
                
            // Log that we've reached the target distance
            Commands.runOnce(() -> {
                System.out.println("Target distance reached: " + tofSensor.getFrontDistance() + " mm");
            })
        );
        
        addRequirements(drivetrain);
    }
    
} // End of Class