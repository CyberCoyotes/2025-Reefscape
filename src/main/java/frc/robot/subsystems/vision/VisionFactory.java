package frc.robot.subsystems.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TagAlignmentCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Factory class for creating vision-related subsystems and commands.
 */
public class VisionFactory {
    
    /**
     * Create a LimelightVisionSubsystem.
     * 
     * @param limelightName The name of the Limelight camera
     * @param drivetrain The swerve drivetrain
     * @return A new LimelightVisionSubsystem
     */
    public static LimelightVisionSubsystem createLimelightVisionSubsystem(
            String limelightName, 
            CommandSwerveDrivetrain drivetrain) {
        return new LimelightVisionSubsystem(limelightName, drivetrain);
    }
    
    /**
     * Create a command to align to the left side of a detected AprilTag.
     * 
     * @param vision The vision subsystem
     * @param drivetrain The swerve drivetrain
     * @param xSupplier Supplier for forward/backward movement
     * @param ySupplier Supplier for left/right movement
     * @param rotSupplier Supplier for rotation
     * @return A command for left-side alignment
     */
    public static Command createAlignToLeftCommand(
            LimelightVisionSubsystem vision,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier) {
        
        return TagAlignmentCommands.alignToTagLeftSide(
            drivetrain, 
            vision, 
            xSupplier, 
            ySupplier, 
            rotSupplier
        );
    }
    
    /**
     * Create a command to align to the right side of a detected AprilTag.
     * 
     * @param vision The vision subsystem
     * @param drivetrain The swerve drivetrain
     * @param xSupplier Supplier for forward/backward movement
     * @param ySupplier Supplier for left/right movement
     * @param rotSupplier Supplier for rotation
     * @return A command for right-side alignment
     */
    public static Command createAlignToRightCommand(
            LimelightVisionSubsystem vision,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier) {
        
        return TagAlignmentCommands.alignToTagRightSide(
            drivetrain, 
            vision, 
            xSupplier, 
            ySupplier, 
            rotSupplier
        );
    }
    
    /**
     * Create a command to center on a detected AprilTag.
     * 
     * @param vision The vision subsystem
     * @param drivetrain The swerve drivetrain
     * @param xSupplier Supplier for forward/backward movement
     * @param ySupplier Supplier for left/right movement
     * @param rotSupplier Supplier for rotation
     * @return A command for centering on a tag
     */
    public static Command createCenterCommand(
            LimelightVisionSubsystem vision,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier) {
        
        return TagAlignmentCommands.centerOnTag(
            drivetrain, 
            vision, 
            xSupplier, 
            ySupplier, 
            rotSupplier
        );
    }
    
    /**
     * Create a command to toggle the Limelight LEDs.
     * 
     * @param vision The vision subsystem
     * @return A command that toggles the LEDs
     */
    public static Command createToggleLEDsCommand(LimelightVisionSubsystem vision) {
        return Commands.runOnce(() -> {
            boolean hasTarget = vision.hasTarget();
            vision.setLeds(!hasTarget);
        });
    }
    
    /**
     * Create a sample implementation of a VisionIO using the Limelight.
     * 
     * @param limelightName The name of the Limelight camera
     * @return A new VisionIO implementation
     */
    public static VisionIO createLimelightVisionIO(String limelightName) {
        return new VisionIOLimelight(limelightName);
    }
}